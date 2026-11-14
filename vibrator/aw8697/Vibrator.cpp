// SPDX-License-Identifier: Apache-2.0
#include "Vibrator.h"
#include "PrimitiveSynth.h"
#include <dirent.h>
#include <fcntl.h>
#include <linux/input.h>
#include <log/log.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace aidl::android::hardware::vibrator {
namespace {
struct EffectMapping {
    Effect effect;
    int ramId;
    float base;  // strength at android default medium intensity
};
// pick waveforms by output, not sample energy: lra is narrow band
constexpr std::array<EffectMapping, 7> kEffects{{
        {Effect::CLICK, 0, 1},
        {Effect::DOUBLE_CLICK, 0, 1},
        {Effect::TICK, 2, 0.8f},
        {Effect::THUD, 1, 1},
        {Effect::POP, 4, 1},
        {Effect::HEAVY_CLICK, 0, 1},
        {Effect::TEXTURE_TICK, 5, 1},
}};
// custom wave effect id and the flag that marks a primitive waveform
constexpr int kRtpEffect = 193;
constexpr uint32_t kPrimitiveFlag = 1u << 15;
// android intensity scales per effect base
constexpr float kLightFactor = 0.6f;
constexpr float kStrongFactor = 1.6f;
// launcher and systemui taps arrive at scale 0.70, hyperos plays them full strength
constexpr float kCompositionGain = 1 / 0.7f;
// aw8697 sysfs nodes share one i2c address
std::string sysfsPath(const char* node) {
    return std::string("/sys/devices/platform/soc/a8c000.i2c/i2c-2/2-005a/") + node;
}
// find one "key=value;" field in status buffer
bool fieldValue(const char* info, const char* key, int* value) {
    const char* field = strstr(info, key);
    return field && sscanf(field, "%*[^=]=%d;", value) == 1;
}
// stock driver: 0..0x3fff means gain 30, (value-0x3fff)/128, linear amplitude collapses to gain 30
int encodeAmplitude(float amplitude) {
    const int gain = std::clamp(static_cast<int>(std::lround(amplitude * 128)), 30, 128);
    return gain == 128 ? 32767 : 16383 + gain * 128;
}
// ram id the effect table gives an effect
int effectRamId(Effect effect) {
    for (const auto& mapping : kEffects)
        if (mapping.effect == effect) return mapping.ramId;
    return -1;
}

// ram id of stock effect for primitive with no packaged waveform
int primitiveRam(CompositePrimitive primitive) {
    switch (primitive) {
        case CompositePrimitive::CLICK:
            return effectRamId(Effect::CLICK);
        case CompositePrimitive::LIGHT_TICK:
        case CompositePrimitive::LOW_TICK:
            return effectRamId(Effect::TICK);
        case CompositePrimitive::THUD:
            return effectRamId(Effect::THUD);
        default:
            return -1;
    }
}
ndk::ScopedAStatus error(int code) {
    return ndk::ScopedAStatus::fromExceptionCode(code);
}
}  // namespace

ndk::ScopedAStatus Vibrator::unsupported() {
    return error(EX_UNSUPPORTED_OPERATION);
}

Vibrator::Vibrator() {
    DIR* dir = opendir("/dev/input");
    if (!dir) return;
    while (auto* ent = readdir(dir)) {
        if (strncmp(ent->d_name, "event", 5)) continue;
        const std::string path = std::string("/dev/input/") + ent->d_name;
        const int fd = open(path.c_str(), O_RDWR | O_CLOEXEC);
        if (fd < 0) continue;
        char name[80]{};
        if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) >= 0 && !strcmp(name, "aw8697_haptic")) {
            mFd = fd;
            break;
        }
        close(fd);
    }
    closedir(dir);
    if (mFd < 0) {
        ALOGE("AW8697 haptic input device not found");
        return;
    }
    // kernel publishes positive period only after ram programming, no vibrator without firmware
    const std::string wavePath = sysfsPath("custom_wave");
    bool ramReady = false;
    for (int attempt = 0; attempt < 100; ++attempt) {
        const int metadataFd = open(wavePath.c_str(), O_RDONLY | O_CLOEXEC);
        if (metadataFd >= 0) {
            char info[256]{};
            const ssize_t n = TEMP_FAILURE_RETRY(read(metadataFd, info, sizeof(info) - 1));
            close(metadataFd);
            int period = 0;
            if (n > 0 && fieldValue(info, "period_size=", &period) && period > 0) {
                mDirectRamGain = strstr(info, "ram_gain_abi=1;") != nullptr;
                int maximum = 0;
                if (strstr(info, "abi_version=2;") && strstr(info, "playback_active=") &&
                    strstr(info, "custom_wave_id=193;") &&
                    fieldValue(info, "max_size=", &maximum) && maximum >= 4096 && period > 1 &&
                    period <= 2048) {
                    mWavePeriod = period;
                }
                ramReady = true;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!ramReady) {
        ALOGE("AW8697 RAM firmware readiness could not be confirmed");
        close(mFd);
        mFd = -1;
        return;
    }
    // query legacy three short abi without starting motor
    std::array<bool, kRamEffectCount> probed{};
    for (const auto& mapping : kEffects) {
        const int id = mapping.ramId;
        if (probed[id]) continue;
        probed[id] = true;
        if (!upload(id, 0, 1, &mDurations[id]) || mDurations[id] <= 0 || mDurations[id] > 1000 ||
            !stopLocked()) {
            close(mFd);
            mFd = -1;
            return;
        }
    }
    mGainFd = open(sysfsPath("gain").c_str(), O_WRONLY | O_CLOEXEC);
    // read cached boot measurement, reading f0 starts calibration, abi reports tenths of hertz
    if (FILE* f = fopen(sysfsPath("f0_value").c_str(), "re")) {
        unsigned int tenths = 0;
        if (fscanf(f, "%u", &tenths) == 1 && tenths >= 1000 && tenths <= 3000)
            mResonantFrequency = tenths / 10.0f;
        fclose(f);
    }
    if (mWavePeriod) {
        mWaveFd = open(wavePath.c_str(), O_WRONLY | O_CLOEXEC);
        mWaveStatusFd = open(wavePath.c_str(), O_RDONLY | O_CLOEXEC);
        if (mWaveFd >= 0 && mWaveStatusFd >= 0) {
            for (size_t i = 0; i < mEffectStreams.size(); ++i)
                mEffectStreams[i] = loadStream(static_cast<uint32_t>(kEffects[i].effect));
            // stock slices play undamped, ring down is most of tap
            for (size_t i = 1; i < mPrimitiveStreams.size(); ++i)
                mPrimitiveStreams[i] = loadStream(kPrimitiveFlag | i);
            // packaged files win over synthesized envelopes
            using alioth::haptics::Shape;
            constexpr std::pair<CompositePrimitive, Shape> kSynth[] = {
                    {CompositePrimitive::SPIN, Shape::kSpin},
                    {CompositePrimitive::QUICK_RISE, Shape::kQuickRise},
                    {CompositePrimitive::SLOW_RISE, Shape::kSlowRise},
                    {CompositePrimitive::QUICK_FALL, Shape::kQuickFall},
            };
            for (const auto& [primitive, shape] : kSynth) {
                const size_t index = static_cast<size_t>(primitive);
                if (mPrimitiveStreams[index]) continue;
                auto& owned = mOwned[index];
                owned.samples = alioth::haptics::synthesize(shape, mResonantFrequency);
                owned.stream = std::make_unique<effect_stream>(owned.samples.size(), 24000,
                                                               owned.samples.data());
                if (streamDuration(owned.stream.get()))
                    mPrimitiveStreams[index] = owned.stream.get();
            }
        }
    }
    mWorker = std::thread(&Vibrator::run, this);
}

Vibrator::~Vibrator() {
    {
        std::lock_guard lock(mMutex);
        mExit = true;
        ++mGeneration;
        mCv.notify_all();
    }
    if (mWorker.joinable()) mWorker.join();
    if (mFd >= 0) {
        stopLocked();
        close(mFd);
    }
    if (mGainFd >= 0) close(mGainFd);
    if (mWaveFd >= 0) close(mWaveFd);
    if (mWaveStatusFd >= 0) close(mWaveStatusFd);
}

// packaged waveform for an effect or primitive id
const effect_stream* Vibrator::loadStream(uint32_t id) {
    const auto* stream = get_effect_stream_exact(id);
    return streamDuration(stream) ? stream : nullptr;
}

// stream for a primitive, low tick falls back to the light tick
const effect_stream* Vibrator::primitiveStream(CompositePrimitive primitive) const {
    const int id = static_cast<int>(primitive);
    if (id < 0 || id >= static_cast<int>(mPrimitiveStreams.size())) return nullptr;
    const auto* stream = mPrimitiveStreams[id];
    if (!stream && primitive == CompositePrimitive::LOW_TICK)
        stream = mPrimitiveStreams[static_cast<int>(CompositePrimitive::LIGHT_TICK)];
    return stream;
}

// one trailing zero only signals eof for period aligned payload
size_t Vibrator::totalLength(const effect_stream& stream) const {
    return stream.length + (stream.length % mWavePeriod == 0);
}

// bound finite stream to ten seconds, refill never exceeds free space
int Vibrator::streamDuration(const effect_stream* stream) const {
    if (!mWavePeriod || !stream || !stream->data || stream->play_rate_hz != 24000 ||
        stream->length == 0 || stream->length > 240000)
        return 0;
    return (totalLength(*stream) + 23) / 24;  // nominal duration, completion follows hardware state
}

bool Vibrator::statusValue(const char* key, int* value) {
    char info[256]{};
    if (lseek(mWaveStatusFd, 0, SEEK_SET) < 0) return false;
    if (TEMP_FAILURE_RETRY(read(mWaveStatusFd, info, sizeof(info) - 1)) <= 0) return false;
    return fieldValue(info, key, value);
}

int Vibrator::waveActive() {
    int active = -1;
    return statusValue("playback_active=", &active) ? active : -1;
}

bool Vibrator::writeWave(const effect_stream& stream, float scale, size_t* offset) {
    // this worker is sole producer, consumer only frees space
    int available = 0;
    if (!statusValue("free_size=", &available) || available < 0) return false;
    const size_t total = totalLength(stream);
    if (*offset >= total) return *offset == total;
    // short or unaligned write signals eof, so align intermediate write
    const size_t chunkLimit = (4095 / mWavePeriod) * mWavePeriod;
    size_t count = std::min(total - *offset, chunkLimit);
    if (count > static_cast<size_t>(available)) count = (available / mWavePeriod) * mWavePeriod;
    if (!count) return true;
    mChunk.assign(count, 0);
    for (size_t i = 0; i < count && *offset + i < stream.length; ++i)
        mChunk[i] = static_cast<int8_t>(std::lround(stream.data[*offset + i] * scale));
    if (lseek(mWaveFd, 0, SEEK_SET) < 0 ||
        TEMP_FAILURE_RETRY(write(mWaveFd, mChunk.data(), count)) != static_cast<ssize_t>(count))
        return false;
    *offset += count;
    return true;
}

bool Vibrator::setGain(float amplitude) {
    if (mGainFd < 0) return event(FF_GAIN, encodeAmplitude(amplitude));
    // unlike ff_gain this abi has no gain plateau and updates saved gain, ram startup reuses it
    char value[16];
    int count = snprintf(value, sizeof(value), "%d",
                         std::clamp(static_cast<int>(std::lround(amplitude * 128)), 1, 128));
    if (lseek(mGainFd, 0, SEEK_SET) < 0) return false;
    return TEMP_FAILURE_RETRY(write(mGainFd, value, count)) == count;
}

bool Vibrator::event(unsigned short code, int value) {
    input_event e{};
    e.type = EV_FF;
    e.code = code;
    e.value = value;
    return TEMP_FAILURE_RETRY(write(mFd, &e, sizeof(e))) == sizeof(e);
}

bool Vibrator::stopLocked() {
    if (mEffect < 0) return true;
    const bool stopped = event(mEffect, 0);
    if (ioctl(mFd, EVIOCRMFF, mEffect) < 0) return false;
    mEffect = -1;
    return stopped;
}

bool Vibrator::upload(int effect, int duration, float scale, int* actual) {
    ff_effect e{};
    e.id = -1;
    int16_t data[4] = {static_cast<int16_t>(effect), 0, 0, 0};
    const bool directGain = mDirectRamGain && !actual && effect >= 0 && effect < kRamEffectCount;
    if (directGain)
        data[3] = 0x4700 | std::clamp(static_cast<int>(std::lround(scale * 128)), 0, 128);
    if (effect >= 0) {
        e.type = FF_PERIODIC;
        e.u.periodic.waveform = FF_CUSTOM;
        e.u.periodic.magnitude = static_cast<int16_t>(encodeAmplitude(scale));
        // legacy abi counts bytes, generic ff api counts samples
        e.u.periodic.custom_len = (directGain ? 4 : 3) * sizeof(int16_t);
        e.u.periodic.custom_data = data;
    } else {
        e.type = FF_CONSTANT;
        e.u.constant.level = static_cast<int16_t>(encodeAmplitude(scale));
        e.replay.length = duration;
    }
    if (ioctl(mFd, EVIOCSFF, &e) < 0) return false;
    mEffect = e.id;
    if (actual) *actual = effect >= 0 ? data[1] * 1000 + data[2] : duration;
    return true;
}

ndk::ScopedAStatus Vibrator::start(std::vector<Step> steps, std::shared_ptr<IVibratorCallback> cb) {
    std::unique_lock lock(mMutex);
    if (!ready()) return error(EX_ILLEGAL_STATE);
    ++mGeneration;
    mPending = false;
    auto previousCallback = std::move(mCallback);
    mCv.notify_all();
    const bool stopped = stopLocked();
    mActiveConstant = false;
    if (stopped) {
        mSteps = std::move(steps);
        mCallback = std::move(cb);
        mPending = true;
        mCv.notify_all();
    }
    // completion callback reports termination, never call back under the mutex, clients may reenter
    lock.unlock();
    if (previousCallback) previousCallback->onComplete();
    return stopped ? ndk::ScopedAStatus::ok() : error(EX_ILLEGAL_STATE);
}

void Vibrator::run() {
    std::unique_lock lock(mMutex);
    while (!mExit) {
        mCv.wait(lock, [&] { return mExit || mPending; });
        if (mExit) break;
        auto steps = std::move(mSteps);
        auto generation = mGeneration;
        mPending = false;
        auto wait = [&](int ms) {
            return !mCv.wait_for(lock, std::chrono::milliseconds(ms),
                                 [&] { return mExit || generation != mGeneration; });
        };
        for (auto s : steps) {
            size_t waveOffset = 0;
            if (s.delay > 0 && !wait(s.delay)) break;
            if (s.scale > 0 && s.duration > 0) {
                mActiveConstant = s.effect < 0;
                if (!upload(s.stream ? kRtpEffect : s.effect, s.duration, s.stream ? 1 : s.scale,
                            nullptr) ||
                    (s.stream && !writeWave(*s.stream, s.scale, &waveOffset)) ||
                    !setGain(s.effect < 0 ? mAmplitude : 1.0f) || !event(mEffect, 1)) {
                    ALOGE("AW8697 effect playback failed: %s", strerror(errno));
                    stopLocked();
                    break;
                }
            }
            if (s.stream && s.scale > 0) {
                const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(s.duration + 2000);
                const size_t total = totalLength(*s.stream);
                bool failed = false;
                while (std::chrono::steady_clock::now() < deadline) {
                    const int active = waveActive();
                    if (active < 0 || (!active && waveOffset < total)) {
                        failed = true;
                        break;
                    }
                    if (waveOffset == total && !active) break;
                    if (waveOffset < total && !writeWave(*s.stream, s.scale, &waveOffset)) {
                        failed = true;
                        break;
                    }
                    if (!wait(2)) break;
                }
                if (mExit || generation != mGeneration) break;
                failed |= waveOffset < total || waveActive() != 0;
                if (failed) {
                    ALOGE("AW8697 RTP refill, completion or status failure");
                    stopLocked();
                    break;
                }
            } else if (!wait(s.duration)) {
                break;
            }
            if (!stopLocked()) {
                ALOGE("AW8697 stop failed");
                break;
            }
        }
        if (generation == mGeneration) {
            mActiveConstant = false;
            // onComplete has no success result, notify also after async failure
            auto callback = std::move(mCallback);
            if (!mExit && callback) {
                lock.unlock();
                callback->onComplete();
                lock.lock();
            }
        }
    }
}

ndk::ScopedAStatus Vibrator::getCapabilities(int32_t* out) {
    *out = CAP_ON_CALLBACK | CAP_PERFORM_CALLBACK;
    if (mGainFd >= 0) *out |= CAP_AMPLITUDE_CONTROL;
    *out |= CAP_COMPOSE_EFFECTS;
    if (mResonantFrequency > 0) *out |= CAP_GET_RESONANT_FREQUENCY;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::off() {
    std::unique_lock lock(mMutex);
    ++mGeneration;
    mPending = false;
    auto callback = std::move(mCallback);
    mCv.notify_all();
    const bool stopped = stopLocked();
    mActiveConstant = false;
    mAmplitude = 1;
    lock.unlock();
    if (callback) callback->onComplete();
    return stopped ? ndk::ScopedAStatus::ok() : error(EX_ILLEGAL_STATE);
}

ndk::ScopedAStatus Vibrator::on(int32_t ms, const std::shared_ptr<IVibratorCallback>& cb) {
    if (ms <= 0) return error(EX_ILLEGAL_ARGUMENT);
    // linux input replay.length is 16 bit, so split long request
    std::vector<Step> steps;
    while (ms > 0) {
        const int duration = std::min(ms, 65535);
        steps.push_back({-1, duration, 1, 0});
        ms -= duration;
    }
    return start(std::move(steps), cb);
}

ndk::ScopedAStatus Vibrator::perform(Effect e, EffectStrength strength,
                                     const std::shared_ptr<IVibratorCallback>& cb, int32_t* out) {
    *out = 0;
    const auto mapping =
            std::find_if(kEffects.begin(), kEffects.end(),
                         [e](const EffectMapping& entry) { return entry.effect == e; });
    if (mapping == kEffects.end()) return unsupported();
    const int id = mapping->ramId;
    float scale;
    switch (strength) {
        // stock playEffect drives magnitude 16384 * strength + 16383, gain 128 * strength
        case EffectStrength::LIGHT:
            scale = mapping->base * kLightFactor;
            break;
        case EffectStrength::MEDIUM:
            scale = mapping->base;
            break;
        case EffectStrength::STRONG:
            scale = std::min(1.0f, mapping->base * kStrongFactor);
            break;
        default:
            return unsupported();
    }
    const auto* stream = mEffectStreams[std::distance(kEffects.begin(), mapping)];
    if (stream) {
        const int duration = streamDuration(stream);
        auto status = start({{kRtpEffect, duration, scale, 0, stream}}, cb);
        if (status.isOk()) *out = duration;
        return status;
    }
    // without effect_1.bin stock ram 1 packs two pulses, reads as one tap
    if (e == Effect::DOUBLE_CLICK) {
        constexpr int kDoubleClickGapMs = 80;
        auto status = start({{id, mDurations[id], scale, 0},
                             {id, mDurations[id], scale, kDoubleClickGapMs}},
                            cb);
        if (status.isOk()) *out = 2 * mDurations[id] + kDoubleClickGapMs;
        return status;
    }
    auto status = start({{id, mDurations[id], scale, 0}}, cb);
    if (status.isOk()) *out = mDurations[id];
    return status;
}

ndk::ScopedAStatus Vibrator::getSupportedEffects(std::vector<Effect>* out) {
    out->clear();
    for (const auto& mapping : kEffects) out->push_back(mapping.effect);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::setAmplitude(float amplitude) {
    if (!std::isfinite(amplitude) || amplitude <= 0 || amplitude > 1)
        return error(EX_ILLEGAL_ARGUMENT);
    std::lock_guard lock(mMutex);
    if (mGainFd < 0) return unsupported();
    // setAmplitude applies to on() only, also next one, gain change corrupts a predefined effect
    if (mActiveConstant && !setGain(amplitude)) return error(EX_ILLEGAL_STATE);
    mAmplitude = amplitude;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::getCompositionDelayMax(int32_t* out) {
    *out = 1000;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::getCompositionSizeMax(int32_t* out) {
    *out = 256;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::getSupportedPrimitives(std::vector<CompositePrimitive>* out) {
    out->clear();
    out->push_back(CompositePrimitive::NOOP);
    for (size_t i = 1; i < mPrimitiveStreams.size(); ++i) {
        auto primitive = static_cast<CompositePrimitive>(i);
        if (mPrimitiveStreams[i] || primitiveRam(primitive) >= 0) out->push_back(primitive);
    }
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::getPrimitiveDuration(CompositePrimitive primitive, int32_t* out) {
    *out = 0;
    if (primitive == CompositePrimitive::NOOP) return ndk::ScopedAStatus::ok();
    if (const auto* stream = primitiveStream(primitive)) {
        *out = streamDuration(stream);
        return ndk::ScopedAStatus::ok();
    }
    if (const int ram = primitiveRam(primitive); ram >= 0) {
        *out = mDurations[ram];
        return ndk::ScopedAStatus::ok();
    }
    return unsupported();
}

ndk::ScopedAStatus Vibrator::compose(const std::vector<CompositeEffect>& composite,
                                     const std::shared_ptr<IVibratorCallback>& callback) {
    if (composite.empty() || composite.size() > 256) return error(EX_ILLEGAL_ARGUMENT);
    std::vector<Step> steps;
    for (const auto& effect : composite) {
        if (effect.delayMs < 0 || effect.delayMs > 1000 || !std::isfinite(effect.scale) ||
            effect.scale < 0 || effect.scale > 1)
            return error(EX_ILLEGAL_ARGUMENT);
        int duration;
        auto status = getPrimitiveDuration(effect.primitive, &duration);
        if (!status.isOk()) return status;
        const auto* stream = primitiveStream(effect.primitive);
        steps.push_back({stream ? kRtpEffect : primitiveRam(effect.primitive), duration,
                         std::min(1.0f, effect.scale * kCompositionGain), effect.delayMs, stream});
    }
    return start(std::move(steps), callback);
}

ndk::ScopedAStatus Vibrator::getResonantFrequency(float* out) {
    if (mResonantFrequency <= 0) return unsupported();
    *out = mResonantFrequency;
    return ndk::ScopedAStatus::ok();
}
}  // namespace aidl::android::hardware::vibrator

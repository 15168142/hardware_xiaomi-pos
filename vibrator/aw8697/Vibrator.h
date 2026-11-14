// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <aidl/android/hardware/vibrator/BnVibrator.h>
#include <effect.h>
#include <array>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace aidl::android::hardware::vibrator {
class Vibrator final : public BnVibrator {
  public:
    Vibrator();
    ~Vibrator() override;
    bool ready() const { return mFd >= 0; }
    ndk::ScopedAStatus getCapabilities(int32_t* _aidl_return) override;
    ndk::ScopedAStatus off() override;
    ndk::ScopedAStatus on(int32_t timeoutMs,
                          const std::shared_ptr<IVibratorCallback>& callback) override;
    ndk::ScopedAStatus perform(Effect effect, EffectStrength strength,
                               const std::shared_ptr<IVibratorCallback>& callback,
                               int32_t* _aidl_return) override;
    ndk::ScopedAStatus getSupportedEffects(std::vector<Effect>* _aidl_return) override;
    ndk::ScopedAStatus setAmplitude(float amplitude) override;
    ndk::ScopedAStatus getCompositionDelayMax(int32_t* maxDelayMs) override;
    ndk::ScopedAStatus getCompositionSizeMax(int32_t* maxSize) override;
    ndk::ScopedAStatus getSupportedPrimitives(std::vector<CompositePrimitive>* supported) override;
    ndk::ScopedAStatus getPrimitiveDuration(CompositePrimitive primitive,
                                            int32_t* durationMs) override;
    ndk::ScopedAStatus compose(const std::vector<CompositeEffect>& composite,
                               const std::shared_ptr<IVibratorCallback>& callback) override;
    ndk::ScopedAStatus getResonantFrequency(float* resonantFreqHz) override;

    // chip has no always-on, pwle or external control path
    ndk::ScopedAStatus setExternalControl(bool) override { return unsupported(); }
    ndk::ScopedAStatus getSupportedAlwaysOnEffects(std::vector<Effect>*) override {
        return unsupported();
    }
    ndk::ScopedAStatus alwaysOnEnable(int32_t, Effect, EffectStrength) override {
        return unsupported();
    }
    ndk::ScopedAStatus alwaysOnDisable(int32_t) override { return unsupported(); }
    ndk::ScopedAStatus getQFactor(float*) override { return unsupported(); }
    ndk::ScopedAStatus getFrequencyResolution(float*) override { return unsupported(); }
    ndk::ScopedAStatus getFrequencyMinimum(float*) override { return unsupported(); }
    ndk::ScopedAStatus getBandwidthAmplitudeMap(std::vector<float>*) override {
        return unsupported();
    }
    ndk::ScopedAStatus getPwlePrimitiveDurationMax(int32_t*) override { return unsupported(); }
    ndk::ScopedAStatus getPwleCompositionSizeMax(int32_t*) override { return unsupported(); }
    ndk::ScopedAStatus getSupportedBraking(std::vector<Braking>*) override { return unsupported(); }
    ndk::ScopedAStatus composePwle(const std::vector<PrimitivePwle>&,
                                   const std::shared_ptr<IVibratorCallback>&) override {
        return unsupported();
    }

  private:
    struct Step {
        int effect;
        int duration;
        float scale;
        int delay;
        const effect_stream* stream = nullptr;
    };
    // synthesized waveform keeps samples next to its stream
    struct Owned {
        std::vector<int8_t> samples;
        std::unique_ptr<effect_stream> stream;
    };
    static ndk::ScopedAStatus unsupported();
    ndk::ScopedAStatus start(std::vector<Step> steps, std::shared_ptr<IVibratorCallback> callback);
    bool writeWave(const effect_stream& stream, float scale, size_t* offset);
    bool statusValue(const char* key, int* value);
    int streamDuration(const effect_stream* stream) const;
    size_t totalLength(const effect_stream& stream) const;
    const effect_stream* loadStream(uint32_t id);
    const effect_stream* primitiveStream(CompositePrimitive primitive) const;
    bool upload(int effect, int duration, float scale, int* actualDuration);
    bool stopLocked();
    bool setGain(float amplitude);
    bool event(unsigned short code, int value);
    void run();
    int mFd = -1, mEffect = -1;
    int mGainFd = -1;
    int mWaveFd = -1;
    int mWaveStatusFd = -1;
    int waveActive();
    unsigned int mWavePeriod = 0;
    std::array<const effect_stream*, 7> mEffectStreams{};
    std::array<const effect_stream*, 9> mPrimitiveStreams{};
    std::array<Owned, 9> mOwned;
    std::vector<int8_t> mChunk;
    bool mDirectRamGain = false;
    static constexpr int kRamEffectCount = 10;
    std::array<int, kRamEffectCount> mDurations{};
    std::mutex mMutex;
    std::condition_variable mCv;
    std::thread mWorker;
    bool mExit = false, mPending = false;
    bool mActiveConstant = false;
    float mAmplitude = 1;
    float mResonantFrequency = 0;
    uint64_t mGeneration = 0;
    std::vector<Step> mSteps;
    std::shared_ptr<IVibratorCallback> mCallback;
};
}  // namespace aidl::android::hardware::vibrator

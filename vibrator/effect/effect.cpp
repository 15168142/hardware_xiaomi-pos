/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 * Copyright (C) 2022-2023 The LineageOS Project
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Changes from Qualcomm Innovation Center are provided under the following license:
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "libqtivibratoreffect.xiaomi"

#include <android-base/logging.h>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "effect.h"

namespace {

const uint32_t kDefaultPlayRateHz = 24000;
const uint16_t kPrimitiveMask = (1 << 15);

std::mutex sEffectMutex;
std::unordered_map<uint32_t, effect_stream> sExactStreams;
std::unordered_map<uint32_t, std::vector<int8_t>> sFifoData;

std::unique_ptr<effect_stream> readEffectStreamFromFile(uint32_t uniqueEffectId) {
    std::filesystem::path filePath;

    uint32_t effectId = uniqueEffectId & ~kPrimitiveMask;

    if ((uniqueEffectId & kPrimitiveMask) != 0) {
        filePath = "/vendor/etc/vibrator/primitive_effect_" + std::to_string(effectId) + ".bin";
    } else {
        filePath = "/vendor/etc/vibrator/effect_" + std::to_string(effectId) + ".bin";
    }

    LOG(VERBOSE) << "Reading fifo data for effect " << effectId << " from " << filePath;

    std::ifstream data(filePath, std::ios::in | std::ios::binary);
    if (!data.is_open()) {
        LOG(ERROR) << "Failed to open " << filePath << " for effect " << effectId;
        return nullptr;
    }

    std::error_code error;
    const auto fileSize = std::filesystem::file_size(filePath, error);
    // bound allocation, reject empty or truncated file before cache
    if (error || fileSize == 0 || fileSize > 240000) return nullptr;

    std::vector<int8_t> fifoData(fileSize);
    if (!data.read(reinterpret_cast<char*>(fifoData.data()), fileSize)) return nullptr;

    auto result = sFifoData.emplace(uniqueEffectId, std::move(fifoData));

    return std::make_unique<effect_stream>(result.first->second.size(), kDefaultPlayRateHz,
                                           result.first->second.data());
}

}  // namespace

// exact lookup for capability discovery, never turn missing primitive to click
const struct effect_stream* get_effect_stream_exact(uint32_t effectId) {
    std::lock_guard lock(sEffectMutex);
    auto it = sExactStreams.find(effectId);
    if (it != sExactStreams.end()) return &it->second;
    auto stream = readEffectStreamFromFile(effectId);
    if (!stream) return nullptr;
    return &sExactStreams.emplace(effectId, *stream).first->second;
}

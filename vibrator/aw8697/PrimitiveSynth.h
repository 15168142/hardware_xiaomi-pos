// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cstdint>
#include <vector>

namespace alioth::haptics {
// envelope primitives with no stock ram or prebaked waveform
enum class Shape { kQuickRise, kSlowRise, kQuickFall, kSpin };
// sustained drive rings 50 to 70 ms, so end each envelope with damping
std::vector<int8_t> synthesize(Shape shape, float resonanceHz);
}  // namespace alioth::haptics

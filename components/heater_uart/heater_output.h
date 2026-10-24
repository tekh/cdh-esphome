#pragma once

// Normalized output domain — the ONE place where fuel requests are converted
// between the controller's domains:
//   normalized [0.0 .. 1.0]  ← shared by ControlPolicy/SafetyPolicy (Phase 2/3)
//   pump Hz                 ← BDAP-class hardware (real fuel pump frequency)
//   power level 1..10       ← Vevor-class hardware (the one fuel dial)
//
// Before this header existed, the level↔Hz↔ratio mapping was hand-copied in
// FOUR places (set_power_level, get_power_level_display, and twice in
// send_vevor_frame) and had to be kept in sync by discipline. Now it's here,
// once, host-tested (tests/test_output_domain.cpp).
//
// Pure C++: no ESPHome includes, compiles on the host for the test harness.
// heater_profile.h is likewise pure (only <cstdint>).

#include <algorithm>
#include <cstdint>

#include "heater_profile.h"

namespace esphome {
namespace heater_uart {
namespace output_domain {

// Clamp a fuel fraction to the valid [0.0, 1.0] range.
inline float clamp01(float n) { return std::max(0.0f, std::min(1.0f, n)); }

// Power level 1..10 -> normalized fraction.
// Out-of-range levels are clamped to the dial's ends.
inline float normalized_from_level(uint8_t level) {
  if (level < 1) level = 1;
  if (level > 10) level = 10;
  return clamp01((static_cast<float>(level) - 1.0f) / 9.0f);
}

// Normalized fraction -> power level 1..10 (nearest level).
inline uint8_t level_from_normalized(float n) {
  n = clamp01(n);
  return static_cast<uint8_t>(1 + static_cast<int>(n * 9.0f + 0.5f));
}

// Pump Hz (profile's fuel range) -> normalized fraction.
// Degenerate profiles (min >= max) map to 0.0 deterministically — never a
// division blow-up on the bus.
inline float normalized_from_hz(float hz, const HeaterProfile &p) {
  const float span = p.pump_freq_max - p.pump_freq_min;
  if (span <= 0.0f) return 0.0f;
  return clamp01((hz - p.pump_freq_min) / span);
}

// Normalized fraction -> pump Hz (profile's fuel range).
inline float hz_from_normalized(float n, const HeaterProfile &p) {
  n = clamp01(n);
  return p.pump_freq_min + n * (p.pump_freq_max - p.pump_freq_min);
}

}  // namespace output_domain
}  // namespace heater_uart
}  // namespace esphome

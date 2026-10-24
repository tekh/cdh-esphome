#include "tiny_test.h"

// Profile sanity tests — guard the invariants every current and future preset
// must hold. Adding heater #7 means adding a profile; these tests make a
// structurally broken profile fail at `make -C tests` time, not on the bench.

#include "heater_profile.h"

using esphome::heater_uart::HeaterModel;
using esphome::heater_uart::HeaterProfile;
using esphome::heater_uart::profile_for;

static const HeaterModel ALL_MODELS[] = {
    HeaterModel::JEABONG_8KW,
    HeaterModel::VEVOR_2KW,
    HeaterModel::VEVOR_2KW_V2,
};

TEST(profile_pump_range_is_sane) {
  for (const HeaterModel m : ALL_MODELS) {
    const HeaterProfile &p = profile_for(m);
    CHECK(p.pump_freq_min < p.pump_freq_max);
    CHECK(p.pump_freq_step > 0.0f);
    CHECK(p.pump_freq_initial >= p.pump_freq_min);
    CHECK(p.pump_freq_initial <= p.pump_freq_max);
    CHECK(p.pump_prime_freq >= p.pump_freq_min);
    CHECK(p.pump_prime_freq <= p.pump_freq_max);
  }
}

TEST(profile_hx_thresholds_are_ordered) {
  // soot floor < nominal < too-hot < critical — an unordered profile silently
  // disables one of the safety overrides.
  for (const HeaterModel m : ALL_MODELS) {
    const HeaterProfile &p = profile_for(m);
    CHECK(p.hx_temp_low < p.hx_temp_target);
    CHECK(p.hx_temp_target < p.hx_temp_high);
    CHECK(p.hx_temp_high < p.hx_temp_critical);
    CHECK(p.cooldown_target_temp < p.hx_temp_low);
  }
}

TEST(profile_fan_range_is_sane) {
  for (const HeaterModel m : ALL_MODELS) {
    const HeaterProfile &p = profile_for(m);
    CHECK(p.fan_rpm_min < p.fan_rpm_max);
    CHECK(p.ignition_fan_rpm >= p.fan_rpm_min);
    CHECK(p.ignition_fan_rpm <= p.fan_rpm_max);
    CHECK(p.cooldown_fan_rpm >= p.fan_rpm_min);
    CHECK(p.cooldown_fan_rpm <= p.fan_rpm_max);
  }
}

TEST(profile_temp_range_is_sane) {
  for (const HeaterModel m : ALL_MODELS) {
    const HeaterProfile &p = profile_for(m);
    CHECK(p.temp_min < p.temp_max);
  }
}

TEST(profile_one_fuel_dial_only) {
  // ISSUES.md #3 (DECIDED): exactly one fuel dial per heater, enforced by the
  // capability flags. Both true would allow two live controls to fight; both
  // false would leave the heater with no fuel control at all.
  for (const HeaterModel m : ALL_MODELS) {
    const HeaterProfile &p = profile_for(m);
    CHECK(p.has_pump_hz_control != p.has_power_level_control);
  }
}

TEST(profile_v2_differs_from_v1_only_in_policy) {
  // vevor_2kw_v2 is a POLICY variant of the same hardware (decision #4): the
  // tuning numbers and capability flags must be identical to v1, so switching
  // the YAML line changes only who owns the HEAT level.
  const HeaterProfile &v1 = profile_for(HeaterModel::VEVOR_2KW);
  const HeaterProfile &v2 = profile_for(HeaterModel::VEVOR_2KW_V2);
  CHECK(v1.heat_mode_manual == false);
  CHECK(v2.heat_mode_manual == true);
  CHECK(v1.protocol == v2.protocol);
  CHECK(v1.uart_baud_rate == v2.uart_baud_rate);
  CHECK(v1.uart_inverted == v2.uart_inverted);
  CHECK(v1.pump_freq_min == v2.pump_freq_min);
  CHECK(v1.pump_freq_max == v2.pump_freq_max);
  CHECK(v1.pump_freq_step == v2.pump_freq_step);
  CHECK(v1.pump_freq_initial == v2.pump_freq_initial);
  CHECK(v1.hx_temp_low == v2.hx_temp_low);
  CHECK(v1.hx_temp_critical == v2.hx_temp_critical);
  CHECK(v1.temp_min == v2.temp_min);
  CHECK(v1.temp_max == v2.temp_max);
  CHECK(v1.has_pump_hz_control == v2.has_pump_hz_control);
  CHECK(v1.has_power_level_control == v2.has_power_level_control);
  CHECK(v1.has_fan_rpm_control == v2.has_fan_rpm_control);
}

#include "tiny_test.h"

// Profile sanity tests — guard the invariants every current and future preset
// must hold. Adding heater #7 means adding a profile; these tests make a
// structurally broken profile fail at `make -C tests` time, not on the bench.

#include "heater_profile.h"

using esphome::heater_uart::HeaterModel;
using esphome::heater_uart::HeaterProfile;
using esphome::heater_uart::ProfileValidationReport;
using esphome::heater_uart::profile_for;
using esphome::heater_uart::validate_profile_invariants;

static HeaterProfile copy(HeaterModel m) { return profile_for(m); }

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

// --- Phase 4: override validation (self-healing to preset) ---

TEST(validation_clean_profile_untouched) {
  for (const HeaterModel m : ALL_MODELS) {
    HeaterProfile p = copy(m);
    const HeaterProfile &preset = profile_for(m);
    const auto r = validate_profile_invariants(p, preset);
    CHECK(r.clean() == true);
    CHECK(p.pump_freq_min == preset.pump_freq_min);
    CHECK(p.hx_temp_critical == preset.hx_temp_critical);
  }
}

TEST(validation_reverts_degenerate_pump_range) {
  HeaterProfile p = copy(HeaterModel::VEVOR_2KW);
  p.pump_freq_min = 3.0f;  // >= max 2.5 -> degenerate
  p.pump_prime_freq = 99.0f;  // outside any range
  const auto r = validate_profile_invariants(p, profile_for(HeaterModel::VEVOR_2KW));
  CHECK(r.pump_range_reverted == true);
  CHECK_NEAR(p.pump_freq_min, 0.5f, 1e-6);
  CHECK_NEAR(p.pump_freq_max, 2.5f, 1e-6);
  CHECK_NEAR(p.pump_prime_freq, 2.5f, 1e-6);
  CHECK(r.hx_chain_reverted == false);
}

TEST(validation_reverts_initial_outside_range) {
  HeaterProfile p = copy(HeaterModel::JEABONG_8KW);
  p.pump_freq_initial = 99.0f;
  const auto r = validate_profile_invariants(p, profile_for(HeaterModel::JEABONG_8KW));
  CHECK(r.pump_range_reverted == true);
  CHECK_NEAR(p.pump_freq_initial, 1.8f, 1e-6);
}

TEST(validation_reverts_broken_hx_chain) {
  HeaterProfile p = copy(HeaterModel::VEVOR_2KW_V2);
  p.hx_temp_high = 100.0f;  // < target 180 -> chain broken
  const auto r = validate_profile_invariants(p, profile_for(HeaterModel::VEVOR_2KW_V2));
  CHECK(r.hx_chain_reverted == true);
  CHECK_NEAR(p.hx_temp_low, 145.0f, 1e-6);
  CHECK_NEAR(p.hx_temp_target, 180.0f, 1e-6);
  CHECK_NEAR(p.hx_temp_high, 190.0f, 1e-6);
  CHECK_NEAR(p.hx_temp_critical, 205.0f, 1e-6);
}

TEST(validation_reverts_degenerate_fan_and_temp) {
  HeaterProfile p = copy(HeaterModel::JEABONG_8KW);
  p.fan_rpm_min = 5000;  // >= max
  p.temp_min = 30; p.temp_max = 10;
  const auto r = validate_profile_invariants(p, profile_for(HeaterModel::JEABONG_8KW));
  CHECK(r.fan_range_reverted == true);
  CHECK(r.temp_range_reverted == true);
  CHECK(p.fan_rpm_min == 1450);
  CHECK(p.temp_min == 0);
  CHECK(p.temp_max == 30);
}

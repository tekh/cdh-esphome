#pragma once

#include <cstdint>

namespace esphome {
namespace heater_uart {

// Heater model selection (mirrors the `heater_model:` YAML option).
// One entry per supported heater hardware.
enum class HeaterModel : uint8_t {
  JEABONG_8KW = 0,  // 8 kW Jeabong heater (original reference hardware)
  VEVOR_2KW = 1,    // 2 kW Vevor heater (replacement hardware)
  VEVOR_2KW_V2 = 2, // 2 kW Vevor, EXPERIMENTAL: user owns the level in HEAT mode
};

// Which link-layer protocol the heater speaks.
//  - BDAP:      classic Chinese diesel heater bus, 0x76/0x16 24-byte frames, 25000 baud
//  - VEVOR_UART: Vevor (XMZ-D2 etc.) bus, 0xAA/0x66/0x77 frames, 4800 baud, additive checksum
enum class HeaterProtocol : uint8_t {
  BDAP = 0,
  VEVOR_UART = 1,
};

// All heater-specific tuning parameters live in one place so the component
// can support different heaters without any code changes. Each model gets a
// preset profile; refining a value only requires editing the profile, never
// the control logic (which reads exclusively from `profile_`).
struct HeaterProfile {
  // --- identity ---
  const char *name;
  HeaterModel model;
  HeaterProtocol protocol;  // UART framing/baud family used by this heater
  uint32_t uart_baud_rate;  // required uart: baud_rate (must match YAML uart block)
  bool uart_inverted;       // required pin inversion (Vevor bus is inverted logic)

  // --- protocol constants injected into the TX frame ---
  uint8_t fan_sensor;  // TX byte 12 (fan sensor type, e.g. 0x01 = SN-1)
  uint8_t glow_power;  // TX byte 13 (glow plug power setting)

  // --- fuel (dosing) pump frequency, Hz ---
  float pump_freq_min;      // absolute minimum pump rate (soot-safe floor)
  float pump_freq_max;      // absolute maximum pump rate (flame ceiling)
  float pump_freq_step;     // thermostat adjustment step
  float pump_freq_initial;  // ignition start rate
  float pump_prime_freq;    // rate used during fuel priming (pump-only)

  // --- combustion fan, RPM ---
  uint16_t fan_rpm_min;          // fan speed at minimum pump rate
  uint16_t fan_rpm_max;          // fan speed at maximum pump rate
  uint16_t ignition_fan_rpm;     // fan speed while flame stabilises (state 4)
  float ignition_hx_threshold;   // °C: below this, fan stays at ignition speed during state 4
  uint16_t cooldown_fan_rpm;     // fan speed during cooldown

  // --- heat exchanger safety thresholds, °C ---
  float hx_temp_low;          // below this, thermostat may not reduce pump (soot prevention)
  float hx_temp_target;       // nominal operating temperature
  float hx_temp_high;         // at/above this, thermostat may not raise pump
  float hx_temp_critical;     // above this: emergency shutdown
  float cooldown_target_temp; // HX °C at which cooldown fan stops

  // --- control authority ---
  // HEAT mode: when true, the user's level is sent verbatim in stable
  // combustion (state 5) and the firmware does NOT modulate it - no approach
  // control and no HX-soot pump override ("HEAT is where the user takes over
  // the fine tuning"). The ignition warm-up ramp (state 4), the HX-too-hot
  // decrease and the HX-critical stop still apply - those are protection, not
  // tuning. AUTO mode is unaffected either way.
  bool heat_mode_manual;

  // True when the heater's ECM owns combustion safety (self-primes, reports
  // E-codes, protects its own HX - Vevor-class). Only on such profiles can
  // the owner disable the firmware's combustion oversight in YAML
  // (`firmware_safety_oversight: false`). On firmware-safety hardware (BDAP
  // class) the stack is always on - the firmware is the safety authority.
  bool ecm_owned_safety;

  // --- which dials this hardware actually has ---
  // One fuel dial per heater, never both: BDAP takes a real pump frequency
  // (Hz), the Vevor takes a power level (1-10). The entity guards use these so
  // a control the profile does not support cannot be driven even if it is
  // declared in the YAML. has_fan_rpm_control = the protocol carries a fan
  // command (BDAP only; the Vevor has no fan field at all).
  bool has_pump_hz_control;
  bool has_power_level_control;
  bool has_fan_rpm_control;

  // --- temperature setpoint range, °C ---
  uint8_t temp_min;
  uint8_t temp_max;
};

// ---------------------------------------------------------------------------
// Presets
// ---------------------------------------------------------------------------

// 8 kW Jeabong heater - the original reference hardware.
// Values are the previously hardcoded constants, moved here verbatim.
inline HeaterProfile profile_jeabong_8kw() {
  HeaterProfile p{};
  p.name = "Jeabong 8kW";
  p.model = HeaterModel::JEABONG_8KW;
  p.protocol = HeaterProtocol::BDAP;
  p.uart_baud_rate = 25000;
  p.uart_inverted = false;
  p.fan_sensor = 0x01;   // SN-1
  p.glow_power = 0x05;
  p.pump_freq_min = 1.3f;
  p.pump_freq_max = 5.5f;
  p.pump_freq_step = 0.1f;
  p.pump_freq_initial = 1.8f;
  p.pump_prime_freq = 5.0f;
  p.fan_rpm_min = 1450;
  p.fan_rpm_max = 4500;
  p.ignition_fan_rpm = 2000;
  p.ignition_hx_threshold = 100.0f;
  p.cooldown_fan_rpm = 4000;
  p.hx_temp_low = 190.0f;
  p.hx_temp_target = 250.0f;
  p.hx_temp_high = 255.0f;
  p.hx_temp_critical = 265.0f;
  p.cooldown_target_temp = 60.0f;
  p.temp_min = 0;
  p.temp_max = 30;
  p.heat_mode_manual = false;  // HEAT modulates like AUTO (original behaviour)
  p.ecm_owned_safety = false;   // BDAP: the firmware is the combustion-safety authority
  p.has_pump_hz_control = true;    // BDAP: real pump frequency field
  p.has_power_level_control = false;
  p.has_fan_rpm_control = true;    // BDAP: real fan RPM field
  return p;
}

// 2 kW Vevor heater - the replacement hardware.
//
// !!! PROVISIONAL VALUES !!!
// Derived from typical 2 kW Chinese diesel heater behaviour (smaller dosing
// pump, smaller blower, cooler heat exchanger than an 8 kW unit) and the
// V9 protocol reference. NOT yet validated against the physical unit.
// Every value must be verified/tuned during the interfacing phase before
// this profile is used for live control.
inline HeaterProfile profile_vevor_2kw() {
  HeaterProfile p{};
  p.name = "Vevor 2kW";
  p.model = HeaterModel::VEVOR_2KW;
  // Vevor bus (per zatakon/esphome-vevor-heater RE): 4800 baud, AA66/AA77 frames,
  // inverted logic levels. Requires uart: baud_rate 4800 + inverted pins in YAML.
  p.protocol = HeaterProtocol::VEVOR_UART;
  p.uart_baud_rate = 4800;
  p.uart_inverted = true;
  p.fan_sensor = 0x01;   // SN-1 (verify on bench)
  p.glow_power = 0x05;   // (verify glow plug rating)
  // 2 kW: ~0.2-0.3 L/h fuel => dosing pump runs well below an 8 kW's range
  p.pump_freq_min = 0.5f;
  p.pump_freq_max = 2.5f;
  p.pump_freq_step = 0.1f;
  p.pump_freq_initial = 1.0f;
  p.pump_prime_freq = 2.5f;
  // 2 kW: smaller blower wheel, lower airflow
  p.fan_rpm_min = 900;
  p.fan_rpm_max = 3000;
  p.ignition_fan_rpm = 1400;
  p.ignition_hx_threshold = 85.0f;
  p.cooldown_fan_rpm = 2500;
  // 2 kW: smaller heat exchanger runs cooler than an 8 kW
  p.hx_temp_low = 145.0f;
  p.hx_temp_target = 180.0f;
  p.hx_temp_high = 190.0f;
  p.hx_temp_critical = 205.0f;
  p.cooldown_target_temp = 55.0f;
  p.temp_min = 0;
  p.temp_max = 30;
  p.heat_mode_manual = false;  // v1: firmware keeps modulating HEAT
  p.ecm_owned_safety = true;   // Vevor ECM: self-primes, self-derates, reports E-codes
  p.has_pump_hz_control = false;   // Vevor: pump Hz is ECM-derived, not commandable
  p.has_power_level_control = true; // Vevor: the one fuel dial (1-10)
  p.has_fan_rpm_control = false;   // Vevor: no fan field exists (probe bytes ignored)
  return p;
}

// ---------------------------------------------------------------------------
// 2 kW Vevor - EXPERIMENTAL v2 profile
// ---------------------------------------------------------------------------
// Identical hardware mapping to profile_vevor_2kw() (levels 1-10, no pump-Hz or
// fan-RPM command in the protocol), but the user owns the level in HEAT mode:
// the firmware stops rewriting it in stable combustion (no approach control,
// no HX-soot pump override). Kept as a SEPARATE profile so the bench can revert
// to v1 behaviour by switching one YAML line back to `heater_model: vevor_2kw`.
//
// Safety that still applies in v2 HEAT: the ignition warm-up ramp (state 4),
// the HX-too-hot pump decrease, the HX-critical emergency stop, and the
// configurable room-temperature limit (ambient_heat_limit).
inline HeaterProfile profile_vevor_2kw_v2() {
  HeaterProfile p = profile_vevor_2kw();
  p.name = "Vevor 2kW v2 (user-owned HEAT)";
  p.model = HeaterModel::VEVOR_2KW_V2;
  p.heat_mode_manual = true;
  return p;
}

// Resolve a model selection to its profile (stable, never dangling).
inline const HeaterProfile &profile_for(HeaterModel model) {
  static const HeaterProfile jeabong = profile_jeabong_8kw();
  static const HeaterProfile vevor = profile_vevor_2kw();
  static const HeaterProfile vevor_v2 = profile_vevor_2kw_v2();
  switch (model) {
    case HeaterModel::JEABONG_8KW:
      return jeabong;
    case HeaterModel::VEVOR_2KW:
      return vevor;
    case HeaterModel::VEVOR_2KW_V2:
      return vevor_v2;
    default:
      return jeabong;
  }
}

// Which invariant groups an override validation had to repair.
struct ProfileValidationReport {
  bool pump_range_reverted = false;    // min >= max (or prime/initial outside)
  bool fan_range_reverted = false;     // min >= max
  bool hx_chain_reverted = false;      // low < target < high < critical broken
  bool temp_range_reverted = false;    // min >= max
  bool clean() const { return !(pump_range_reverted || fan_range_reverted ||
                                hx_chain_reverted || temp_range_reverted); }
};

// Repair YAML overrides that would break a profile invariant. Permissive by
// design (single-operator audience): every violation is reverted to the
// compiled preset value for that model - a typo can never silently disable a
// safety net, and the warning text tells the owner which keys to fix.
// Pure (host-tested in tests/test_profiles.cpp).
inline ProfileValidationReport validate_profile_invariants(HeaterProfile &p,
                                                           const HeaterProfile &preset) {
  ProfileValidationReport r;

  // --- fuel pump: min < max, and prime/initial inside the range ---
  if (!(p.pump_freq_min < p.pump_freq_max)) {
    p.pump_freq_min = preset.pump_freq_min;
    p.pump_freq_max = preset.pump_freq_max;
    r.pump_range_reverted = true;
  }
  if (!(p.pump_prime_freq >= p.pump_freq_min && p.pump_prime_freq <= p.pump_freq_max)) {
    p.pump_prime_freq = preset.pump_prime_freq;
    r.pump_range_reverted = true;
  }
  if (!(p.pump_freq_initial >= p.pump_freq_min && p.pump_freq_initial <= p.pump_freq_max)) {
    p.pump_freq_initial = preset.pump_freq_initial;
    r.pump_range_reverted = true;
  }

  // --- fan: min < max (ignition/cooldown clamped by encode, not validated) ---
  if (!(p.fan_rpm_min < p.fan_rpm_max)) {
    p.fan_rpm_min = preset.fan_rpm_min;
    p.fan_rpm_max = preset.fan_rpm_max;
    r.fan_range_reverted = true;
  }

  // --- HX safety chain: low < target < high < critical, cooldown below low ---
  if (!(p.hx_temp_low < p.hx_temp_target && p.hx_temp_target < p.hx_temp_high &&
        p.hx_temp_high < p.hx_temp_critical &&
        p.cooldown_target_temp < p.hx_temp_low)) {
    p.hx_temp_low = preset.hx_temp_low;
    p.hx_temp_target = preset.hx_temp_target;
    p.hx_temp_high = preset.hx_temp_high;
    p.hx_temp_critical = preset.hx_temp_critical;
    p.cooldown_target_temp = preset.cooldown_target_temp;
    r.hx_chain_reverted = true;
  }

  // --- setpoint range ---
  if (!(p.temp_min < p.temp_max)) {
    p.temp_min = preset.temp_min;
    p.temp_max = preset.temp_max;
    r.temp_range_reverted = true;
  }

  return r;
}

}  // namespace heater_uart
}  // namespace esphome
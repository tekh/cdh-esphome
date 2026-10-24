#pragma once

// Controller-side policies, extracted from the HeaterUart facade (ROADMAP.md
// Phase 3). Pure C++ so the host test harness can drive the thermostat state
// machine, the combustion-protection rules and the room-sensor physics guard
// without a device.
//
//  - Thermostat: the AUTO three-phase thermostat + HEAT (modulating and
//    user-owned) + auto-shutdown / standby / auto-restart state machine.
//    Works in the profile's pump-Hz domain; the facade converts to the
//    normalized output domain at the driver boundary.
//  - CombustionSafety: heat-exchanger protection that overrides the
//    thermostat (soot floor, too-hot, critical emergency stop), the BDAP
//    fan computation (air matched to fuel, ignition hold, cooldown purge)
//    and the cooldown completion monitor.
//  - RoomSensorGuard: physics guard for the external room sensor (DHT22):
//    NaN rejection, glitch-step rejection, boot 3-sample consensus, staleness.
//
// The safety override stack is ON by default on every heater (owner decision
// #3) and can only be disabled on profiles whose ECM owns combustion safety
// (`ecm_owned_safety`), via `firmware_safety_oversight`.

#include <cstdint>

#include "heater_profile.h"
#include "heater_protocol_driver.h"

namespace esphome {
namespace heater_uart {

// Ambient temperature limit for HEAT mode safety (configurable via config)
static const float AMBIENT_HEAT_LIMIT = 26.0f;

// Heater operating mode (control vocabulary - owned by the policies layer).
enum class HeaterMode : uint8_t {
  OFF = 0,   // Heater stays off
  AUTO = 1,  // Thermostat controls - auto-shutdown/restart enabled
  ON = 2     // Force heater on, bypass thermostat (safety limits still apply)
};

// === Thermostat (control policy) ============================================

class Thermostat {
 public:
  // Per-tick control parameters (facade-owned configuration + profile policy).
  struct Params {
    float approach_threshold = 1.0f;         // °C from target where approach control begins
    float auto_shutdown_overshoot = 0.5f;    // °C above target before AUTO auto-shutdown
    float auto_shutdown_hysteresis = 1.5f;   // °C below target before AUTO re-engage
    bool heat_mode_manual = false;           // profile policy: HEAT level is user-owned
    bool oversight_enabled = true;           // firmware combustion oversight (gated by ecm_owned_safety)
    bool external_sensor_present = false;    // external room sensor declared in YAML
    bool sensor_stale = false;               // room sensor blind (RoomSensorGuard)
  };

  // What the thermostat wants applied this tick. Multiple flags can be set;
  // the facade applies them in a fixed order (stop, start, pulse stop) to
  // preserve v1 semantics.
  struct Result {
    float pump_request_hz = 0.0f;  // fuel request in the profile's Hz domain
    bool request_start = false;    // assert START (+ persistent intent)
    bool request_stop = false;     // assert STOP (+ clear intent)
    bool pulse_stop = false;       // one-shot STOP (intent kept)
    bool enter_cooldown = false;   // facade should begin the cooldown purge
  };

  void begin(const HeaterProfile &profile) { profile_ = &profile; }

  // Periodic control tick - the former HeaterUart::thermostat_control(),
  // lifted verbatim with the hardcoded cadence/aggressiveness constants
  // hoisted to named constants. `sensor_stale` implements the room-sensor
  // health gate (the guard computes it; see RoomSensorGuard).
  Result control(const Telemetry &tlm, float room_temp, float target_temp, HeaterMode mode,
                 const Params &params, uint32_t now);

  // State introspection (binary sensors) and user-facing setpoints.
  bool in_standby() const { return in_standby_; }
  bool in_auto_shutdown() const { return in_auto_shutdown_; }
  float pump_request() const { return pump_freq_setting_; }
  float user_request() const { return pump_freq_user_request_; }

  // User dial writes: set the live request AND the user setpoint (v1
  // set_pump_frequency semantics).
  void set_request(float hz) {
    pump_freq_setting_ = hz;
    pump_freq_user_request_ = hz;
  }
  // Internal re-seating of the live request without touching the user
  // setpoint (mode-entry start levels, ignition ramps).
  void seat_request(float hz) { pump_freq_setting_ = hz; }
  void reset_adjust_timer(uint32_t now) { last_pump_adjust_time_ = now; }
  void set_standby(bool s) { in_standby_ = s; }
  void set_auto_shutdown(bool s) { in_auto_shutdown_ = s; }

 protected:
  static const uint32_t PUMP_ADJUST_INTERVAL_MS = 5000;  // thermal mass: 5 s between adjustments
  // Thermostat aggressiveness (hardcoded in 7 subtly-diverging copies in the
  // old thermostat_control; hoisted here as named policy constants - Phase 4
  // may expose them to YAML).
  static constexpr float GAP_THRESHOLD_FAR = 5.0f;   // °C: gap >= this -> 3x step
  static constexpr float GAP_THRESHOLD_MID = 2.0f;   // °C: gap >= this -> 2x step
  static constexpr float STEP_MULT_FAR = 3.0f;
  static constexpr float STEP_MULT_MID = 2.0f;
  static constexpr float OS_THRESHOLD_BIG = 1.0f;    // °C: overshoot >= this -> 3x reduction
  static constexpr float OS_THRESHOLD_MID = 0.5f;    // °C: overshoot >= this -> 2x reduction
  static constexpr float OS_MULT_BIG = 3.0f;
  static constexpr float OS_MULT_MID = 2.0f;
  static constexpr float PUMP_EPSILON = 0.05f;       // Hz tolerance for at-min comparisons

  const HeaterProfile *profile_ = nullptr;
  float pump_freq_setting_ = 1.6f;
  float pump_freq_user_request_ = 1.6f;
  bool in_standby_ = false;
  bool in_auto_shutdown_ = false;
  uint32_t last_pump_adjust_time_ = 0;
};

// === CombustionSafety (protection policy) ===================================

class CombustionSafety {
 public:
  void begin(const HeaterProfile &profile, bool firmware_oversight) {
    profile_ = &profile;
    oversight_ = firmware_oversight;
  }

  // --- ambient HEAT-mode limit ---
  // True when HEAT mode must be force-shut (room above the configurable
  // ambient limit). The facade asserts pulse_stop on true, every tick while
  // hot - the heater restarts when the room cools below the limit (v1).
  bool ambient_limit_exceeded(HeaterMode mode, float room_temp, float limit) const {
    return mode == HeaterMode::ON && room_temp > limit;
  }

  // --- BDAP fan computation (air matched to fuel) ---
  // Moved verbatim from the old build_tx_frame: cooldown purge RPM, ignition
  // hold below the HX threshold, otherwise proportional to the fuel rate.
  // (The fuel fraction IS the old pump_ratio: fraction = (hz-min)/span.)
  uint16_t fan_rpm(float output_fraction, const Telemetry &tlm) const;

  // --- cooldown monitor ---
  void begin_cooldown() { in_cooldown_ = true; }
  void end_cooldown() { in_cooldown_ = false; }
  bool in_cooldown() const { return in_cooldown_; }
  // True when the heat exchanger has cooled enough for the fan to stop.
  bool cooldown_complete(float hx_temp) const { return hx_temp <= profile_->cooldown_target_temp; }

  // firmware_safety_oversight gate (read by the facade when wiring results).
  bool oversight() const { return oversight_; }

 protected:
  const HeaterProfile *profile_ = nullptr;
  bool oversight_ = true;
  bool in_cooldown_ = false;
};

// === RoomSensorGuard (physics guard) ========================================

class RoomSensorGuard {
 public:
  // Feed one external sensor sample. Returns true when the sample was
  // ACCEPTED (fresh baseline or plausible step). NaN and implausible jumps
  // are rejected; accepted samples refresh the staleness timer.
  bool refresh(float sample, uint32_t now);

  // True when no accepted reading arrived within the timeout (or none ever
  // since boot). The thermostat must not burn on blind data.
  bool stale(uint32_t now) const;

  // Latest trusted reading; 0.0 until a baseline is established (pins the
  // v1 `current_temperature_value_ > 0` fallback quirk).
  float value() const { return value_; }
  bool baseline_established() const { return accepted_ever_; }

 protected:
  static const uint32_t TIMEOUT_MS = 10 * 60 * 1000;  // 10 min without an accepted reading = blind
  static constexpr float MAX_STEP = 0.5f;  // max plausible change per 10 s sensor interval (field rate ~0.3 C/min; 10x headroom)
  static constexpr float BOOT_MAX_SPREAD = 2.0f;  // first 3 boot samples must agree within this (physical drift over ~30 s is ~0.05 C)
  static constexpr uint8_t BOOT_SAMPLES = 3;
  static constexpr uint32_t BOOT_MIN_SPACING_MS = 5000;  // counted boot samples >= 5 s apart: distinct DHT22 conversions

  float value_ = 0.0f;
  bool accepted_ever_ = false;
  uint32_t last_accept_time_ = 0;  // millis() of the last ACCEPTED reading; 0 = none yet
  float boot_sample_[BOOT_SAMPLES] = {0.0f, 0.0f, 0.0f};
  uint8_t boot_sample_count_ = 0;
  uint32_t boot_last_sample_time_ = 0;
  uint32_t last_reject_log_ = 0;  // glitch-reject log throttle
};

}  // namespace heater_uart
}  // namespace esphome

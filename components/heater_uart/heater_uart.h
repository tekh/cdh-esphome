#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"

#include <map>
#include <memory>
#include <string>

#include "heater_policies.h"
#include "heater_profile.h"
#include "heater_protocol_driver.h"

namespace esphome {
namespace heater_uart {

// Operating voltage options (encoded into BDAP TX byte 11)
static const uint8_t VOLTAGE_12V = 0x78;  // 120 = 12.0V
static const uint8_t VOLTAGE_24V = 0xF0;  // 240 = 24.0V

// Fuel priming run length (60 s pump-only). Moves into Tuning/YAML at the
// tuning-exposure step (ROADMAP.md Phase 4).
static const uint32_t PRIMING_DURATION_MS = 60000;

// Run-state constants and the protocol driver live in heater_protocol_driver.h
// (included above): RUN_STATE_*, HeaterProtocolDriver, Telemetry, ControlInputs.
// The policies (Thermostat / CombustionSafety / RoomSensorGuard) live in
// heater_policies.h.

// HeaterUart is the ESPHome facade: entity registry + publishing, user/mode
// intent, and orchestration of the three policies and the protocol driver.
// All protocol-specific behaviour - framing, cadence, echo, checksums,
// encode/decode - lives behind HeaterProtocolDriver (BdapDriver / VevorDriver).
class HeaterUart : public PollingComponent, public uart::UARTDevice, public ByteStream {
 public:
  HeaterUart() = default;

  // Setters for sensors
  void set_sensor(const std::string &key, sensor::Sensor *sensor);
  void set_text_sensor(const std::string &key, text_sensor::TextSensor *text_sensor);
  void set_binary_sensor(const std::string &key, binary_sensor::BinarySensor *binary_sensor);
  void set_temperature_number(number::Number *num) { this->temperature_number_ = num; }
  void set_temperature_sensor(sensor::Sensor *sensor) { this->external_temp_sensor_ = sensor; }
  void set_pump_number(number::Number *num) { this->pump_number_ = num; }
  void set_power_level_number(number::Number *num) { this->power_level_number_ = num; }
  void set_fan_probe_number(number::Number *num) { this->fan_probe_number_ = num; }
  void set_mode_select(select::Select *sel) { this->mode_select_ = sel; }

  // Configuration setters
  void set_heater_model(HeaterModel model) { this->profile_ = profile_for(model); }
  void set_operating_voltage(uint8_t voltage) { this->operating_voltage_ = voltage; }
  void set_temperature_backoff(float offset) { this->temp_backoff_offset_ = offset; }
  void set_altitude(uint16_t altitude) { this->altitude_ = altitude; }
  void set_auto_shutdown_overshoot(float overshoot) { this->auto_shutdown_overshoot_ = overshoot; }
  void set_auto_shutdown_hysteresis(float hysteresis) { this->auto_shutdown_hysteresis_ = hysteresis; }
  void set_approach_threshold(float threshold) { this->approach_threshold_ = threshold; }
  void set_ambient_heat_limit(float limit) { this->ambient_heat_limit_ = limit; }
  void set_firmware_safety_oversight(bool enabled) { this->firmware_safety_oversight_ = enabled; }

  // --- Tuning overrides (ROADMAP Phase 4). All optional in YAML; each key
  // mutates the copied profile_ (per-key override over the compiled preset).
  // NOT exposed: protocol/capability/policy flags (heat_mode_manual,
  // ecm_owned_safety, has_*_control) - they are model identity, not tuning.
  void set_pump_freq_min(float v) { profile_.pump_freq_min = v; }
  void set_pump_freq_max(float v) { profile_.pump_freq_max = v; }
  void set_pump_freq_step(float v) { profile_.pump_freq_step = v; }
  void set_pump_freq_initial(float v) { profile_.pump_freq_initial = v; }
  void set_pump_prime_freq(float v) { profile_.pump_prime_freq = v; }
  void set_fan_rpm_min(uint16_t v) { profile_.fan_rpm_min = v; }
  void set_fan_rpm_max(uint16_t v) { profile_.fan_rpm_max = v; }
  void set_ignition_fan_rpm(uint16_t v) { profile_.ignition_fan_rpm = v; }
  void set_cooldown_fan_rpm(uint16_t v) { profile_.cooldown_fan_rpm = v; }
  void set_ignition_hx_threshold(float v) { profile_.ignition_hx_threshold = v; }
  void set_hx_temp_low(float v) { profile_.hx_temp_low = v; }
  void set_hx_temp_target(float v) { profile_.hx_temp_target = v; }
  void set_hx_temp_high(float v) { profile_.hx_temp_high = v; }
  void set_hx_temp_critical(float v) { profile_.hx_temp_critical = v; }
  void set_cooldown_target_temp(float v) { profile_.cooldown_target_temp = v; }
  void set_fan_sensor(uint8_t v) { profile_.fan_sensor = v; }
  void set_glow_power(uint8_t v) { profile_.glow_power = v; }
  void set_temp_min(uint8_t v) { profile_.temp_min = v; }
  void set_temp_max(uint8_t v) { profile_.temp_max = v; }

  // Thermostat policy overrides (v1 constants, now YAML-tunable).
  void set_pump_adjust_interval(float seconds) { pump_adjust_interval_ms_ = (uint32_t)(seconds * 1000.0f); }
  void set_priming_duration(float seconds) { priming_duration_ms_ = (uint32_t)(seconds * 1000.0f); }
  void set_thermostat_gap_far(float v) { gap_threshold_far_ = v; }
  void set_thermostat_gap_mid(float v) { gap_threshold_mid_ = v; }
  void set_thermostat_step_mult_far(float v) { step_mult_far_ = v; }
  void set_thermostat_step_mult_mid(float v) { step_mult_mid_ = v; }
  void set_thermostat_os_big(float v) { os_threshold_big_ = v; }
  void set_thermostat_os_mid(float v) { os_threshold_mid_ = v; }
  void set_thermostat_os_mult_big(float v) { os_mult_big_ = v; }
  void set_thermostat_os_mult_mid(float v) { os_mult_mid_ = v; }

  // Heater mode control
  void set_heater_mode(HeaterMode mode);

  void setup() override;
  void loop() override;
  void update() override;

  // ByteStream (driver transport): HeaterUart IS-A UARTDevice; the driver
  // talks to the bus through this interface.
  int available() override;
  int read() override;
  void write_array(const uint8_t *data, size_t length) override;
  void flush() override;

  // Command methods
  void turn_on();
  void turn_off();
  void set_desired_temperature(float temperature);
  void set_pump_frequency(float frequency);
  // Power-level dial (1-10) for profiles whose fuel command is a level, not a
  // frequency (Vevor). Converts via the shared output_domain mapping.
  void set_power_level(uint8_t level);
  // Current level for display: mirrors the user's setpoint on manual-HEAT
  // profiles, otherwise the firmware's current request.
  uint8_t get_power_level_display() const;
  // Experimental fan-speed probe (Vevor): 0 = disabled, >0 = write BE uint16
  // into undocumented TX bytes 10-11. See heater_fan_probe_number.h.
  void set_fan_probe_rpm(uint16_t rpm);
  uint16_t get_fan_probe_rpm() const;

  // State accessors
  bool get_on_off_state() const;
  float get_desired_temperature() const { return desired_temp_setting_; }
  // Room-sensor health: true when the external room sensor has not produced a
  // valid (non-NaN) reading for >10 minutes. The thermostat must not burn on
  // blind data (a stale last value hides an overheat, and a stale cold value
  // could trigger a blind start). 2026-10-20, user requirement.
  bool is_room_sensor_stale() const;
  // Accept a fresh external room-sensor reading only if it is valid (non-NaN)
  // AND physically plausible (within RoomSensorGuard::MAX_STEP of the last
  // accepted value per sensor interval). A DHT22 right after a failed read
  // can emit a garbage-but-valid sample (2026-10-21: 18.4C against a 19.2C
  // trend, which tripped a false standby->start); such samples are rejected
  // and the last trusted value is held. At boot, no single sample may become
  // the baseline: the first 3 distinct readings (>=5 s apart, ~30 s total)
  // must agree within 2.0C, and the baseline is the median of the three.
  // Until then the sensor counts as stale, so AUTO keeps the heater off (the
  // safe state right after a reboot anyway).
  // Returns true if the reading was accepted.
  bool refresh_room_temperature();
  float get_current_temperature() const { return room_sensor_.value(); }
  float get_pump_frequency_setting() const { return thermostat_.pump_request(); }
  bool is_in_auto_shutdown() const { return thermostat_.in_auto_shutdown(); }
  bool is_in_standby() const { return thermostat_.in_standby(); }
  HeaterMode get_heater_mode() const { return heater_mode_; }
  bool is_priming() const { return is_priming_; }
  bool safety_oversight_active() const { return safety_.oversight(); }

  // Active profile accessors (used by the number entities for clamping)
  const HeaterProfile &get_profile() const { return profile_; }
  float get_pump_freq_min() const { return profile_.pump_freq_min; }
  float get_pump_freq_max() const { return profile_.pump_freq_max; }
  uint8_t get_temp_min() const { return profile_.temp_min; }
  uint8_t get_temp_max() const { return profile_.temp_max; }

  // Fuel priming control
  void start_priming();
  void stop_priming();

 protected:
  // Active heater profile (model-specific tuning parameters)
  HeaterProfile profile_{profile_jeabong_8kw()};
  std::map<std::string, sensor::Sensor *> sensors_;
  std::map<std::string, text_sensor::TextSensor *> text_sensors_;
  std::map<std::string, binary_sensor::BinarySensor *> binary_sensors_;

  // Temperature number reference (for state sync)
  number::Number *temperature_number_{nullptr};

  // External temperature sensor (room temperature)
  sensor::Sensor *external_temp_sensor_{nullptr};

  // Pump frequency number reference (for state sync)
  number::Number *pump_number_{nullptr};

  // Power level number reference (for state sync; Vevor-class profiles)
  number::Number *power_level_number_{nullptr};

  // Fan speed probe number reference (experimental, Vevor undocumented bytes)
  number::Number *fan_probe_number_{nullptr};

   // Mode select reference (for state sync)
   select::Select *mode_select_{nullptr};

   float ambient_heat_limit_ = AMBIENT_HEAT_LIMIT; // Configurable limit for HEAT mode shutdown
   HeaterMode heater_mode_ = HeaterMode::OFF;

  // Frame-independent off-state recheck cadence (see loop()): the Vevor ECM
  // stops answering status polls once it is fully off, so on_fresh_frame() -
  // and with it Thermostat::control() - goes silent while idle. The
  // standby/auto-restart recheck is therefore driven off this timer (1 Hz,
  // matching the driver cadence) instead of the RX frame.
  uint32_t last_off_recheck_time_ = 0;

  // Protocol driver (owns framing, cadence, echo, checksums, decode).
  // Selected by profile_.protocol in setup().
  std::unique_ptr<HeaterProtocolDriver> driver_;

  // Controller policies (ROADMAP.md Phase 3)
  Thermostat thermostat_;
  CombustionSafety safety_;
  RoomSensorGuard room_sensor_;

  // --- Facade configuration / state ---
  uint8_t operating_voltage_ = VOLTAGE_12V;
  uint16_t altitude_ = 750;              // Altitude in metres (affects combustion)
  float desired_temp_setting_ = 19.0f;   // Desired temperature. 2026-10-20: default 22.0 -> 19.0 (post-reboot state; mode default is already OFF)
  float temp_backoff_offset_ = 0.5f;     // RESERVED: accepted from YAML (temperature_backoff) and stored, but v1 never read it in any logic path - kept so the YAML key stays valid; becomes a real Tuning key at the tuning-exposure step (ROADMAP.md).

  // Auto-shutdown configuration (passed into Thermostat::Params each tick)
  float auto_shutdown_overshoot_ = 0.5f;   // Temperature overshoot before auto-shutdown (°C)
  float auto_shutdown_hysteresis_ = 1.5f;  // Temperature drop before auto-restart (°C)
  float approach_threshold_ = 1.0f;        // Start reducing pump when within this of target (°C)
  bool firmware_safety_oversight_ = true;  // gated: false only honoured on ecm_owned_safety profiles

  // Thermostat policy overrides (defaults = the v1 hardcoded constants)
  uint32_t pump_adjust_interval_ms_ = 5000;
  float gap_threshold_far_ = 5.0f;
  float gap_threshold_mid_ = 2.0f;
  float step_mult_far_ = 3.0f;
  float step_mult_mid_ = 2.0f;
  float os_threshold_big_ = 1.0f;
  float os_threshold_mid_ = 0.5f;
  float os_mult_big_ = 3.0f;
  float os_mult_mid_ = 2.0f;

  // Fuel priming state
  bool is_priming_ = false;                // Fuel priming active
  uint32_t priming_start_time_ = 0;        // Time when priming started (millis)
  uint32_t priming_duration_ms_ = PRIMING_DURATION_MS;  // YAML-overridable run length

  // Telemetry mirror for update() publishing (copied on each fresh frame)
  int fan_speed_value_ = 0;
  float supply_voltage_value_ = 0;
  float heat_exchanger_temp_value_ = 0;
  float glow_plug_voltage_value_ = 0;
  float glow_plug_current_value_ = 0;
  float glow_a_value_ = 0;
  float glow_b_value_ = 0;
  float glow_c_value_ = 0;
  float pump_frequency_value_ = 0;
  float fan_voltage_value_ = 0;
  int power_level_actual_value_ = 0;
  int error_code_value_ = 0;
  bool on_off_value_ = false;
  std::string run_state_description_ = "Unknown";
  std::string error_code_description_ = "Unknown";

  // Driver plumbing: called when loop() received at least one fresh frame.
  void on_fresh_frame(const Thermostat::Params &params);
  // Per-tick policy parameters from facade configuration + the sensor guard.
  Thermostat::Params build_params() const;
};

}  // namespace heater_uart
}  // namespace esphome

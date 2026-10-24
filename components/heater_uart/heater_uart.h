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

#include "heater_profile.h"
#include "heater_protocol_driver.h"

namespace esphome {
namespace heater_uart {

// Ambient temperature limit for HEAT mode safety (configurable via config)
static const float AMBIENT_HEAT_LIMIT = 26.0f;

enum class HeaterMode : uint8_t {
  OFF = 0,   // Heater stays off
  AUTO = 1,  // Thermostat controls - auto-shutdown/restart enabled
  ON = 2     // Force heater on, bypass thermostat (safety limits still apply)
};

// Operating voltage options (encoded into BDAP TX byte 11)
static const uint8_t VOLTAGE_12V = 0x78;  // 120 = 12.0V
static const uint8_t VOLTAGE_24V = 0xF0;  // 240 = 24.0V

// Pump adjustment timing - thermal mass means changes take time to show effect
static const uint32_t PUMP_ADJUST_INTERVAL_MS = 5000;  // 5 seconds between adjustments

// Fuel priming run length (60 s pump-only). Moves into Tuning/YAML at Phase 3
// of the refactor (ROADMAP.md).
static const uint32_t PRIMING_DURATION_MS = 60000;

// Run-state constants and the protocol driver live in heater_protocol_driver.h
// (included above): RUN_STATE_*, HeaterProtocolDriver, Telemetry, ControlInputs.

// HeaterUart is the ESHome facade: entity registry + publishing, user/mode
// intent, room-sensor health, priming, and the thermostat/safety policy (the
// Phase 3 extraction moves the policy out). All protocol-specific behaviour -
// framing, cadence, echo, checksums, encode/decode - lives behind
// HeaterProtocolDriver (BdapDriver / VevorDriver), selected by the profile.
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
  void set_altitude(uint16_t altitude) { this->altitude_ = altitude; }
  void set_auto_shutdown_overshoot(float overshoot) { this->auto_shutdown_overshoot_ = overshoot; }
  void set_auto_shutdown_hysteresis(float hysteresis) { this->auto_shutdown_hysteresis_ = hysteresis; }
  void set_approach_threshold(float threshold) { this->approach_threshold_ = threshold; }
  void set_ambient_heat_limit(float limit) { this->ambient_heat_limit_ = limit; }

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
  // AND physically plausible (within ROOM_TEMP_MAX_STEP of the last accepted
  // value per sensor interval). A DHT22 right after a failed read can emit a
  // garbage-but-valid sample (2026-10-21: 18.4C against a 19.2C trend, which
  // tripped a false standby->start); such samples are rejected and the last
  // trusted value is held. At boot, no single sample may become the baseline:
  // the first ROOM_TEMP_BOOT_SAMPLES distinct readings (>=5 s apart, ~30 s
  // total) must agree within ROOM_TEMP_BOOT_MAX_SPREAD, and the baseline is
  // the median of the three. Until then the sensor counts as stale, so AUTO
  // keeps the heater off (the safe state right after a reboot anyway).
  // Returns true if the reading was accepted.
  bool refresh_room_temperature();
  float get_current_temperature() const { return current_temperature_value_; }
  float get_pump_frequency_setting() const { return pump_freq_setting_; }
  bool is_in_auto_shutdown() const { return in_auto_shutdown_; }
  bool is_in_standby() const { return in_standby_; }
  HeaterMode get_heater_mode() const { return heater_mode_; }
  bool is_priming() const { return is_priming_; }

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

  // External temperature sensor (room temperature in standalone mode)
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

  // Protocol driver (owns framing, cadence, echo, checksums, decode).
  // Selected by profile_.protocol in setup().
  std::unique_ptr<HeaterProtocolDriver> driver_;

  // --- Controller state (mode/intent; the driver holds wire-level intent) ---
  uint8_t operating_voltage_ = VOLTAGE_12V;
  uint16_t altitude_ = 750;              // Altitude in metres (affects combustion)
  float desired_temp_setting_ = 19.0f;   // Desired temperature. 2026-10-20: default 22.0 -> 19.0 (post-reboot state; mode default is already OFF)
  float pump_freq_setting_ = 1.6f;       // Fuel request in the profile's Hz domain (normalized at the TX boundary)
  float pump_freq_user_request_ = 1.6f;  // User's requested setting (Hz). On profiles with heat_mode_manual the firmware sends THIS verbatim in HEAT/state-5 instead of modulating, and the pump number entity mirrors it (so the displayed setpoint never gets clobbered by the ignition warm-up ramp).
  bool in_cooldown_ = false;             // Cooldown mode active (fan running to cool HX)
  uint32_t last_pump_adjust_time_ = 0;   // Last time pump frequency was adjusted

  // Auto-shutdown configuration (used in AUTO mode)
  float auto_shutdown_overshoot_ = 0.5f;   // Temperature overshoot before auto-shutdown (°C)
  float auto_shutdown_hysteresis_ = 1.5f;  // Temperature drop before auto-restart (°C)
  float approach_threshold_ = 1.0f;        // Start reducing pump when within this of target (°C)
  bool in_auto_shutdown_ = false;          // Currently in auto-shutdown state
  bool in_standby_ = false;                // Waiting for temp to drop before starting (AUTO mode)

  // Fuel priming state
  bool is_priming_ = false;                // Fuel priming active
  uint32_t priming_start_time_ = 0;        // Time when priming started (millis)

  // Room temperature (external sensor, physics-guarded; see
  // refresh_room_temperature())
  float current_temperature_value_ = 0;
  uint32_t last_valid_room_temp_time_ = 0;  // millis() of the last ACCEPTED room-sensor reading; 0 = none yet (treated as stale after 10 min)
  static const uint32_t ROOM_SENSOR_TIMEOUT_MS = 10 * 60 * 1000;  // 10 min without an accepted room reading = sensor fault
  static constexpr float ROOM_TEMP_MAX_STEP = 0.5f;  // max plausible room-temp change per 10s sensor interval (observed field rate ~0.3C/min); larger jumps are DHT22 glitch samples
  bool room_temp_accepted_ever_ = false;  // baseline established (via boot consensus); the first valid reading alone is NOT accepted
  static constexpr float ROOM_TEMP_BOOT_MAX_SPREAD = 2.0f;  // the first 3 boot samples must agree within this spread to establish the baseline (physical drift over ~30s is ~0.05C, so wider means a bad sample)
  static constexpr uint8_t ROOM_TEMP_BOOT_SAMPLES = 3;
  static constexpr uint32_t ROOM_TEMP_BOOT_MIN_SPACING_MS = 5000;  // counted boot samples must be >=5 s apart (sensor interval is 10 s) so each is a distinct DHT22 conversion, not a re-read of the same value
  float room_temp_boot_sample_[ROOM_TEMP_BOOT_SAMPLES] = {0.0f, 0.0f, 0.0f};
  uint8_t room_temp_boot_sample_count_ = 0;
  uint32_t room_temp_boot_last_sample_time_ = 0;

  // Last parsed telemetry mirror for publishing (copied on each fresh frame)
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
  void on_fresh_frame();
  // Shared thermostat/safety logic (both protocols call this after parsing)
  void thermostat_control();
};

}  // namespace heater_uart
}  // namespace esphome

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/number/number.h"

#include <map>
#include <string>

namespace esphome {
namespace heater_uart {

// Command bytes for Tx Byte[2]
static const uint8_t CMD_NO_CHANGE = 0x00;
static const uint8_t CMD_STOP = 0x05;
static const uint8_t CMD_START = 0xA0;

// Temperature limits (from protocol)
static const uint8_t TEMP_MIN = 8;
static const uint8_t TEMP_MAX = 35;

// Operating voltage options
static const uint8_t VOLTAGE_12V = 0x78;  // 120 = 12.0V
static const uint8_t VOLTAGE_24V = 0xF0;  // 240 = 24.0V

// Default frame parameters (from Afterburner project)
static const uint8_t DEFAULT_MIN_PUMP_FREQ = 0x0E;    // 1.4 Hz
static const uint8_t DEFAULT_MAX_PUMP_FREQ = 0x32;    // 5.0 Hz
static const uint16_t DEFAULT_MIN_FAN_RPM = 1450;
static const uint16_t DEFAULT_MAX_FAN_RPM = 4500;
static const uint8_t DEFAULT_FAN_SENSOR = 0x01;       // SN-1
static const uint8_t DEFAULT_GLOW_POWER = 0x05;
static const uint16_t DEFAULT_ALTITUDE = 0x0DAC;      // 3500m

// Standalone mode timing
static const uint32_t STANDALONE_TX_INTERVAL_MS = 1000;  // 1 Hz
static const uint32_t STANDALONE_RX_TIMEOUT_MS = 200;    // Wait for response (heater responds in ~100-130ms)

class HeaterUart : public PollingComponent, public uart::UARTDevice {
 public:
  HeaterUart() = default;

  // Setters for sensors
  void set_sensor(const std::string &key, sensor::Sensor *sensor);
  void set_text_sensor(const std::string &key, text_sensor::TextSensor *text_sensor);
  void set_binary_sensor(const std::string &key, binary_sensor::BinarySensor *binary_sensor);
  void set_power_switch(switch_::Switch *sw) { this->power_switch_ = sw; }
  void set_temperature_number(number::Number *num) { this->temperature_number_ = num; }
  void set_temperature_sensor(sensor::Sensor *sensor) { this->external_temp_sensor_ = sensor; }

  // Configuration setters
  void set_standalone_mode(bool standalone) { this->standalone_mode_ = standalone; }
  void set_operating_voltage(uint8_t voltage) { this->operating_voltage_ = voltage; }

  void setup() override;
  void loop() override;
  void update() override;

  // Command methods
  void turn_on();
  void turn_off();
  void set_desired_temperature(uint8_t temperature);

  // State accessors
  bool get_on_off_state() const { return on_off_value_; }
  int get_desired_temperature() const { return desired_temperature_value_; }
  bool is_standalone_mode() const { return standalone_mode_; }

 protected:
  // Sensor storage
  std::map<std::string, sensor::Sensor *> sensors_;
  std::map<std::string, text_sensor::TextSensor *> text_sensors_;
  std::map<std::string, binary_sensor::BinarySensor *> binary_sensors_;

  // Power switch reference (for state sync)
  switch_::Switch *power_switch_{nullptr};

  // Temperature number reference (for state sync)
  number::Number *temperature_number_{nullptr};

  // External temperature sensor (for standalone mode)
  sensor::Sensor *external_temp_sensor_{nullptr};

  // Frame handling
  uint8_t frame_[48];
  int frame_index_ = 0;
  bool waiting_for_start_ = true;

  // Store last TX frame from LCD controller (for command injection)
  uint8_t last_tx_frame_[24];
  bool has_valid_tx_frame_ = false;

  // Pending command to inject (0x00 = none)
  uint8_t pending_command_ = CMD_NO_CHANGE;

  // Pending temperature to inject (0 = none, valid range 8-35)
  uint8_t pending_temperature_ = 0;

  // Track if we just sent a command (to handle heater's response)
  bool awaiting_inject_response_ = false;
  uint32_t inject_response_timeout_ = 0;

  // Standalone mode settings
  bool standalone_mode_ = false;
  uint8_t operating_voltage_ = VOLTAGE_12V;
  uint8_t desired_temp_setting_ = 22;  // Our desired temperature (for standalone)
  bool heater_on_request_ = false;     // Whether we want heater on
  uint8_t pending_on_off_command_ = CMD_NO_CHANGE;  // Pending on/off command
  uint32_t last_tx_time_ = 0;          // Last frame transmission time
  bool awaiting_rx_ = false;           // Waiting for heater response
  uint32_t rx_timeout_ = 0;            // RX timeout timestamp
  uint8_t rx_frame_[24];               // Buffer for RX-only frame in standalone
  int rx_frame_index_ = 0;             // Index for RX frame

  // Parsed data
  float current_temperature_value_ = 0;
  int fan_speed_value_ = 0;
  float supply_voltage_value_ = 0;
  float heat_exchanger_temp_value_ = 0;
  float glow_plug_voltage_value_ = 0;
  float glow_plug_current_value_ = 0;
  float pump_frequency_value_ = 0;
  float fan_voltage_value_ = 0;
  int desired_temperature_value_ = 0;
  int error_code_value_ = 0;
  int run_state_value_ = 0;
  bool on_off_value_ = false;

  std::string run_state_description_ = "Unknown";
  std::string error_code_description_ = "Unknown";

  void parse_frame(const uint8_t *frame, size_t length);
  void parse_rx_frame(const uint8_t *frame, size_t length);
  void reset_frame();
  void send_command(uint8_t command, uint8_t temperature = 0);
  uint16_t calc_crc16(const uint8_t *data, size_t length);

  // Standalone mode methods
  void standalone_loop();
  void build_tx_frame(uint8_t *frame, uint8_t command);
  void send_standalone_frame();

  // Mappings for error and run states
  static const std::map<int, std::string> run_state_map;
  static const std::map<int, std::string> error_code_map;
};

}  // namespace heater_uart
}  // namespace esphome

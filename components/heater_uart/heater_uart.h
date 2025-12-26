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

class HeaterUart : public PollingComponent, public uart::UARTDevice {
 public:
  HeaterUart() = default;

  // Setters for sensors
  void set_sensor(const std::string &key, sensor::Sensor *sensor);
  void set_text_sensor(const std::string &key, text_sensor::TextSensor *text_sensor);
  void set_binary_sensor(const std::string &key, binary_sensor::BinarySensor *binary_sensor);
  void set_power_switch(switch_::Switch *sw) { this->power_switch_ = sw; }
  void set_temperature_number(number::Number *num) { this->temperature_number_ = num; }

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

 protected:
  // Sensor storage
  std::map<std::string, sensor::Sensor *> sensors_;
  std::map<std::string, text_sensor::TextSensor *> text_sensors_;
  std::map<std::string, binary_sensor::BinarySensor *> binary_sensors_;

  // Power switch reference (for state sync)
  switch_::Switch *power_switch_{nullptr};

  // Temperature number reference (for state sync)
  number::Number *temperature_number_{nullptr};

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
  void reset_frame();
  void send_command(uint8_t command, uint8_t temperature = 0);
  uint16_t calc_crc16(const uint8_t *data, size_t length);

  // Mappings for error and run states
  static const std::map<int, std::string> run_state_map;
  static const std::map<int, std::string> error_code_map;
};

}  // namespace heater_uart
}  // namespace esphome

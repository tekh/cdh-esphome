#include "heater_uart.h"
#include "esphome/core/log.h"

namespace esphome {
namespace heater_uart {

static const char *TAG = "heater_uart";

// Mappings for error and run states
const std::map<int, std::string> HeaterUart::run_state_map = {
    {0, "Off / Standby"}, {1, "Start Acknowledge"}, {2, "Glow Plug Pre-heat"},
    {3, "Failed Ignition - Pause for Retry"}, {4, "Ignited – Heating to Full Temp"},
    {5, "Running"}, {6, "Stop Acknowledge"}, {7, "Stopping - Post Run Glow Re-heat"},
    {8, "Cooldown"}
};

const std::map<int, std::string> HeaterUart::error_code_map = {
    {0, "No Error"}, {1, "No Error, But Started"}, {2, "Voltage Too Low"},
    {3, "Voltage Too High"}, {4, "Ignition Plug Failure"}, {5, "Pump Failure – Over Current"},
    {6, "Too Hot"}, {7, "Motor Failure"}, {8, "Serial Connection Lost"},
    {9, "Fire Extinguished"}, {10, "Temperature Sensor Failure"}
};

void HeaterUart::setup() {
    ESP_LOGCONFIG(TAG, "Setting up Heater UART...");
}

void HeaterUart::loop() {
    // Sliding window approach to find valid frames
    // Look for ANY frame starting with 04 04 04
    while (available()) {
        uint8_t byte = read();

        // Shift the frame buffer left and add new byte at the end
        for (int i = 0; i < 31; i++) {
            frame_[i] = frame_[i + 1];
        }
        frame_[31] = byte;

        // Look for frame start: 04 04 04 followed by any command byte
        // Command byte should NOT be 0x04 (to avoid false positives)
        if (frame_[0] == 0x04 && frame_[1] == 0x04 && frame_[2] == 0x04 && frame_[3] != 0x04) {
            parse_frame(frame_, 32);
        }
    }
}

void HeaterUart::update() {
    for (const auto &sensor_entry : sensors_) {
        const std::string &key = sensor_entry.first;
        sensor::Sensor *sensor = sensor_entry.second;

        if (key == "current_temperature")
            sensor->publish_state(current_temperature_value_);
        else if (key == "fan_speed")
            sensor->publish_state(fan_speed_value_);
        else if (key == "supply_voltage")
            sensor->publish_state(supply_voltage_value_);
        else if (key == "heat_exchanger_temp")
            sensor->publish_state(heat_exchanger_temp_value_);
        else if (key == "glow_plug_voltage")
            sensor->publish_state(glow_plug_voltage_value_);
        else if (key == "glow_plug_current")
            sensor->publish_state(glow_plug_current_value_);
        else if (key == "pump_frequency")
            sensor->publish_state(pump_frequency_value_);
        else if (key == "fan_voltage")
            sensor->publish_state(fan_voltage_value_);
        else if (key == "desired_temperature")
            sensor->publish_state(desired_temperature_value_);
    }

    for (const auto &text_entry : text_sensors_) {
        const std::string &key = text_entry.first;
        text_sensor::TextSensor *text_sensor = text_entry.second;

        if (key == "run_state")
            text_sensor->publish_state(run_state_description_);
        else if (key == "error_code")
            text_sensor->publish_state(error_code_description_);
    }

    for (const auto &binary_entry : binary_sensors_) {
        const std::string &key = binary_entry.first;
        binary_sensor::BinarySensor *binary_sensor = binary_entry.second;

        if (key == "on_off_state")
            binary_sensor->publish_state(on_off_value_);
    }
}

void HeaterUart::parse_frame(const uint8_t *frame, size_t length) {
    static uint32_t last_log_time = 0;
    uint32_t now = millis();

    uint8_t cmd = frame[3];

    // Process command 0x0E frames - these contain the main sensor data
    if (cmd == 0x0E && length >= 14) {
        // Frame structure for 0x0E:
        // [0-2]: 04 04 04 (markers)
        // [3]: 0E (command)
        // [4]: Unknown (often 0x34 = 52)
        // [5]: Current temperature (direct celsius value)
        // [6]: Unknown (often 0x16 = 22, maybe set temp?)
        // [7-10]: Unknown
        // [11]: Supply voltage (scaled by 0.1)
        // [12+]: Other sensor data

        current_temperature_value_ = frame[5];
        desired_temperature_value_ = frame[6];  // Might be set temperature
        supply_voltage_value_ = frame[11] * 0.1;

        // Log full frame occasionally
        if (now - last_log_time > 3000) {
            char hex_buffer[150];
            int pos = 0;
            for (int i = 0; i < length && i < 25; i++) {
                pos += snprintf(hex_buffer + pos, sizeof(hex_buffer) - pos, "%02X ", frame[i]);
            }
            ESP_LOGD(TAG, "CMD 0x0E: %s", hex_buffer);
            ESP_LOGD(TAG, "Parsed - Temp: %.1f°C (byte 5), SetTemp: %.1f°C (byte 6), Voltage: %.1fV (byte 11)",
                     current_temperature_value_, desired_temperature_value_, supply_voltage_value_);
            last_log_time = now;
        }

        // Set other values
        fan_speed_value_ = 0;  // TODO: identify in remaining bytes
        run_state_value_ = 5;  // Assume "Running"
        on_off_value_ = true;  // Heater is on
        error_code_value_ = 0;  // No error

        heat_exchanger_temp_value_ = current_temperature_value_;
        glow_plug_voltage_value_ = 0;
        glow_plug_current_value_ = 0;
        pump_frequency_value_ = 0;
        fan_voltage_value_ = 0;

        run_state_description_ = run_state_map.count(run_state_value_)
                                    ? run_state_map.at(run_state_value_)
                                    : "Unknown Run State";

        error_code_description_ = error_code_map.count(error_code_value_)
                                    ? error_code_map.at(error_code_value_)
                                    : "Unknown Error Code";
    } else {
        // For other command types, just log occasionally for analysis
        if (now - last_log_time > 5000) {
            ESP_LOGV(TAG, "Ignoring CMD 0x%02X (not a data frame)", cmd);
            last_log_time = now;
        }
    }
}

void HeaterUart::reset_frame() {
    frame_index_ = 0;
}

void HeaterUart::set_sensor(const std::string &key, sensor::Sensor *sensor) {
    sensors_[key] = sensor;
}

void HeaterUart::set_text_sensor(const std::string &key, text_sensor::TextSensor *text_sensor) {
    text_sensors_[key] = text_sensor;
}

void HeaterUart::set_binary_sensor(const std::string &key, binary_sensor::BinarySensor *binary_sensor) {
    binary_sensors_[key] = binary_sensor;
}

}  // namespace heater_uart
}  // namespace esphome

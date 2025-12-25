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
    const int FRAME_SIZE = 25;  // Reduced frame size for actual data
    const uint8_t FRAME_MARKER = 0x04;

    while (available()) {
        uint8_t byte = read();

        if (waiting_for_start_) {
            // Look for frame start: 3 consecutive 0x04 bytes
            if (byte == FRAME_MARKER) {
                frame_[frame_index_++] = byte;

                // Check if we have 3 consecutive 0x04 bytes
                if (frame_index_ >= 3 &&
                    frame_[0] == FRAME_MARKER &&
                    frame_[1] == FRAME_MARKER &&
                    frame_[2] == FRAME_MARKER) {
                    waiting_for_start_ = false;
                    ESP_LOGD("heater_uart", "Frame start detected");
                }
            } else {
                // Reset if we don't get consecutive 0x04
                frame_index_ = 0;
            }
        } else {
            // Collecting frame data
            frame_[frame_index_++] = byte;

            if (frame_index_ >= FRAME_SIZE) {
                // We have a complete frame
                parse_frame(frame_, frame_index_);
                reset_frame();
            }
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
    if (length < 20) {
        ESP_LOGW(TAG, "Invalid frame length: %d bytes (too short)", length);
        return;
    }

    // Frame structure (based on analysis):
    // [0-2]: 0x04 0x04 0x04 (frame markers)
    // [3]: Command byte (0xFF, 0xCE, 0xC6, etc.)
    // [4]: Temperature (single byte, direct °C value)
    // [8]: Supply voltage (single byte, scaled by 10, e.g., 128 = 12.8V)
    // Other positions vary by command byte

    uint8_t command_byte = frame[3];

    // Extract temperature (appears at position 4 in many frames)
    if (length > 4) {
        current_temperature_value_ = frame[4];
        desired_temperature_value_ = frame[4];  // May need adjustment
    }

    // Extract supply voltage (position 8, scaled by 10)
    if (length > 8) {
        supply_voltage_value_ = frame[8] * 0.1;
    }

    // Try to extract other values based on command byte
    // Command 0xFF seems to be a status frame
    if (command_byte == 0xFF && length >= 25) {
        // Fan speed - need to identify correct position
        // For now, use a placeholder
        fan_speed_value_ = frame[14] * 100;  // Rough estimate

        // Run state
        run_state_value_ = frame[18] % 9;  // Modulo to keep in range

        // On/off state
        on_off_value_ = (frame[23] > 0);

        // Error code
        error_code_value_ = frame[17] % 11;  // Modulo to keep in range
    } else {
        // Default values for other command types
        fan_speed_value_ = 0;
        run_state_value_ = 0;
        on_off_value_ = false;
        error_code_value_ = 0;
    }

    // Placeholder values for sensors we haven't identified yet
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

    ESP_LOGD(TAG, "Parsed frame - Cmd: 0x%02X, Temp: %.1f°C, Voltage: %.1fV, State: %d",
             command_byte, current_temperature_value_, supply_voltage_value_, run_state_value_);
}

void HeaterUart::reset_frame() {
    frame_index_ = 0;
    waiting_for_start_ = true;
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

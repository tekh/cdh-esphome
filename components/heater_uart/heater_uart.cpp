#include "heater_uart.h"
#include "esphome/core/log.h"

namespace esphome {
namespace heater_uart {

static const char *TAG = "heater_uart";

// CRC-16/MODBUS lookup table
static const uint16_t CRC16_TABLE[] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

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
    memset(last_tx_frame_, 0, sizeof(last_tx_frame_));
}

void HeaterUart::loop() {
    const int FRAME_SIZE = 48;
    const int TX_FRAME_END_INDEX = 23;
    const int RX_FRAME_START_INDEX = 24;
    const uint8_t END_OF_FRAME_MARKER = 0x00;

    // Handle timeout for awaiting inject response
    if (awaiting_inject_response_ && millis() > inject_response_timeout_) {
        ESP_LOGW(TAG, "Inject response timeout - returning to normal operation");
        awaiting_inject_response_ = false;
        reset_frame();
    }

    // If awaiting inject response, discard incoming bytes until timeout
    if (awaiting_inject_response_) {
        while (available()) {
            read();  // Discard heater's response to our injected command
        }
        return;
    }

    while (available()) {
        uint8_t byte = read();
        if (waiting_for_start_) {
            if (byte == 0x76) {
                frame_[frame_index_++] = byte;
                waiting_for_start_ = false;
            }
        } else {
            frame_[frame_index_++] = byte;
            if (frame_index_ == TX_FRAME_END_INDEX + 1) {
                if (frame_[21] == END_OF_FRAME_MARKER) {
                    ESP_LOGW(TAG, "Invalid Transmit Packet. Resetting frame.");
                    reset_frame();
                    return;
                }
            }
            if (frame_index_ == FRAME_SIZE) {
                if (frame_[45] == END_OF_FRAME_MARKER && frame_[RX_FRAME_START_INDEX] == 0x76) {
                    // Store the LCD's TX frame for potential command injection
                    memcpy(last_tx_frame_, frame_, 24);
                    has_valid_tx_frame_ = true;

                    parse_frame(frame_, FRAME_SIZE);

                    // Check if we have a pending command or temperature to inject
                    if ((pending_command_ != CMD_NO_CHANGE || pending_temperature_ != 0) && has_valid_tx_frame_) {
                        ESP_LOGI(TAG, "Injecting frame - command: 0x%02X, temperature: %d",
                                 pending_command_, pending_temperature_);
                        send_command(pending_command_, pending_temperature_);
                        pending_command_ = CMD_NO_CHANGE;
                        pending_temperature_ = 0;
                    }
                } else {
                    ESP_LOGW(TAG, "Invalid Receive Packet or incorrect order. Resetting frame.");
                }
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

    // Sync power switch state with actual heater state
    if (power_switch_ != nullptr) {
        power_switch_->publish_state(on_off_value_);
    }

    // Sync temperature number with actual desired temperature
    if (temperature_number_ != nullptr) {
        temperature_number_->publish_state(desired_temperature_value_);
    }
}

void HeaterUart::parse_frame(const uint8_t *frame, size_t length) {
    if (length != 48) {
        ESP_LOGW(TAG, "Invalid frame length: %d bytes (expected 48)", length);
        return;
    }

    const uint8_t *command_frame = &frame[0];
    const uint8_t *response_frame = &frame[24];

    current_temperature_value_ = command_frame[3];
    desired_temperature_value_ = command_frame[4];
    fan_speed_value_ = (response_frame[6] << 8) | response_frame[7];
    supply_voltage_value_ = ((response_frame[4] << 8) | response_frame[5]) * 0.1;
    heat_exchanger_temp_value_ = ((response_frame[10] << 8) | response_frame[11]);
    glow_plug_voltage_value_ = ((response_frame[12] << 8) | response_frame[13]) * 0.1;
    glow_plug_current_value_ = ((response_frame[14] << 8) | response_frame[15]) * 0.01;
    pump_frequency_value_ = response_frame[16] * 0.1;
    fan_voltage_value_ = ((response_frame[8] << 8) | response_frame[9]) * 0.1;
    run_state_value_ = response_frame[2];
    on_off_value_ = response_frame[3] == 1;
    error_code_value_ = response_frame[17];

    run_state_description_ = run_state_map.count(run_state_value_)
                                ? run_state_map.at(run_state_value_)
                                : "Unknown Run State";

    error_code_description_ = error_code_map.count(error_code_value_)
                                ? error_code_map.at(error_code_value_)
                                : "Unknown Error Code";
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

void HeaterUart::turn_on() {
    if (!has_valid_tx_frame_) {
        ESP_LOGW(TAG, "Cannot turn on - no valid TX frame captured yet");
        return;
    }
    ESP_LOGI(TAG, "Queuing START command");
    pending_command_ = CMD_START;
}

void HeaterUart::turn_off() {
    if (!has_valid_tx_frame_) {
        ESP_LOGW(TAG, "Cannot turn off - no valid TX frame captured yet");
        return;
    }
    ESP_LOGI(TAG, "Queuing STOP command");
    pending_command_ = CMD_STOP;
}

void HeaterUart::set_desired_temperature(uint8_t temperature) {
    if (!has_valid_tx_frame_) {
        ESP_LOGW(TAG, "Cannot set temperature - no valid TX frame captured yet");
        return;
    }
    if (temperature < TEMP_MIN || temperature > TEMP_MAX) {
        ESP_LOGW(TAG, "Temperature %d out of range (%d-%d)", temperature, TEMP_MIN, TEMP_MAX);
        return;
    }
    ESP_LOGI(TAG, "Queuing temperature change to %d°C", temperature);
    pending_temperature_ = temperature;
}

void HeaterUart::send_command(uint8_t command, uint8_t temperature) {
    uint8_t tx_frame[24];

    // Copy the last LCD TX frame as base
    memcpy(tx_frame, last_tx_frame_, 24);

    // Modify the command byte (byte 2) if not CMD_NO_CHANGE
    if (command != CMD_NO_CHANGE) {
        tx_frame[2] = command;
    }

    // Modify the temperature byte (byte 4) if specified (non-zero)
    if (temperature != 0) {
        tx_frame[4] = temperature;
    }

    // Recalculate CRC-16/MODBUS for bytes 0-21
    uint16_t crc = calc_crc16(tx_frame, 22);

    // CRC is stored as MSB first in the protocol
    tx_frame[22] = (crc >> 8) & 0xFF;
    tx_frame[23] = crc & 0xFF;

    // Log the frame being sent
    ESP_LOGI(TAG, "Sending command frame:");
    ESP_LOGI(TAG, "  Bytes 0-7:   %02X %02X %02X %02X %02X %02X %02X %02X",
             tx_frame[0], tx_frame[1], tx_frame[2], tx_frame[3],
             tx_frame[4], tx_frame[5], tx_frame[6], tx_frame[7]);
    ESP_LOGI(TAG, "  Bytes 8-15:  %02X %02X %02X %02X %02X %02X %02X %02X",
             tx_frame[8], tx_frame[9], tx_frame[10], tx_frame[11],
             tx_frame[12], tx_frame[13], tx_frame[14], tx_frame[15]);
    ESP_LOGI(TAG, "  Bytes 16-23: %02X %02X %02X %02X %02X %02X %02X %02X",
             tx_frame[16], tx_frame[17], tx_frame[18], tx_frame[19],
             tx_frame[20], tx_frame[21], tx_frame[22], tx_frame[23]);

    // Send the frame
    write_array(tx_frame, 24);
    flush();

    // Set up to ignore the heater's response (timeout after 50ms)
    awaiting_inject_response_ = true;
    inject_response_timeout_ = millis() + 50;

    ESP_LOGI(TAG, "Command sent, awaiting response timeout");
}

uint16_t HeaterUart::calc_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        uint8_t index = data[i] ^ (crc & 0xFF);
        crc = (crc >> 8) ^ CRC16_TABLE[index];
    }
    return crc;
}

}  // namespace heater_uart
}  // namespace esphome

#include "heater_uart.h"
#include "esphome/core/log.h"
#include <cmath>

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
    memset(rx_frame_, 0, sizeof(rx_frame_));

    if (standalone_mode_) {
        ESP_LOGI(TAG, "Running in STANDALONE mode - ESP32 is the heater controller");
        ESP_LOGI(TAG, "Operating voltage: %s", operating_voltage_ == VOLTAGE_12V ? "12V" : "24V");
        ESP_LOGI(TAG, "Initial desired temperature: %d°C", desired_temp_setting_);
    } else {
        ESP_LOGI(TAG, "Running in INJECTION mode - working alongside LCD controller");
    }
}

void HeaterUart::loop() {
    // Use different loop logic depending on mode
    if (standalone_mode_) {
        standalone_loop();
        return;
    }

    // === INJECTION MODE (original logic) ===
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

    // Sync pump number with current setting
    if (pump_number_ != nullptr) {
        pump_number_->publish_state(pump_freq_setting_);
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
    if (standalone_mode_) {
        ESP_LOGI(TAG, "Queuing START command (standalone mode)");
        pending_on_off_command_ = CMD_START;
        heater_on_request_ = true;
    } else {
        if (!has_valid_tx_frame_) {
            ESP_LOGW(TAG, "Cannot turn on - no valid TX frame captured yet");
            return;
        }
        ESP_LOGI(TAG, "Queuing START command (injection mode)");
        pending_command_ = CMD_START;
    }
}

void HeaterUart::turn_off() {
    if (standalone_mode_) {
        ESP_LOGI(TAG, "Queuing STOP command (standalone mode) - entering cooldown");
        pending_on_off_command_ = CMD_STOP;
        heater_on_request_ = false;
        in_cooldown_ = true;  // Start cooldown sequence
    } else {
        if (!has_valid_tx_frame_) {
            ESP_LOGW(TAG, "Cannot turn off - no valid TX frame captured yet");
            return;
        }
        ESP_LOGI(TAG, "Queuing STOP command (injection mode)");
        pending_command_ = CMD_STOP;
    }
}

void HeaterUart::set_desired_temperature(uint8_t temperature) {
    if (temperature < TEMP_MIN || temperature > TEMP_MAX) {
        ESP_LOGW(TAG, "Temperature %d out of range (%d-%d)", temperature, TEMP_MIN, TEMP_MAX);
        return;
    }

    if (standalone_mode_) {
        ESP_LOGI(TAG, "Setting desired temperature to %d°C (standalone mode)", temperature);
        desired_temp_setting_ = temperature;
        // Temperature will be sent in next frame automatically
    } else {
        if (!has_valid_tx_frame_) {
            ESP_LOGW(TAG, "Cannot set temperature - no valid TX frame captured yet");
            return;
        }
        ESP_LOGI(TAG, "Queuing temperature change to %d°C (injection mode)", temperature);
        pending_temperature_ = temperature;
    }
}

void HeaterUart::set_pump_frequency(float frequency) {
    if (frequency < PUMP_FREQ_MIN || frequency > PUMP_FREQ_MAX) {
        ESP_LOGW(TAG, "Pump frequency %.1f out of range (%.1f-%.1f Hz)",
                 frequency, PUMP_FREQ_MIN, PUMP_FREQ_MAX);
        return;
    }

    ESP_LOGI(TAG, "Setting pump frequency to %.1f Hz", frequency);
    pump_freq_setting_ = frequency;
    // Frequency will be sent in next frame automatically (standalone mode)
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

// ==================== STANDALONE MODE IMPLEMENTATION ====================

void HeaterUart::standalone_loop() {
    uint32_t now = millis();

    // If we're waiting for RX response
    if (awaiting_rx_) {
        while (available()) {
            uint8_t byte = read();

            // If we haven't started the frame yet, look for start marker
            if (rx_frame_index_ == 0) {
                if (byte == 0x76) {
                    rx_frame_[rx_frame_index_++] = byte;
                }
                // else discard byte (still looking for sync)
            } else {
                // Already synced, collect remaining bytes
                rx_frame_[rx_frame_index_++] = byte;

                // Check if we have a complete frame
                if (rx_frame_index_ >= 24) {
                    ESP_LOGD(TAG, "RX: %02X %02X %02X %02X %02X %02X %02X %02X ...",
                             rx_frame_[0], rx_frame_[1], rx_frame_[2], rx_frame_[3],
                             rx_frame_[4], rx_frame_[5], rx_frame_[6], rx_frame_[7]);
                    parse_rx_frame(rx_frame_, 24);
                    awaiting_rx_ = false;
                    rx_frame_index_ = 0;
                    return;
                }
            }
        }

        // Check for timeout
        if (now > rx_timeout_) {
            if (rx_frame_index_ > 0) {
                ESP_LOGW(TAG, "RX timeout after %d bytes received", rx_frame_index_);
            } else {
                ESP_LOGW(TAG, "RX timeout - no response from heater");
            }
            awaiting_rx_ = false;
            rx_frame_index_ = 0;
        }

        return;
    }

    // Time to send next frame?
    if (now - last_tx_time_ >= STANDALONE_TX_INTERVAL_MS) {
        send_standalone_frame();
        last_tx_time_ = now;
    }
}

void HeaterUart::build_tx_frame(uint8_t *frame, uint8_t command) {
    // Clear frame
    memset(frame, 0, 24);

    // Byte 0: Start marker
    frame[0] = 0x76;

    // Byte 1: Protocol version
    frame[1] = 0x16;

    // Byte 2: Command
    frame[2] = command;

    // Byte 3: Current temperature
    // In standalone mode, use external temperature sensor if available
    if (external_temp_sensor_ != nullptr && !std::isnan(external_temp_sensor_->state)) {
        frame[3] = static_cast<uint8_t>(external_temp_sensor_->state);
        current_temperature_value_ = external_temp_sensor_->state;
    } else if (current_temperature_value_ > 0) {
        frame[3] = static_cast<uint8_t>(current_temperature_value_);
    } else {
        frame[3] = desired_temp_setting_;  // Fallback if no reading yet
    }

    // Byte 4: Desired temperature
    frame[4] = desired_temp_setting_;

    // Bytes 5-6: Pump frequency (0.1 Hz units)
    // Set both min and max to same value for fixed Hz mode (direct pump control)
    uint8_t pump_freq_raw = static_cast<uint8_t>(pump_freq_setting_ * 10.0f);
    frame[5] = pump_freq_raw;  // Min pump frequency
    frame[6] = pump_freq_raw;  // Max pump frequency (same = fixed Hz mode)

    // Bytes 7-10: Fan RPM
    uint16_t fan_rpm;

    if (in_cooldown_) {
        // Cooldown mode: run fan at high speed to cool heat exchanger
        fan_rpm = COOLDOWN_FAN_RPM;
    } else {
        // Normal operation: scale fan proportionally with pump frequency
        // This maintains proper combustion by matching fan speed to fuel rate.
        float pump_ratio = (pump_freq_setting_ - PUMP_FREQ_MIN) / (PUMP_FREQ_MAX - PUMP_FREQ_MIN);
        pump_ratio = std::max(0.0f, std::min(1.0f, pump_ratio));  // Clamp 0-1
        fan_rpm = static_cast<uint16_t>(
            DEFAULT_MIN_FAN_RPM + pump_ratio * (DEFAULT_MAX_FAN_RPM - DEFAULT_MIN_FAN_RPM)
        );
    }

    // Set both min and max to same value for fixed RPM mode
    frame[7] = (fan_rpm >> 8) & 0xFF;
    frame[8] = fan_rpm & 0xFF;
    frame[9] = (fan_rpm >> 8) & 0xFF;
    frame[10] = fan_rpm & 0xFF;

    // Byte 11: Operating voltage
    frame[11] = operating_voltage_;

    // Byte 12: Fan sensor type
    frame[12] = DEFAULT_FAN_SENSOR;

    // Byte 13: Glow plug power
    frame[13] = DEFAULT_GLOW_POWER;

    // Bytes 14-15: Reserved (zeros)
    frame[14] = 0x00;
    frame[15] = 0x00;

    // Bytes 16-17: Prime pump frequency (zeros = no priming)
    frame[16] = 0x00;
    frame[17] = 0x00;

    // Bytes 18-19: Unknown/reserved
    frame[18] = 0x00;
    frame[19] = 0x00;

    // Bytes 20-21: Altitude (big endian)
    frame[20] = (DEFAULT_ALTITUDE >> 8) & 0xFF;
    frame[21] = DEFAULT_ALTITUDE & 0xFF;

    // Calculate and append CRC-16/MODBUS
    uint16_t crc = calc_crc16(frame, 22);
    frame[22] = (crc >> 8) & 0xFF;  // MSB first
    frame[23] = crc & 0xFF;
}

void HeaterUart::send_standalone_frame() {
    uint8_t tx_frame[24];
    uint8_t command = CMD_NO_CHANGE;

    // Check for pending on/off command
    if (pending_on_off_command_ != CMD_NO_CHANGE) {
        command = pending_on_off_command_;
        pending_on_off_command_ = CMD_NO_CHANGE;
        ESP_LOGI(TAG, "Sending %s command", command == CMD_START ? "START" : "STOP");
    }

    // Build the frame
    build_tx_frame(tx_frame, command);

    // Log frame (debug level to reduce spam)
    ESP_LOGD(TAG, "TX: %02X %02X %02X %02X %02X %02X %02X %02X ...",
             tx_frame[0], tx_frame[1], tx_frame[2], tx_frame[3],
             tx_frame[4], tx_frame[5], tx_frame[6], tx_frame[7]);

    // Send frame
    write_array(tx_frame, 24);
    flush();

    // On half-duplex single-wire, we receive our own TX as echo.
    // Wait briefly for TX to complete, then discard the echo.
    delay(15);  // ~15ms for 24 bytes at 25000 baud (24*10/25000 = 9.6ms) + margin

    // Discard TX echo (24 bytes)
    int discarded = 0;
    while (available() && discarded < 24) {
        read();
        discarded++;
    }
    ESP_LOGD(TAG, "Discarded %d echo bytes", discarded);

    // Set up RX wait for heater response
    awaiting_rx_ = true;
    rx_timeout_ = millis() + STANDALONE_RX_TIMEOUT_MS;
    rx_frame_index_ = 0;
}

void HeaterUart::parse_rx_frame(const uint8_t *frame, size_t length) {
    if (length != 24) {
        ESP_LOGW(TAG, "Invalid RX frame length: %d bytes (expected 24)", length);
        return;
    }

    // Validate CRC (optional but recommended)
    uint16_t received_crc = (frame[22] << 8) | frame[23];
    uint16_t calculated_crc = calc_crc16(frame, 22);
    if (received_crc != calculated_crc) {
        ESP_LOGW(TAG, "RX CRC mismatch: received 0x%04X, calculated 0x%04X", received_crc, calculated_crc);
        // Continue parsing anyway - some heaters may have CRC issues
    }

    // Parse response frame (same structure as bytes 24-47 in injection mode)
    // Byte 0: 0x76 start marker
    // Byte 1: Protocol version
    // Byte 2: Run state
    run_state_value_ = frame[2];

    // Byte 3: On/off state (1 = on)
    on_off_value_ = frame[3] == 1;

    // Bytes 4-5: Supply voltage (big endian, 0.1V units)
    supply_voltage_value_ = ((frame[4] << 8) | frame[5]) * 0.1f;

    // Bytes 6-7: Fan speed RPM (big endian)
    fan_speed_value_ = (frame[6] << 8) | frame[7];

    // Bytes 8-9: Fan voltage (big endian, 0.1V units)
    fan_voltage_value_ = ((frame[8] << 8) | frame[9]) * 0.1f;

    // Bytes 10-11: Heat exchanger temperature (big endian)
    heat_exchanger_temp_value_ = (frame[10] << 8) | frame[11];

    // Bytes 12-13: Glow plug voltage (big endian, 0.1V units)
    glow_plug_voltage_value_ = ((frame[12] << 8) | frame[13]) * 0.1f;

    // Bytes 14-15: Glow plug current (big endian, 0.01A units)
    glow_plug_current_value_ = ((frame[14] << 8) | frame[15]) * 0.01f;

    // Byte 16: Pump frequency (0.1 Hz units)
    pump_frequency_value_ = frame[16] * 0.1f;

    // Byte 17: Error code
    error_code_value_ = frame[17];

    // Note: Current temperature comes from our TX frame (byte 3), not RX
    // In standalone mode, we'd need an external sensor - for now use heat exchanger
    // or keep the last value

    // Update desired temperature to reflect what we're sending
    desired_temperature_value_ = desired_temp_setting_;

    // Map run state and error code to descriptions
    run_state_description_ = run_state_map.count(run_state_value_)
                                ? run_state_map.at(run_state_value_)
                                : "Unknown Run State";

    error_code_description_ = error_code_map.count(error_code_value_)
                                ? error_code_map.at(error_code_value_)
                                : "Unknown Error Code";

    ESP_LOGD(TAG, "RX: State=%d (%s), OnOff=%d, Fan=%d RPM, Supply=%.1fV, HX=%.0f°C, Error=%d%s",
             run_state_value_, run_state_description_.c_str(), on_off_value_,
             fan_speed_value_, supply_voltage_value_, heat_exchanger_temp_value_, error_code_value_,
             in_cooldown_ ? " [COOLDOWN]" : "");

    float hx_temp = heat_exchanger_temp_value_;

    // Cooldown monitoring: check if heat exchanger has cooled enough
    if (in_cooldown_ && standalone_mode_) {
        if (hx_temp <= COOLDOWN_TARGET_TEMP) {
            ESP_LOGI(TAG, "Cooldown complete: HX temp %.0f°C <= %.0f°C target. Fan stopping.",
                     hx_temp, COOLDOWN_TARGET_TEMP);
            in_cooldown_ = false;
        } else {
            ESP_LOGD(TAG, "Cooldown in progress: HX temp %.0f°C (target: %.0f°C), Fan at %d RPM",
                     hx_temp, COOLDOWN_TARGET_TEMP, COOLDOWN_FAN_RPM);
        }
        return;  // Skip normal safety logic during cooldown
    }

    // Heat exchanger temperature safety logic (only when heater is running)
    if (on_off_value_ && standalone_mode_) {
        // CRITICAL: Emergency shutdown if heat exchanger is too hot
        if (hx_temp > HX_TEMP_CRITICAL) {
            ESP_LOGE(TAG, "CRITICAL: Heat exchanger %.0f°C > %.0f°C limit! Emergency shutdown!",
                     hx_temp, HX_TEMP_CRITICAL);
            pending_on_off_command_ = CMD_STOP;
            heater_on_request_ = false;
            in_cooldown_ = true;  // Enter cooldown mode
            return;
        }

        // Heat exchanger too cold: increase pump to add more fuel/heat
        if (hx_temp < HX_TEMP_LOW) {
            float new_freq = pump_freq_setting_ + PUMP_FREQ_STEP;
            if (new_freq <= PUMP_FREQ_MAX) {
                ESP_LOGI(TAG, "HX temp %.0f°C < %.0f°C: increasing pump %.1f -> %.1f Hz",
                         hx_temp, HX_TEMP_LOW, pump_freq_setting_, new_freq);
                pump_freq_setting_ = new_freq;
            }
        }
        // Heat exchanger at/above target: decrease pump to reduce fuel/heat
        else if (hx_temp >= HX_TEMP_HIGH) {
            float new_freq = pump_freq_setting_ - PUMP_FREQ_STEP;
            if (new_freq >= PUMP_FREQ_MIN) {
                ESP_LOGI(TAG, "HX temp %.0f°C >= %.0f°C: decreasing pump %.1f -> %.1f Hz",
                         hx_temp, HX_TEMP_HIGH, pump_freq_setting_, new_freq);
                pump_freq_setting_ = new_freq;
            }
        }
    }
}

}  // namespace heater_uart
}  // namespace esphome

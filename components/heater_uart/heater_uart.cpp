#include "heater_uart.h"
#include "esphome/core/log.h"
#include <cmath>

#include "heater_output.h"

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

// Vevor error codes (from the zatakon/vevor_heater_control reverse engineering)
const std::map<int, std::string> HeaterUart::vevor_error_map = {
    {0x00, "No Error"}, {0x01, "E10 - Startup Failure"}, {0x02, "E08 - Check Fuel Level"},
    {0x03, "E01 - Supply Voltage Overrun"}, {0x04, "E04 - ?"}, {0x05, "E05 - ?"},
    {0x06, "E04 - Fuel Pump Failure"}, {0x07, "E06 - Fan Failure"},
    {0x08, "E03 - Check Ignition"}, {0x09, "E05 - Overheat"}
};

void HeaterUart::setup() {
    ESP_LOGCONFIG(TAG, "Setting up Heater UART...");
    memset(last_tx_frame_, 0, sizeof(last_tx_frame_));
    memset(rx_frame_, 0, sizeof(rx_frame_));

    // Re-apply the profile selected by `heater_model:` (fallback: 8kW defaults).
    // NOTE: ambient heat limit is NOT reset here - it comes from YAML only.
    this->profile_ = profile_for(this->profile_.model);

    if (standalone_mode_) {
        // Start the pump at the profile's ignition rate
        pump_freq_setting_ = profile_.pump_freq_initial;

        ESP_LOGI(TAG, "Running in STANDALONE mode - ESP32 is the heater controller");
        ESP_LOGI(TAG, "Heater model: %s | pump %.1f-%.1f Hz | fan %u-%u RPM | HX critical %.0f C | temp %u-%u C",
                 profile_.name, profile_.pump_freq_min, profile_.pump_freq_max,
                 profile_.fan_rpm_min, profile_.fan_rpm_max, profile_.hx_temp_critical,
                 profile_.temp_min, profile_.temp_max);
        ESP_LOGI(TAG, "Protocol: %s @ %u baud (inverted: %s)",
                 profile_.protocol == HeaterProtocol::VEVOR_UART ? "VEVOR-UART (0xAA/0x66)"
                                                                 : "BDAP (0x76/0x16)",
                 (unsigned) profile_.uart_baud_rate, profile_.uart_inverted ? "yes" : "no");
        ESP_LOGI(TAG, "Operating voltage: %s", operating_voltage_ == VOLTAGE_12V ? "12V" : "24V");
        ESP_LOGI(TAG, "Altitude: %d m", altitude_);
        ESP_LOGI(TAG, "Initial desired temperature: %.1f°C", desired_temp_setting_);
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
        else if (key == "glow_a")
            sensor->publish_state(glow_a_value_);
        else if (key == "glow_b")
            sensor->publish_state(glow_b_value_);
        else if (key == "glow_c")
            sensor->publish_state(glow_c_value_);
        else if (key == "power_level_actual")
            sensor->publish_state(power_level_actual_value_);
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
        else if (key == "auto_shutdown_active")
            binary_sensor->publish_state(in_auto_shutdown_);
        else if (key == "standby_active")
            binary_sensor->publish_state(in_standby_);
        else if (key == "priming_active")
            binary_sensor->publish_state(is_priming_);
    }

    // Sync temperature number with actual desired temperature
    if (temperature_number_ != nullptr) {
        temperature_number_->publish_state(desired_temperature_value_);
    }

    // Sync pump number with current setting
    if (pump_number_ != nullptr) {
        // Manual-HEAT profiles: mirror the user's setpoint, not the send value,
        // so the ignition warm-up ramp never clobbers what they set.
        if (profile_.heat_mode_manual && heater_mode_ == HeaterMode::ON) {
            pump_number_->publish_state(pump_freq_user_request_);
        } else {
            pump_number_->publish_state(pump_freq_setting_);
        }
    }

    // Sync power level dial (level-commanded profiles: Vevor)
    if (power_level_number_ != nullptr) {
        power_level_number_->publish_state(get_power_level_display());
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
        // Set pump to initial ignition frequency - will ramp up after flame stabilizes
        pump_freq_setting_ = profile_.pump_freq_initial;
        last_pump_adjust_time_ = millis();  // Reset adjustment timer
        ESP_LOGI(TAG, "Pump frequency set to %.1f Hz for ignition", pump_freq_setting_);
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
        in_auto_shutdown_ = false;  // Manual turn off clears auto-shutdown state
    } else {
        if (!has_valid_tx_frame_) {
            ESP_LOGW(TAG, "Cannot turn off - no valid TX frame captured yet");
            return;
        }
        ESP_LOGI(TAG, "Queuing STOP command (injection mode)");
        pending_command_ = CMD_STOP;
    }
}

void HeaterUart::set_desired_temperature(float temperature) {
    // Round to nearest 0.5 degree
    float rounded = std::round(temperature * 2.0f) / 2.0f;

    if (rounded < profile_.temp_min || rounded > profile_.temp_max) {
        ESP_LOGW(TAG, "Temperature %.1f out of range (%d-%d)", rounded, profile_.temp_min, profile_.temp_max);
        return;
    }

    if (standalone_mode_) {
        ESP_LOGI(TAG, "Setting desired temperature to %.1f°C (standalone mode)", rounded);
        desired_temp_setting_ = rounded;
        // Temperature will be sent in next frame automatically
    } else {
        if (!has_valid_tx_frame_) {
            ESP_LOGW(TAG, "Cannot set temperature - no valid TX frame captured yet");
            return;
        }
        ESP_LOGI(TAG, "Queuing temperature change to %.1f°C (injection mode)", rounded);
        pending_temperature_ = static_cast<uint8_t>(std::round(rounded));
    }
}

void HeaterUart::set_pump_frequency(float frequency) {
    if (frequency < profile_.pump_freq_min || frequency > profile_.pump_freq_max) {
        ESP_LOGW(TAG, "Pump frequency %.1f out of range (%.1f-%.1f Hz)",
                 frequency, profile_.pump_freq_min, profile_.pump_freq_max);
        return;
    }

    ESP_LOGI(TAG, "Setting pump frequency to %.1f Hz", frequency);
    pump_freq_setting_ = frequency;
    pump_freq_user_request_ = frequency;  // user's setpoint (manual-HEAT authority)
    // Frequency will be sent in next frame automatically (standalone mode)
}

void HeaterUart::set_power_level(uint8_t level) {
    if (level < 1) level = 1;
    if (level > 10) level = 10;

    // The thermostat and the TX boundary both work in the Hz domain; the level
    // is the honest unit for level-commanded hardware. Single shared mapping
    // (output_domain, host-tested): level -> normalized fraction -> Hz.
    const float hz = output_domain::hz_from_normalized(
        output_domain::normalized_from_level(level), profile_);

    ESP_LOGI(TAG, "Setting power level %u (%.2f Hz equivalent on %s)", level, hz, profile_.name);
    set_pump_frequency(hz);
}

uint8_t HeaterUart::get_power_level_display() const {
    // Mirror the user's setpoint on manual-HEAT profiles (the ignition ramp
    // must not show up on the dial), otherwise the firmware's live request.
    const float hz = (profile_.heat_mode_manual && heater_mode_ == HeaterMode::ON)
                         ? pump_freq_user_request_
                         : pump_freq_setting_;
    return output_domain::level_from_normalized(
        output_domain::normalized_from_hz(hz, profile_));
}

void HeaterUart::set_heater_mode(HeaterMode mode) {
    HeaterMode old_mode = heater_mode_;
    heater_mode_ = mode;

    const char *mode_str = (mode == HeaterMode::OFF) ? "Off" :
                           (mode == HeaterMode::AUTO) ? "Auto" : "On";
    ESP_LOGI(TAG, "Heater mode set to: %s", mode_str);

    if (mode == HeaterMode::OFF) {
        // Force off - stop heater if running
        if (on_off_value_ || heater_on_request_) {
            ESP_LOGI(TAG, "Mode Off: stopping heater");
            pending_on_off_command_ = CMD_STOP;
            heater_on_request_ = false;
            in_cooldown_ = true;
        }
        in_standby_ = false;
        in_auto_shutdown_ = false;

    } else if (mode == HeaterMode::AUTO) {
        // Check if we should start or enter standby
        float room_temp = current_temperature_value_;
        float threshold = desired_temp_setting_ - auto_shutdown_hysteresis_;

        if (is_room_sensor_stale()) {
            // No blind starts: the room temperature is stale (sensor dead or
            // never read). Park in standby; the thermostat's standby check
            // starts the heater once fresh readings arrive. If the heater is
            // running, stop it (burning on blind data).
            ESP_LOGW(TAG, "Mode Auto: room sensor stale - no blind start, parking in standby");
            in_standby_ = true;
            if (on_off_value_ || heater_on_request_) {
                pending_on_off_command_ = CMD_STOP;
                heater_on_request_ = false;
                in_cooldown_ = true;
            }
        } else if (room_temp < threshold) {
            // Cold enough - start heater
            ESP_LOGI(TAG, "Mode Auto: room %.1f°C < %.1f°C threshold, starting heater",
                     room_temp, threshold);
            pending_on_off_command_ = CMD_START;
            heater_on_request_ = true;
            pump_freq_setting_ = profile_.pump_freq_initial;
            last_pump_adjust_time_ = millis();
            in_standby_ = false;
        } else {
            // Too warm - enter standby, wait for temp drop
            ESP_LOGI(TAG, "Mode Auto: room %.1f°C >= %.1f°C threshold, entering standby",
                     room_temp, threshold);
            in_standby_ = true;
            // If heater is running, stop it
            if (on_off_value_ || heater_on_request_) {
                pending_on_off_command_ = CMD_STOP;
                heater_on_request_ = false;
                in_cooldown_ = true;
            }
        }
        in_auto_shutdown_ = false;

    } else if (mode == HeaterMode::ON) {
        // Force on - start heater regardless of temperature
        ESP_LOGI(TAG, "Mode On: forcing heater start");
        pending_on_off_command_ = CMD_START;
        heater_on_request_ = true;
        // Start level on a cold start. Manual-HEAT profiles start at FULL POWER and
        // ramp down as the room approaches the requested temperature
        // (thermostat_control()); everything else keeps the ignition-rate default.
        pump_freq_setting_ = profile_.heat_mode_manual ? profile_.pump_freq_max
                                                       : profile_.pump_freq_initial;
        ESP_LOGI(TAG, "Mode On: start level %.1f Hz (%.1f = full power)", pump_freq_setting_,
                 profile_.pump_freq_max);
        last_pump_adjust_time_ = millis();
        in_standby_ = false;
        in_auto_shutdown_ = false;
    }
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

    // Route to the protocol-specific loop (Vevor has its own framing/cadence)
    if (profile_.protocol == HeaterProtocol::VEVOR_UART) {
        vevor_loop();
        return;
    }

        // === AMBIENT HEAT LIMIT CHECK (HEAT Mode ONLY) ===
	if (heater_mode_ == HeaterMode::ON && current_temperature_value_ > ambient_heat_limit_) {
            ESP_LOGW(TAG, "Ambient Limit Exceeded in HEAT mode: Room %.1f°C > Limit %.1f°C. Forcing shutdown.",
                     current_temperature_value_, ambient_heat_limit_);
            pending_on_off_command_ = CMD_STOP;
            in_cooldown_ = true; // Start cooldown sequence immediately
        }
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

    // Check priming timeout (60 seconds)
    if (is_priming_ && (now - priming_start_time_ >= 60000)) {
        ESP_LOGI(TAG, "Fuel priming timeout reached (60 seconds) - stopping priming");
        stop_priming();
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
    // In standalone mode, use the physics-guarded external room-temperature
    // reading (refresh_room_temperature rejects NaN and implausible DHT22
    // glitch samples and holds the last trusted value).
    refresh_room_temperature();
    if (current_temperature_value_ > 0) {
        frame[3] = static_cast<uint8_t>(current_temperature_value_);
    } else {
        frame[3] = static_cast<uint8_t>(std::round(desired_temp_setting_));  // Fallback if no reading yet
    }

    // Byte 4: Desired temperature (convert float to uint8_t for protocol)
    frame[4] = static_cast<uint8_t>(std::round(desired_temp_setting_));

    // Bytes 5-6: Pump frequency (0.1 Hz units)
    // Set both min and max to same value for fixed Hz mode (direct pump control)
    // During priming, override with the profile's prime rate to run pump without starting heater
    uint8_t pump_freq_raw;
    if (is_priming_) {
        pump_freq_raw = static_cast<uint8_t>(profile_.pump_prime_freq * 10.0f);  // Hz -> 0.1 Hz units
    } else {
        pump_freq_raw = static_cast<uint8_t>(pump_freq_setting_ * 10.0f);
    }
    frame[5] = pump_freq_raw;  // Min pump frequency
    frame[6] = pump_freq_raw;  // Max pump frequency (same = fixed Hz mode)

    // Bytes 7-10: Fan RPM
    uint16_t fan_rpm;

    if (in_cooldown_ || run_state_value_ == RUN_STATE_COOLDOWN) {
        // Cooldown mode: run fan at high speed to cool heat exchanger
        fan_rpm = profile_.cooldown_fan_rpm;
    } else if (run_state_value_ == RUN_STATE_IGNITED && heat_exchanger_temp_value_ < profile_.ignition_hx_threshold) {
        // State 4 (Ignited) with HX below threshold: keep fan at moderate speed for stable combustion
        // Pump can ramp up, but fan should wait until HX >= threshold to scale with pump
        fan_rpm = profile_.ignition_fan_rpm;
        ESP_LOGD(TAG, "Ignition: HX %.0f°C < %.0f°C, fan at ignition speed %d RPM",
                 heat_exchanger_temp_value_, profile_.ignition_hx_threshold, fan_rpm);
    } else {
        // Normal operation: scale fan proportionally with pump frequency
        // This maintains proper combustion by matching fan speed to fuel rate.
        float pump_ratio = (pump_freq_setting_ - profile_.pump_freq_min) / (profile_.pump_freq_max - profile_.pump_freq_min);
        pump_ratio = std::max(0.0f, std::min(1.0f, pump_ratio));  // Clamp 0-1
        fan_rpm = static_cast<uint16_t>(
            profile_.fan_rpm_min + pump_ratio * (profile_.fan_rpm_max - profile_.fan_rpm_min)
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
    frame[12] = profile_.fan_sensor;

    // Byte 13: Glow plug power
    frame[13] = profile_.glow_power;

    // Bytes 14-15: Reserved (zeros)
    frame[14] = 0x00;
    frame[15] = 0x00;

    // Bytes 16-17: Prime pump frequency (zeros = no priming)
    if (is_priming_) {
        // Priming active: set prime pump frequency to the profile's prime rate
        frame[16] = static_cast<uint8_t>(profile_.pump_prime_freq * 10.0f);  // Hz -> 0.1 Hz units
        frame[17] = 0x5A;  // Manual pump priming mode (protocol spec: 0x5A = priming, 0x00 = normal)
    } else {
        frame[16] = 0x00;
        frame[17] = 0x00;
    }

    // Bytes 18-19: Unknown/reserved
    frame[18] = 0x00;
    frame[19] = 0x00;

    // Bytes 20-21: Altitude (big endian, configurable via YAML)
    frame[20] = (altitude_ >> 8) & 0xFF;
    frame[21] = altitude_ & 0xFF;

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

    // Log frame (debug level to reduce spam, but always log during priming)
    if (is_priming_) {
        ESP_LOGI(TAG, "TX (PRIMING): %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                 tx_frame[0], tx_frame[1], tx_frame[2], tx_frame[3],
                 tx_frame[4], tx_frame[5], tx_frame[6], tx_frame[7],
                 tx_frame[8], tx_frame[9], tx_frame[10], tx_frame[11],
                 tx_frame[12], tx_frame[13], tx_frame[14], tx_frame[15],
                 tx_frame[16], tx_frame[17]);
    } else {
        ESP_LOGD(TAG, "TX: %02X %02X %02X %02X %02X %02X %02X %02X ...",
                 tx_frame[0], tx_frame[1], tx_frame[2], tx_frame[3],
                 tx_frame[4], tx_frame[5], tx_frame[6], tx_frame[7]);
    }

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

    float hx_temp = heat_exchanger_temp_value_;
    float room_temp = current_temperature_value_;  // From external sensor (set in build_tx_frame)
    float target_temp = static_cast<float>(desired_temp_setting_);

    ESP_LOGD(TAG, "RX: State=%d (%s), OnOff=%d, Fan=%d RPM, HX=%.0f°C, Room=%.1f°C, Target=%.1f°C, Pump=%.1fHz%s",
             run_state_value_, run_state_description_.c_str(), on_off_value_,
             fan_speed_value_, hx_temp, room_temp, target_temp, pump_freq_setting_,
             in_cooldown_ ? " [COOLDOWN]" : "");

    // Cooldown monitoring: check if heat exchanger has cooled enough
    if (in_cooldown_ && standalone_mode_) {
        if (hx_temp <= profile_.cooldown_target_temp) {
            ESP_LOGI(TAG, "Cooldown complete: HX temp %.0f°C <= %.0f°C target. Fan stopping.",
                     hx_temp, profile_.cooldown_target_temp);
            in_cooldown_ = false;
        } else {
            ESP_LOGD(TAG, "Cooldown in progress: HX temp %.0f°C (target: %.0f°C), Fan at %d RPM",
                     hx_temp, profile_.cooldown_target_temp, profile_.cooldown_fan_rpm);
        }
        return;  // Skip thermostat logic during cooldown
    }

    // Standby check, thermostat, safety limits and auto-restart (shared with Vevor protocol)
    thermostat_control();
}

// Physics guard for the external room sensor (DHT22, standalone mode).
// A DHT22 right after a failed read can emit a sample that is valid but
// garbage (2026-10-21: 18.4C against a 19.2C trend, which tripped a false
// standby->start in AUTO mode). Such samples must not be acted upon:
//   - NaN / warning readings are rejected outright.
//   - A non-NaN reading more than ROOM_TEMP_MAX_STEP away from the last
//     accepted value is a glitch sample: reject it, hold the last trusted
//     value, and do NOT refresh the staleness timestamp (observed field
//     rate ~0.3C/min, so 0.5C per 10 s sensor interval is 10x headroom).
//   - Accepted readings refresh last_valid_room_temp_time_, so a DHT22 that
//     is dead or persistently implausible is declared blind after
//     ROOM_SENSOR_TIMEOUT_MS and the thermostat stops the heater.
//   - At boot, no single sample may become the baseline (a DHT22 can misfire
//     on its first conversion after power-on). The first ROOM_TEMP_BOOT_SAMPLES
//     non-NaN samples, at least ROOM_TEMP_BOOT_MIN_SPACING_MS apart so each is
//     a distinct sensor conversion (~30 s total), must agree within
//     ROOM_TEMP_BOOT_MAX_SPREAD; the baseline is their median. A failed window
//     is discarded and retried with the next three samples. Until the baseline
//     is established last_valid_room_temp_time_ stays 0, is_room_sensor_stale()
//     is true, and AUTO keeps the heater off - the safe state right after a
//     reboot, when the relay has just de-energized.
bool HeaterUart::refresh_room_temperature() {
    if (external_temp_sensor_ == nullptr) return false;
    const float sample = external_temp_sensor_->state;
    if (std::isnan(sample)) return false;

    if (!room_temp_accepted_ever_) {
        // Boot consensus window: require 3 distinct readings to agree before
        // establishing the baseline.
        const uint32_t now = millis();
        if (room_temp_boot_sample_count_ > 0 &&
            now - room_temp_boot_last_sample_time_ < ROOM_TEMP_BOOT_MIN_SPACING_MS) {
            return false;  // same DHT22 reading as the one already counted; wait for the next conversion
        }
        room_temp_boot_sample_[room_temp_boot_sample_count_++] = sample;
        room_temp_boot_last_sample_time_ = now;
        if (room_temp_boot_sample_count_ < ROOM_TEMP_BOOT_SAMPLES) return false;
        room_temp_boot_sample_count_ = 0;

        const float a = room_temp_boot_sample_[0];
        const float b = room_temp_boot_sample_[1];
        const float c = room_temp_boot_sample_[2];
        const float mn = a < b ? (a < c ? a : c) : (b < c ? b : c);
        const float mx = a > b ? (a > c ? a : c) : (b > c ? b : c);
        const float spread = mx - mn;
        if (spread > ROOM_TEMP_BOOT_MAX_SPREAD) {
            ESP_LOGW(TAG, "Boot room-temp samples disagree (spread %.1fC > %.1fC): retrying 3-sample consensus",
                     spread, ROOM_TEMP_BOOT_MAX_SPREAD);
            return false;  // discard window; next three samples start over
        }
        const float median = a + b + c - mn - mx;  // median of three
        room_temp_accepted_ever_ = true;
        current_temperature_value_ = median;
        last_valid_room_temp_time_ = now;
        ESP_LOGI(TAG, "Room temp baseline %.1fC established (3-sample consensus, spread %.1fC)",
                 median, spread);
        return true;
    }

    if (std::abs(sample - current_temperature_value_) > ROOM_TEMP_MAX_STEP) {
        // Implausible jump: DHT22 glitch sample. Hold the last trusted value
        // and deliberately do NOT refresh last_valid_room_temp_time_.
        static uint32_t last_reject_log = 0;  // loop-task context, single instance
        if (millis() - last_reject_log >= 60000) {
            last_reject_log = millis();
            ESP_LOGW(TAG, "Room sensor glitch sample %.1fC rejected (last accepted %.1fC, max step %.1fC/interval): holding last trusted value",
                     sample, current_temperature_value_, ROOM_TEMP_MAX_STEP);
        }
        return false;
    }

    current_temperature_value_ = sample;
    last_valid_room_temp_time_ = millis();
    return true;
}

bool HeaterUart::is_room_sensor_stale() const {
    if (!standalone_mode_ || external_temp_sensor_ == nullptr) return false;
    // Staleness is defined on ACCEPTED readings (refresh_room_temperature):
    // NaN and implausible samples do not refresh the timer, so a sensor that
    // is dead or stuck emitting garbage is blind after the timeout.
    if (last_valid_room_temp_time_ == 0) return true;  // no accepted reading since boot
    return (millis() - last_valid_room_temp_time_) > ROOM_SENSOR_TIMEOUT_MS;
}

void HeaterUart::thermostat_control() {
    float hx_temp = heat_exchanger_temp_value_;
    float room_temp = current_temperature_value_;  // external sensor, refreshed at TX time
    float target_temp = static_cast<float>(desired_temp_setting_);

    // === ROOM-SENSOR HEALTH GATE (standalone mode) ===
    // While the external room sensor keeps producing ACCEPTED readings, the
    // timestamp is refreshed in refresh_room_temperature() (TX path, 1 Hz).
    // NaN and physically implausible samples do NOT refresh it, so a DHT22
    // that is dead or stuck emitting garbage is blind after 10 min. Then the
    // last known value could hide an overheat (the 26C ambient limit reads
    // the same stale value) and a stale cold value could trigger a blind
    // start. Stop a running heater and block all auto-start/stop logic until
    // fresh data arrives. (User requirement, 2026-10-20: DHT22 in warning
    // state > 10 min -> heater off; 2026-10-21: also don't act on glitch
    // samples.)
    if (standalone_mode_ && external_temp_sensor_ != nullptr && is_room_sensor_stale()) {
        if (on_off_value_ || heater_on_request_) {
            ESP_LOGW(TAG, "Room sensor stale > %.0f min (last valid %.1fC): stopping heater",
                     ROOM_SENSOR_TIMEOUT_MS / 60000.0f, current_temperature_value_);
            pending_on_off_command_ = CMD_STOP;
            heater_on_request_ = false;
            in_cooldown_ = true;
            in_auto_shutdown_ = false;
            if (heater_mode_ == HeaterMode::AUTO) {
                in_standby_ = true;  // re-arm auto-restart for when the sensor recovers
            }
        }
        return;  // block standby/auto-restart/auto-shutdown until fresh readings
    }

    // === STANDBY CHECK (AUTO mode) ===
    // If in standby, check if temperature has dropped enough to start
    if (in_standby_ && heater_mode_ == HeaterMode::AUTO && standalone_mode_) {
        float threshold = target_temp - auto_shutdown_hysteresis_;
        if (room_temp < threshold) {
            ESP_LOGI(TAG, "Standby->Start: room %.1f°C < %.1f°C threshold", room_temp, threshold);
            pending_on_off_command_ = CMD_START;
            heater_on_request_ = true;
            pump_freq_setting_ = profile_.pump_freq_initial;
            last_pump_adjust_time_ = millis();
            in_standby_ = false;
        }
    }

    // Thermostat and safety logic
    // During early ignition (states 1-3) and shutdown (states 6-8), the heater's internal
    // controller manages pump/fan - we should NOT interfere with those sequences.
    // State 4 (Ignited): Ramp pump to max to reach operating temperature quickly.
    // State 5 (Running): Normal thermostat control.
    // Note: In ON mode, we skip thermostat control but still respect safety limits.
    if (on_off_value_ && standalone_mode_) {
        uint32_t now = millis();

        if (run_state_value_ == RUN_STATE_IGNITED) {
            // Heater has ignited - ramp pump to max to reach operating temp quickly
            // But respect the adjustment interval - thermal response takes time
            if (pump_freq_setting_ < profile_.pump_freq_max) {
                if (now - last_pump_adjust_time_ >= PUMP_ADJUST_INTERVAL_MS) {
                    // Scale step based on how cold the room is
                    float temp_gap = target_temp - room_temp;
                    float multiplier = (temp_gap >= 5.0f) ? 3.0f : (temp_gap >= 2.0f) ? 2.0f : 1.0f;
                    float step = profile_.pump_freq_step * multiplier;

                    float new_freq = std::min(pump_freq_setting_ + step, profile_.pump_freq_max);
                    ESP_LOGI(TAG, "Ignited (state 4): gap %.1f°C, ramping pump %.1f -> %.1f Hz (x%.0f)",
                             temp_gap, pump_freq_setting_, new_freq, multiplier);
                    pump_freq_setting_ = new_freq;
                    last_pump_adjust_time_ = now;
                }
            }
            return;
        }

        if (run_state_value_ != RUN_STATE_RUNNING) {
            // Heater is in early startup (1-3) or shutdown (6-8) - let it handle pump/fan
            ESP_LOGD(TAG, "Ignition/shutdown in progress (state %d) - thermostat paused", run_state_value_);
            return;
        }

        // === SAFETY: Emergency shutdown if heat exchanger is critically hot ===
        if (hx_temp > profile_.hx_temp_critical) {
            ESP_LOGE(TAG, "CRITICAL: Heat exchanger %.0f°C > %.0f°C limit! Emergency shutdown!",
                     hx_temp, profile_.hx_temp_critical);
            pending_on_off_command_ = CMD_STOP;
            heater_on_request_ = false;
            in_cooldown_ = true;
            return;
        }

        // === ROOM TEMPERATURE THERMOSTAT (AUTO mode only) ===
        // In ON mode, skip thermostat control - user wants heat regardless of temperature
        // Only adjust if enough time has passed - thermal mass means changes take time
        if (now - last_pump_adjust_time_ < PUMP_ADJUST_INTERVAL_MS) {
            return;  // Wait for thermal response before next adjustment
        }

        float new_freq = pump_freq_setting_;

        // Room temperature control - different behavior for AUTO vs HEAT mode
        if (heater_mode_ == HeaterMode::AUTO) {
            // AUTO mode: Three-phase thermostat control with auto-shutdown
            // 1. Heating phase: room < (target - approach_threshold) → increase pump
            // 2. Approach phase: within approach_threshold of target → decrease pump slowly
            // 3. At/above target: decrease pump rapidly to prevent overshoot
            // 4. Auto-shutdown when at minimum pump and temp exceeded

            float approach_temp = target_temp - approach_threshold_;
            float temp_gap = std::abs(room_temp - target_temp);

            if (room_temp < approach_temp) {
                // HEATING PHASE: Room is cold, increase pump
                // Larger gaps = more aggressive increase
                float multiplier = (temp_gap >= 5.0f) ? 3.0f : (temp_gap >= 2.0f) ? 2.0f : 1.0f;
                float step = profile_.pump_freq_step * multiplier;
                new_freq = pump_freq_setting_ + step;
                if (new_freq <= profile_.pump_freq_max && new_freq != pump_freq_setting_) {
                    ESP_LOGI(TAG, "Heating: room %.1f°C < %.1f°C, pump %.1f -> %.1f Hz (x%.0f)",
                             room_temp, approach_temp, pump_freq_setting_, new_freq, multiplier);
                }
            } else if (room_temp < target_temp) {
                // APPROACH PHASE: Within approach threshold, start reducing pump slowly
                // This prevents overshoot by slowing down before reaching target
                float step = profile_.pump_freq_step;  // Always 1x step for gentle approach
                new_freq = pump_freq_setting_ - step;
                if (new_freq >= profile_.pump_freq_min && new_freq != pump_freq_setting_) {
                    ESP_LOGI(TAG, "Approach: room %.1f°C approaching %.1f°C, slowing pump %.1f -> %.1f Hz",
                             room_temp, target_temp, pump_freq_setting_, new_freq);
                }
            } else {
                // AT/ABOVE TARGET: Reduce pump rapidly to prevent overshoot
                // Larger overshoot = more aggressive reduction
                float overshoot = room_temp - target_temp;
                float multiplier = (overshoot >= 1.0f) ? 3.0f : (overshoot >= 0.5f) ? 2.0f : 1.0f;
                float step = profile_.pump_freq_step * multiplier;
                new_freq = pump_freq_setting_ - step;
                if (new_freq >= profile_.pump_freq_min && new_freq != pump_freq_setting_) {
                    ESP_LOGI(TAG, "At target: room %.1f°C >= %.1f°C, reducing pump %.1f -> %.1f Hz (x%.0f)",
                             room_temp, target_temp, pump_freq_setting_, new_freq, multiplier);
                }
            }
        } else if (heater_mode_ == HeaterMode::ON) {
            if (profile_.heat_mode_manual) {
                // HEAT (v2): cold start at full power, then ramp DOWN as the room
                // approaches the requested temperature. Above the request the ramp
                // keeps descending to the profile floor (anti-overshoot); in the
                // approach band it descends toward the user's dial value, which is
                // the level the burn settles at. The HX-soot increase stays disabled
                // here (the user owns the fine tuning); the HX-too-hot decrease and
                // the HX-critical stop still apply below.
                float approach_temp = target_temp - approach_threshold_;
                if (room_temp < approach_temp) {
                    // still well below the request: hold full power
                    new_freq = profile_.pump_freq_max;
                } else if (room_temp < target_temp) {
                    // approaching the request: step down toward the user's level
                    float step = profile_.pump_freq_step;
                    new_freq = std::max(pump_freq_user_request_, pump_freq_setting_ - step);
                } else {
                    // at/above the request: keep stepping down, all the way to the floor
                    float overshoot = room_temp - target_temp;
                    float multiplier = (overshoot >= 1.0f) ? 3.0f : (overshoot >= 0.5f) ? 2.0f : 1.0f;
                    float step = profile_.pump_freq_step * multiplier;
                    new_freq = std::max(profile_.pump_freq_min, pump_freq_setting_ - step);
                }
            } else {
            // HEAT mode: Continuous heat with approach control, no auto-shutdown
            // Ramp pump to minimum when approaching target, but keep heater running

            float approach_temp = target_temp - approach_threshold_;

            if (room_temp < approach_temp) {
                // HEATING PHASE: Room is cold, increase pump as needed
                float temp_gap = target_temp - room_temp;
                float multiplier = (temp_gap >= 5.0f) ? 3.0f : (temp_gap >= 2.0f) ? 2.0f : 1.0f;
                float step = profile_.pump_freq_step * multiplier;
                new_freq = pump_freq_setting_ + step;
                if (new_freq <= profile_.pump_freq_max && new_freq != pump_freq_setting_) {
                    ESP_LOGI(TAG, "HEAT mode: room %.1f°C < %.1f°C, pump %.1f -> %.1f Hz (x%.0f)",
                             room_temp, approach_temp, pump_freq_setting_, new_freq, multiplier);
                }
            } else {
                // APPROACH/AT TARGET: Ramp down to minimum pump, keep running
                // This maintains heat without overshooting too much
                if (pump_freq_setting_ > profile_.pump_freq_min) {
                    float overshoot = room_temp - target_temp;
                    float multiplier = (overshoot >= 1.0f) ? 3.0f : (overshoot >= 0.5f) ? 2.0f : 1.0f;
                    float step = profile_.pump_freq_step * multiplier;
                    new_freq = pump_freq_setting_ - step;
                    if (new_freq < profile_.pump_freq_min) {
                        new_freq = profile_.pump_freq_min;
                    }
                    if (new_freq != pump_freq_setting_) {
                        ESP_LOGI(TAG, "HEAT mode approach: room %.1f°C >= %.1f°C, pump %.1f -> %.1f Hz (min %.1f)",
                                 room_temp, approach_temp, pump_freq_setting_, new_freq, profile_.pump_freq_min);
                    }
                }
            }
            }
        }

        // === SAFETY: Heat exchanger limits override thermostat ===
        // Combustion safety takes priority over comfort
        // In manual-HEAT the soot-increase is skipped: the user owns the level,
        // and this override is exactly what used to fight a low-power setpoint.
        // The HX-too-hot decrease below stays active - that is protection.
        bool manual_heat = profile_.heat_mode_manual && heater_mode_ == HeaterMode::ON;
        if (!manual_heat && hx_temp < profile_.hx_temp_low && new_freq < pump_freq_setting_) {
            // HX too cold but thermostat wants to reduce - override, force increase
            new_freq = pump_freq_setting_ + profile_.pump_freq_step;
            ESP_LOGW(TAG, "HX soot safety override: HX %.0f°C < %.0f°C, forcing pump increase to %.1f Hz",
                     hx_temp, profile_.hx_temp_low, new_freq);
        } else if (hx_temp >= profile_.hx_temp_high && new_freq > pump_freq_setting_) {
            // HX too hot but thermostat wants to increase - override, force decrease
            new_freq = pump_freq_setting_ - profile_.pump_freq_step;
            ESP_LOGW(TAG, "HX safety override: HX %.0f°C >= %.0f°C, forcing pump decrease to %.1f Hz",
                     hx_temp, profile_.hx_temp_high, new_freq);
        }

        // Apply new pump frequency within limits
        new_freq = std::max(profile_.pump_freq_min, std::min(profile_.pump_freq_max, new_freq));
        if (new_freq != pump_freq_setting_) {
            pump_freq_setting_ = new_freq;
            last_pump_adjust_time_ = now;
        }

        // === AUTO-SHUTDOWN LOGIC (AUTO mode only) ===
        // Check if conditions are met for auto-shutdown:
        // - Mode is AUTO (not ON - ON mode bypasses thermostat)
        // - Pump at minimum (can't reduce further)
        // - Room temperature exceeds target + overshoot threshold
        if (heater_mode_ == HeaterMode::AUTO) {
            const float PUMP_EPSILON = 0.05f;  // Tolerance for pump frequency comparison
            bool pump_at_min = (pump_freq_setting_ <= profile_.pump_freq_min + PUMP_EPSILON);
            bool temp_overshoot = (room_temp > target_temp + auto_shutdown_overshoot_);

            if (pump_at_min && temp_overshoot && !in_auto_shutdown_) {
                ESP_LOGI(TAG, "Auto-shutdown: pump at min (%.1f Hz), room %.1f°C > target %.1f°C + %.1f°C overshoot",
                         pump_freq_setting_, room_temp, target_temp, auto_shutdown_overshoot_);
                pending_on_off_command_ = CMD_STOP;
                heater_on_request_ = false;
                in_cooldown_ = true;
                in_auto_shutdown_ = true;
            }
        }
    }

    // === AUTO-RESTART LOGIC (AUTO mode only) ===
    // Check if heater should auto-restart from auto-shutdown state
    // This runs when heater is off and was auto-shutdown (not manual off)
    if (!on_off_value_ && in_auto_shutdown_ && heater_mode_ == HeaterMode::AUTO && standalone_mode_) {
        float restart_threshold = target_temp - auto_shutdown_hysteresis_;

        if (room_temp < restart_threshold) {
            ESP_LOGI(TAG, "Auto-restart: room %.1f°C < target %.1f°C - %.1f°C hysteresis",
                     room_temp, target_temp, auto_shutdown_hysteresis_);
            pending_on_off_command_ = CMD_START;
            heater_on_request_ = true;
            pump_freq_setting_ = profile_.pump_freq_initial;
            last_pump_adjust_time_ = millis();
            in_auto_shutdown_ = false;
        }
    }
}

// ==================== VEVOR (AA66/AA77, 4800 baud) PROTOCOL ====================

// Air-density ratio sigma(h) = rho/rho0 (International Standard Atmosphere).
// Fuel flow must scale by this factor at altitude or combustion runs rich
// (soot, HX fouling). e.g. 762 m -> ~0.929 (-7.1 %).
static float air_density_ratio(float altitude_m) {
    return std::pow(1.0f - 0.0065f * altitude_m / 288.15f, 4.2561f);
}

void HeaterUart::vevor_loop() {
    uint32_t now = millis();

    // === AMBIENT HEAT LIMIT CHECK (HEAT Mode ONLY) ===
    if (heater_mode_ == HeaterMode::ON && current_temperature_value_ > ambient_heat_limit_) {
        ESP_LOGW(TAG, "Ambient Limit Exceeded in HEAT mode: Room %.1f°C > Limit %.1f°C. Forcing shutdown.",
                 current_temperature_value_, ambient_heat_limit_);
        pending_on_off_command_ = CMD_STOP;
        in_cooldown_ = true;  // Start cooldown sequence immediately
    }

    // Continuously collect frames. Resync on 0xAA; ignore our own echo (id 0x66).
    while (available()) {
        uint8_t byte = read();
        vevor_rx_last_byte_ = now;
        if (vevor_rx_index_ == 0) {
            if (byte == 0xAA) {
                vevor_rx_[vevor_rx_index_++] = byte;
            }
            // else: discard byte, keep hunting for the start marker
        } else {
            vevor_rx_[vevor_rx_index_++] = byte;
            if (vevor_rx_index_ >= 4) {
                // Frame length = length-field byte + 5 (observed: 0x0B->16 controller,
                // 0x33->56 and 0x34->57 heater frames)
                uint8_t expected = vevor_rx_[3] + 5;
                if (expected < 16 || expected > 60) {
                    // Garbage length field - resync instead of overflowing the buffer
                    ESP_LOGW(TAG, "VEVOR: implausible length field 0x%02X, resyncing", vevor_rx_[3]);
                    vevor_rx_index_ = 0;
                    continue;
                }
                if (vevor_rx_index_ >= expected) {
                    uint8_t rx[64];
                    memcpy(rx, vevor_rx_, expected);
                    vevor_rx_index_ = 0;
                    if (rx[1] == 0x66) {
                        ESP_LOGD(TAG, "VEVOR: own echo ignored (controller id 0x66)");
                        continue;
                    }
                    parse_vevor_rx_frame(rx, expected);
                } else if (vevor_rx_index_ > expected + 8) {
                    ESP_LOGW(TAG, "VEVOR: frame too long (%d bytes), resyncing", vevor_rx_index_);
                    vevor_rx_index_ = 0;
                }
            }
        }
    }

    // Partial frame timeout - resync
    if (vevor_rx_index_ > 0 && (now - vevor_rx_last_byte_) > 300) {
        ESP_LOGW(TAG, "VEVOR: partial frame timeout (%d bytes), resyncing", vevor_rx_index_);
        vevor_rx_index_ = 0;
    }

    // Priming timeout (parity with BDAP; priming itself is unsupported on Vevor)
    if (is_priming_ && (now - priming_start_time_ >= 60000)) {
        ESP_LOGI(TAG, "Fuel priming timeout reached (60 seconds) - stopping priming");
        stop_priming();
    }

    // 1 Hz controller cadence - controller always talks first, heater answers
    if (now - last_tx_time_ >= STANDALONE_TX_INTERVAL_MS) {
        send_vevor_frame();
        last_tx_time_ = now;
    }
}

void HeaterUart::send_vevor_frame() {
    uint8_t frame[16];

    // Refresh room temperature from the external sensor (physics guard:
    // rejects NaN and implausible DHT22 glitch samples, holds last trusted
    // value). Same as BDAP TX path.
    refresh_room_temperature();

    // Command / requested-state selection (mirrors the zatakon state machine)
    uint8_t command;
    uint8_t requested_state;
    if (pending_on_off_command_ == CMD_START) {
        command = 0x06;           // start
        requested_state = 0x06;
        pending_on_off_command_ = CMD_NO_CHANGE;
        ESP_LOGI(TAG, "VEVOR: sending START (power %d)", static_cast<int>(
                    output_domain::level_from_normalized(
                        output_domain::normalized_from_hz(pump_freq_setting_, profile_))));
    } else if (pending_on_off_command_ == CMD_STOP) {
        command = 0x06;           // stop
        requested_state = 0x05;
        pending_on_off_command_ = CMD_NO_CHANGE;
        ESP_LOGI(TAG, "VEVOR: sending STOP");
    } else if (heater_on_request_) {
        if (run_state_value_ == RUN_STATE_OFF) {
            command = 0x06;       // keep asserting start until the heater comes up
            requested_state = 0x06;
        } else {
            command = 0x02;       // running status request
            requested_state = 0x08;
        }
    } else if (run_state_value_ != RUN_STATE_OFF && run_state_value_ != RUN_STATE_COOLDOWN) {
        // Heater has not yet entered OFF/cooldown: keep asserting STOP every second.
        // (Live finding: the Vevor ignores a one-shot 0x06/0x05 and only stops on a
        // persistent request until raw state becomes 0 (off) or 4 (stopping/cooling).)
        command = 0x06;
        requested_state = 0x05;
    } else {
        command = 0x02;           // idle status request
        requested_state = 0x02;
    }

    frame[0] = 0xAA;              // start marker
    frame[1] = 0x66;              // controller id
    frame[2] = command;
    frame[3] = 0x0B;              // controller frame length field
    frame[4] = 0x00;
    frame[5] = 0x00;
    frame[6] = 0x00;
    frame[7] = 0x00;

    // Power level 1-10, derived from the internal pump-frequency setting (Hz domain).
    // The shared thermostat logic adjusts pump_freq_setting_; we convert at the
    // boundary through the single shared mapping (output_domain, host-tested).
    float output = output_domain::normalized_from_hz(pump_freq_setting_, profile_);

    // Altitude / air-density derating (Vevor frames have no altitude field - unlike
    // BDAP where the heater self-derates - so the host must cut fuel itself).
    // At altitude there is less air per stroke: uncorrected fuel runs rich (soot,
    // HX fouling, overheating). Derate the level by the ISA density ratio sigma(h),
    // applied only at stable combustion so ignition keeps its full requested rate.
    if (run_state_value_ == RUN_STATE_RUNNING) {
        output = output_domain::clamp01(output * air_density_ratio(altitude_));
    }
    frame[8] = output_domain::level_from_normalized(output);

    frame[9] = requested_state;

    // Experimental fan-speed probe. The documented Vevor 2kW protocol has NO
    // fan command: the frame carries only power level (byte 8) and requested
    // state (byte 9), and every live capture shows bytes 10-14 as 0x00.
    // When fan_probe_rpm_ > 0 we write it as a BE uint16 into bytes 10-11,
    // mirroring the RX fan encoding (uint16 RPM at RX 28-29), to test whether
    // the ECM accepts a fan target (e.g. to push the pump below the 1.6 Hz
    // level-1 floor). 0 = disabled: protocol-default bytes.
    //
    // NOTE: frame[] is a local stack array - bytes 10-11 MUST be zeroed
    // explicitly here when the probe is disabled, otherwise they carry
    // uninitialised stack garbage onto the bus (observed 0xFB 0x3F).
    frame[10] = 0x00;
    frame[11] = 0x00;
    if (fan_probe_rpm_ > 0) {
        frame[10] = (fan_probe_rpm_ >> 8) & 0xFF;
        frame[11] = fan_probe_rpm_ & 0xFF;
    }

    frame[12] = 0x00;
    frame[13] = 0x00;
    frame[14] = 0x00;
    frame[15] = calc_checksum(frame, 16);   // additive checksum, last byte

    ESP_LOGD(TAG, "VEVOR TX: %02X %02X %02X %02X %02X %02X %02X %02X | %02X %02X %02X %02X %02X %02X %02X %02X",
             frame[0], frame[1], frame[2], frame[3], frame[4], frame[5], frame[6], frame[7],
             frame[8], frame[9], frame[10], frame[11], frame[12], frame[13], frame[14], frame[15]);
    write_array(frame, 16);
    flush();
}

void HeaterUart::parse_vevor_rx_frame(const uint8_t *frame, size_t length) {
    if (length < 52 || length > 60) {
        ESP_LOGW(TAG, "VEVOR: unexpected frame length %d (expected 52-60)", (int) length);
        return;
    }

    // Additive checksum over bytes 2..len-2. Log mismatch but keep parsing - the bus
    // is noisy and the reference implementations accept bad checksums too.
    uint8_t received_csum = frame[length - 1];
    uint8_t calculated_csum = calc_checksum(frame, length);
    if (received_csum != calculated_csum) {
        ESP_LOGD(TAG, "VEVOR: checksum mismatch: rx 0x%02X, calc 0x%02X", received_csum, calculated_csum);
    }

    // Map Vevor state (byte 5) onto the shared BDAP run-state enumeration
    uint8_t vs = frame[5];
    switch (vs) {
        case 0:  run_state_value_ = RUN_STATE_OFF; break;
        case 1:  run_state_value_ = RUN_STATE_GLOW_PREHEAT; break;
        case 2:  run_state_value_ = RUN_STATE_IGNITED; break;
        case 3:  run_state_value_ = RUN_STATE_RUNNING; break;
        case 4:  run_state_value_ = RUN_STATE_COOLDOWN; break;
        default: run_state_value_ = RUN_STATE_OFF; break;
    }
    on_off_value_ = (vs != 0);

    supply_voltage_value_ = frame[11] * 0.1f;                                          // V x10

    // Byte 15: fan voltage (integer volts, 0-16). Identified from a full-cycle capture
    // (649 frames): 0 when the fan is stopped, 8-11 at running speeds, and the ceiling
    // tracks the unit's own supply rail (ours 11 of 13.2 V; the reference unit 16 of
    // its 15.5 V rail). Fan RPM LAGS it - it is the commanded drive, not a tach echo.
    fan_voltage_value_ = frame[15];

    // Bytes 24-27: glow-plug cluster. Non-zero ONLY while the glow is lit (ignition
    // glow and post-run glow), ramping up then zeroing - verified across two glow
    // episodes in the same capture.
    //   byte 27 -> glow current: range 4-12, matching the reference unit's glow-current
    //              byte (0-11 A during exactly these phases). CANDIDATE, not clamp-verified.
    //   bytes 24-26 -> ramp together into the 80-100 range; voltage vs temperature is
    //              NOT separated, so they are published RAW and unit-less.
    // NOTE: byte 13 is deliberately NOT decoded any more. On the XMZ-D2 it reads 0
    // through the whole glow-preheat phase (glow definitively on) and 78-128 while
    // running with the glow off, with only {0,78,88,98,108,118,128} ever observed -
    // so it is not glow current on this revision. See ISSUES.md #5.
    glow_plug_current_value_ = frame[27];
    glow_a_value_ = frame[24];
    glow_b_value_ = frame[25];
    glow_c_value_ = frame[26];

    // Byte 6: the level the ECM is actually running at. This is the feedback for
    // the HEAT start-at-max-then-ramp-down behaviour: the Power Level *dial* is the
    // user's request, this sensor is what the heater is doing.
    power_level_actual_value_ = frame[6];

    heat_exchanger_temp_value_ = static_cast<int16_t>((frame[16] << 8) | frame[17]) * 0.1f;  // int16, x10
    pump_frequency_value_ = frame[23] * 0.1f;                                          // Hz x10
    fan_speed_value_ = (frame[28] << 8) | frame[29];                                   // RPM
    error_code_value_ = frame[7];

    desired_temperature_value_ = desired_temp_setting_;
    run_state_description_ = run_state_map.count(run_state_value_)
                                ? run_state_map.at(run_state_value_)
                                : "Unknown Run State";
    error_code_description_ = vevor_error_map.count(error_code_value_)
                                ? vevor_error_map.at(error_code_value_)
                                : "Unknown Error Code";

    float hx_temp = heat_exchanger_temp_value_;
    float room_temp = current_temperature_value_;
    float target_temp = static_cast<float>(desired_temp_setting_);

    // byte 14: status flags. bit0 = cooling (per reference); this unit sends 0x80
    // even while idle (unknown meaning) - log the raw byte for future reverse
    // engineering rather than guessing.
    ESP_LOGD(TAG, "VEVOR RX: State=%d (%s) on=%d, power=%d, V=%.1f, HX=%.0f C, pump=%.1f Hz, fan=%d RPM, err=%d, status=0x%02X%s",
             run_state_value_, run_state_description_.c_str(), on_off_value_, frame[6],
             supply_voltage_value_, hx_temp, pump_frequency_value_, fan_speed_value_,
             error_code_value_, frame[14], (frame[14] & 0x01) ? " [COOLING]" : "");

    // Cooldown monitoring (HX-based, same behaviour as BDAP)
    if (in_cooldown_) {
        if (hx_temp <= profile_.cooldown_target_temp) {
            ESP_LOGI(TAG, "VEVOR: cooldown complete: HX %.0f C <= %.0f C", hx_temp, profile_.cooldown_target_temp);
            in_cooldown_ = false;
        }
        return;  // skip thermostat logic during cooldown
    }

    // Shared thermostat / safety / auto-restart logic (Hz domain; TX converts level back)
    thermostat_control();
}

uint8_t HeaterUart::calc_checksum(const uint8_t *data, size_t length) {
    uint32_t sum = 0;
    for (size_t i = 2; i < length - 1; ++i) {
        sum += data[i];
    }
    return static_cast<uint8_t>(sum & 0xFF);
}


void HeaterUart::start_priming() {
    // Only allow priming when heater is OFF
    if (on_off_value_ || heater_on_request_) {
        ESP_LOGW(TAG, "Cannot start priming - heater is currently running");
        return;
    }

    if (!standalone_mode_) {
        ESP_LOGW(TAG, "Fuel priming is only available in standalone mode");
        return;
    }

    if (profile_.protocol != HeaterProtocol::BDAP) {
        // No documented pump-prime command on the Vevor bus - refuse rather than guess
        ESP_LOGW(TAG, "Fuel priming is not supported on this heater's protocol");
        return;
    }

    if (is_priming_) {
        ESP_LOGI(TAG, "Priming already active - stopping priming");
        stop_priming();
        return;
    }

    ESP_LOGI(TAG, "Starting fuel priming - pump will run at %.1f Hz for 60 seconds", profile_.pump_prime_freq);
    is_priming_ = true;
    priming_start_time_ = millis();

    // Publish state immediately
    auto it = binary_sensors_.find("priming_active");
    if (it != binary_sensors_.end() && it->second != nullptr) {
        it->second->publish_state(true);
    }
}

void HeaterUart::stop_priming() {
    if (!is_priming_) {
        return;
    }

    ESP_LOGI(TAG, "Stopping fuel priming");
    is_priming_ = false;

    // Publish state immediately
    auto it = binary_sensors_.find("priming_active");
    if (it != binary_sensors_.end() && it->second != nullptr) {
        it->second->publish_state(false);
    }
}

}  // namespace heater_uart
}  // namespace esphome

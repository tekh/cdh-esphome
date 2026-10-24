#include "heater_uart.h"
#include "esphome/core/log.h"
#include <cmath>

#include "bdap_driver.h"
#include "heater_output.h"
#include "vevor_driver.h"

namespace esphome {
namespace heater_uart {

static const char *TAG = "heater_uart";

void HeaterUart::setup() {
    ESP_LOGCONFIG(TAG, "Setting up Heater UART...");

    // Re-apply the profile selected by `heater_model:` (fallback: 8kW defaults).
    // NOTE: ambient heat limit is NOT reset here - it comes from YAML only.
    this->profile_ = profile_for(this->profile_.model);

    // Select and bind the protocol driver for this heater (ROADMAP Phase 2):
    // the driver owns framing, cadence, echo discard, checksums, encode/decode.
    if (profile_.protocol == HeaterProtocol::VEVOR_UART) {
        driver_ = std::make_unique<VevorDriver>();
    } else {
        driver_ = std::make_unique<BdapDriver>();
    }
    driver_->begin(this, profile_, operating_voltage_, altitude_);

    // Start the pump at the profile's ignition rate
    pump_freq_setting_ = profile_.pump_freq_initial;

    ESP_LOGI(TAG, "Running in STANDALONE mode - ESP32 is the heater controller");
    ESP_LOGI(TAG, "Heater model: %s | pump %.1f-%.1f Hz | fan %u-%u RPM | HX critical %.0f C | temp %u-%u C",
             profile_.name, profile_.pump_freq_min, profile_.pump_freq_max,
             profile_.fan_rpm_min, profile_.fan_rpm_max, profile_.hx_temp_critical,
             profile_.temp_min, profile_.temp_max);
    ESP_LOGI(TAG, "Protocol: %s @ %u baud (inverted: %s)",
             driver_->name(), (unsigned) profile_.uart_baud_rate,
             profile_.uart_inverted ? "yes" : "no");
    ESP_LOGI(TAG, "Operating voltage: %s", operating_voltage_ == VOLTAGE_12V ? "12V" : "24V");
    ESP_LOGI(TAG, "Altitude: %d m", altitude_);
    ESP_LOGI(TAG, "Initial desired temperature: %.1f°C", desired_temp_setting_);
}

void HeaterUart::loop() {
    if (driver_ == nullptr) return;  // setup() has not run yet
    const uint32_t now = millis();

    // === AMBIENT HEAT LIMIT CHECK (HEAT Mode ONLY) ===
    // pulse_stop(): assert STOP without clearing the start intent - in HEAT
    // mode the check re-asserts every second while the room is hot, and the
    // heater restarts once the room cools below the limit (v1 semantics).
    if (heater_mode_ == HeaterMode::ON && current_temperature_value_ > ambient_heat_limit_) {
        ESP_LOGW(TAG, "Ambient Limit Exceeded in HEAT mode: Room %.1f°C > Limit %.1f°C. Forcing shutdown.",
                 current_temperature_value_, ambient_heat_limit_);
        driver_->pulse_stop();
        in_cooldown_ = true;  // Start cooldown sequence immediately
    }

    // Priming timeout (facade-level; duration: PRIMING_DURATION_MS)
    if (is_priming_ && (now - priming_start_time_ >= PRIMING_DURATION_MS)) {
        ESP_LOGI(TAG, "Fuel priming timeout reached (%u seconds) - stopping priming",
                 (unsigned) (PRIMING_DURATION_MS / 1000));
        stop_priming();
    }

    // Refresh the room-sensor reading (physics guard: rejects NaN and
    // implausible DHT22 glitch samples, holds the last trusted value).
    refresh_room_temperature();

    // Push the controller state to the driver, then give it the bus for this
    // tick (TX cadence + RX collection/parsing are the driver's business).
    ControlInputs inputs;
    inputs.output_fraction = output_domain::normalized_from_hz(pump_freq_setting_, profile_);
    inputs.desired_temp = desired_temp_setting_;
    inputs.room_temp = current_temperature_value_;
    inputs.cooldown_active = in_cooldown_;
    inputs.priming_active = is_priming_;
    driver_->update_inputs(inputs);

    if (driver_->loop(now)) {
        on_fresh_frame();
    }
}

void HeaterUart::on_fresh_frame() {
    const Telemetry &tlm = driver_->telemetry();

    // Mirror the parsed telemetry for update() publishing
    on_off_value_ = tlm.on;
    fan_speed_value_ = tlm.fan_speed;
    supply_voltage_value_ = tlm.supply_voltage;
    heat_exchanger_temp_value_ = tlm.hx_temp;
    glow_plug_voltage_value_ = tlm.glow_plug_voltage;
    glow_plug_current_value_ = tlm.glow_plug_current;
    glow_a_value_ = tlm.glow_a;
    glow_b_value_ = tlm.glow_b;
    glow_c_value_ = tlm.glow_c;
    pump_frequency_value_ = tlm.pump_hz;
    fan_voltage_value_ = tlm.fan_voltage;
    power_level_actual_value_ = tlm.power_level_actual;
    error_code_value_ = tlm.error_code;
    run_state_description_ = driver_->run_state_description();
    error_code_description_ = driver_->error_description();

    // Cooldown monitoring: check if heat exchanger has cooled enough
    if (in_cooldown_) {
        if (tlm.hx_temp <= profile_.cooldown_target_temp) {
            ESP_LOGI(TAG, "Cooldown complete: HX temp %.0f°C <= %.0f°C target. Fan stopping.",
                     tlm.hx_temp, profile_.cooldown_target_temp);
            in_cooldown_ = false;
        } else {
            ESP_LOGD(TAG, "Cooldown in progress: HX temp %.0f°C (target: %.0f°C), Fan at %d RPM",
                     tlm.hx_temp, profile_.cooldown_target_temp, profile_.cooldown_fan_rpm);
        }
        return;  // Skip thermostat logic during cooldown
    }

    // Standby check, thermostat, safety limits and auto-restart
    thermostat_control();
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
            sensor->publish_state(desired_temp_setting_);
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
        temperature_number_->publish_state(desired_temp_setting_);
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

// === ByteStream: the driver's view of the bus ===
// HeaterUart is a uart::UARTDevice; the overrides forward to the real UART.
int HeaterUart::available() { return uart::UARTDevice::available(); }
int HeaterUart::read() { return uart::UARTDevice::read(); }
void HeaterUart::write_array(const uint8_t *data, size_t length) {
    uart::UARTDevice::write_array(data, length);
}
void HeaterUart::flush() { uart::UARTDevice::flush(); }

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
    ESP_LOGI(TAG, "Queuing START command");
    driver_->request_start();
    // Set pump to initial ignition frequency - will ramp up after flame stabilizes
    pump_freq_setting_ = profile_.pump_freq_initial;
    last_pump_adjust_time_ = millis();  // Reset adjustment timer
    ESP_LOGI(TAG, "Pump frequency set to %.1f Hz for ignition", pump_freq_setting_);
}

void HeaterUart::turn_off() {
    ESP_LOGI(TAG, "Queuing STOP command - entering cooldown");
    driver_->request_stop();
    in_cooldown_ = true;  // Start cooldown sequence
    in_auto_shutdown_ = false;  // Manual turn off clears auto-shutdown state
}

void HeaterUart::set_desired_temperature(float temperature) {
    // Round to nearest 0.5 degree
    float rounded = std::round(temperature * 2.0f) / 2.0f;

    if (rounded < profile_.temp_min || rounded > profile_.temp_max) {
        ESP_LOGW(TAG, "Temperature %.1f out of range (%d-%d)", rounded, profile_.temp_min, profile_.temp_max);
        return;
    }

    ESP_LOGI(TAG, "Setting desired temperature to %.1f°C", rounded);
    desired_temp_setting_ = rounded;
    // Temperature will be sent in next frame automatically
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
    // Frequency will be sent in next frame automatically
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

void HeaterUart::set_fan_probe_rpm(uint16_t rpm) {
    if (driver_ != nullptr) driver_->set_fan_probe_rpm(rpm);
}

uint16_t HeaterUart::get_fan_probe_rpm() const {
    return driver_ != nullptr ? driver_->fan_probe_rpm() : 0;
}

void HeaterUart::set_heater_mode(HeaterMode mode) {
    heater_mode_ = mode;

    const char *mode_str = (mode == HeaterMode::OFF) ? "Off" :
                           (mode == HeaterMode::AUTO) ? "Auto" : "On";
    ESP_LOGI(TAG, "Heater mode set to: %s", mode_str);

    if (mode == HeaterMode::OFF) {
        // Force off - stop heater if running
        if (driver_->telemetry().on || driver_->on_requested()) {
            ESP_LOGI(TAG, "Mode Off: stopping heater");
            driver_->request_stop();
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
            if (driver_->telemetry().on || driver_->on_requested()) {
                driver_->request_stop();
                in_cooldown_ = true;
            }
        } else if (room_temp < threshold) {
            // Cold enough - start heater
            ESP_LOGI(TAG, "Mode Auto: room %.1f°C < %.1f°C threshold, starting heater",
                     room_temp, threshold);
            driver_->request_start();
            pump_freq_setting_ = profile_.pump_freq_initial;
            last_pump_adjust_time_ = millis();
            in_standby_ = false;
        } else {
            // Too warm - enter standby, wait for temp drop
            ESP_LOGI(TAG, "Mode Auto: room %.1f°C >= %.1f°C threshold, entering standby",
                     room_temp, threshold);
            in_standby_ = true;
            // If heater is running, stop it
            if (driver_->telemetry().on || driver_->on_requested()) {
                driver_->request_stop();
                in_cooldown_ = true;
            }
        }
        in_auto_shutdown_ = false;

    } else if (mode == HeaterMode::ON) {
        // Force on - start heater regardless of temperature
        ESP_LOGI(TAG, "Mode On: forcing heater start");
        driver_->request_start();
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

// ==================== ROOM SENSOR (physics guard) ====================

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
    if (external_temp_sensor_ == nullptr) return false;
    // Staleness is defined on ACCEPTED readings (refresh_room_temperature):
    // NaN and implausible samples do not refresh the timer, so a sensor that
    // is dead or stuck emitting garbage is blind after the timeout.
    if (last_valid_room_temp_time_ == 0) return true;  // no accepted reading since boot
    return (millis() - last_valid_room_temp_time_) > ROOM_SENSOR_TIMEOUT_MS;
}

// ==================== THERMOSTAT / SAFETY POLICY ====================
// (Phase 3 of ROADMAP.md extracts this into a pure ControlPolicy class; it
// stays in the facade for now - behaviour must not change while the driver
// seam lands.)

void HeaterUart::thermostat_control() {
    const Telemetry &tlm = driver_->telemetry();
    float hx_temp = tlm.hx_temp;
    float room_temp = current_temperature_value_;  // external sensor, refreshed at TX time
    float target_temp = static_cast<float>(desired_temp_setting_);

    // === ROOM-SENSOR HEALTH GATE ===
    // While the external room sensor keeps producing ACCEPTED readings, the
    // timestamp is refreshed in refresh_room_temperature() (loop path, 1 Hz).
    // NaN and physically implausible samples do NOT refresh it, so a DHT22
    // that is dead or stuck emitting garbage is blind after 10 min. Then the
    // last known value could hide an overheat (the 26C ambient limit reads
    // the same stale value) and a stale cold value could trigger a blind
    // start. Stop a running heater and block all auto-start/stop logic until
    // fresh data arrives. (User requirement, 2026-10-20: DHT22 in warning
    // state > 10 min -> heater off; 2026-10-21: also don't act on glitch
    // samples.)
    if (external_temp_sensor_ != nullptr && is_room_sensor_stale()) {
        if (tlm.on || driver_->on_requested()) {
            ESP_LOGW(TAG, "Room sensor stale > %.0f min (last valid %.1fC): stopping heater",
                     ROOM_SENSOR_TIMEOUT_MS / 60000.0f, current_temperature_value_);
            driver_->request_stop();
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
    if (in_standby_ && heater_mode_ == HeaterMode::AUTO) {
        float threshold = target_temp - auto_shutdown_hysteresis_;
        if (room_temp < threshold) {
            ESP_LOGI(TAG, "Standby->Start: room %.1f°C < %.1f°C threshold", room_temp, threshold);
            driver_->request_start();
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
    if (tlm.on) {
        uint32_t now = millis();

        if (tlm.run_state == RUN_STATE_IGNITED) {
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

        if (tlm.run_state != RUN_STATE_RUNNING) {
            // Heater is in early startup (1-3) or shutdown (6-8) - let it handle pump/fan
            ESP_LOGD(TAG, "Ignition/shutdown in progress (state %d) - thermostat paused", tlm.run_state);
            return;
        }

        // === SAFETY: Emergency shutdown if heat exchanger is critically hot ===
        if (hx_temp > profile_.hx_temp_critical) {
            ESP_LOGE(TAG, "CRITICAL: Heat exchanger %.0f°C > %.0f°C limit! Emergency shutdown!",
                     hx_temp, profile_.hx_temp_critical);
            driver_->request_stop();
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
                driver_->request_stop();
                in_cooldown_ = true;
                in_auto_shutdown_ = true;
            }
        }
    }

    // === AUTO-RESTART LOGIC (AUTO mode only) ===
    // Check if heater should auto-restart from auto-shutdown state
    // This runs when heater is off and was auto-shutdown (not manual off)
    if (!tlm.on && in_auto_shutdown_ && heater_mode_ == HeaterMode::AUTO) {
        float restart_threshold = target_temp - auto_shutdown_hysteresis_;

        if (room_temp < restart_threshold) {
            ESP_LOGI(TAG, "Auto-restart: room %.1f°C < target %.1f°C - %.1f°C hysteresis",
                     room_temp, target_temp, auto_shutdown_hysteresis_);
            driver_->request_start();
            pump_freq_setting_ = profile_.pump_freq_initial;
            last_pump_adjust_time_ = millis();
            in_auto_shutdown_ = false;
        }
    }
}

// ==================== FUEL PRIMING ====================

void HeaterUart::start_priming() {
    // Only allow priming when heater is OFF
    if (driver_->telemetry().on || driver_->on_requested()) {
        ESP_LOGW(TAG, "Cannot start priming - heater is currently running");
        return;
    }

    if (!driver_->supports_priming()) {
        // No documented pump-prime command on the Vevor bus - refuse rather than guess
        ESP_LOGW(TAG, "Fuel priming is not supported on this heater's protocol");
        return;
    }

    if (is_priming_) {
        ESP_LOGI(TAG, "Priming already active - stopping priming");
        stop_priming();
        return;
    }

    ESP_LOGI(TAG, "Starting fuel priming - pump will run at %.1f Hz for %u seconds",
             profile_.pump_prime_freq, (unsigned) (PRIMING_DURATION_MS / 1000));
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

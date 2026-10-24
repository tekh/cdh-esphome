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

    // Bind the policies (ROADMAP Phase 3). The combustion-oversight gate:
    // OFF is honoured only on ECM-owned-safety profiles (owner decision #3).
    bool oversight = firmware_safety_oversight_;
    if (!oversight && !profile_.ecm_owned_safety) {
        ESP_LOGW(TAG, "firmware_safety_oversight: false ignored on %s - for this heater "
                 "the firmware IS the combustion-safety authority (no ECM oversight to lean on)",
                 profile_.name);
        oversight = true;
    }
    thermostat_.begin(profile_);
    safety_.begin(profile_, oversight);

    // Start the pump at the profile's ignition rate
    thermostat_.seat_request(profile_.pump_freq_initial);

    ESP_LOGI(TAG, "Running in STANDALONE mode - ESP32 is the heater controller");
    ESP_LOGI(TAG, "Heater model: %s | pump %.1f-%.1f Hz | fan %u-%u RPM | HX critical %.0f C | temp %u-%u C",
             profile_.name, profile_.pump_freq_min, profile_.pump_freq_max,
             profile_.fan_rpm_min, profile_.fan_rpm_max, profile_.hx_temp_critical,
             profile_.temp_min, profile_.temp_max);
    ESP_LOGI(TAG, "Protocol: %s @ %u baud (inverted: %s)",
             driver_->name(), (unsigned) profile_.uart_baud_rate,
             profile_.uart_inverted ? "yes" : "no");
    ESP_LOGI(TAG, "Combustion oversight: %s", oversight ? "firmware (default)" : "ECM (YAML opt-out)");
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
    if (safety_.ambient_limit_exceeded(heater_mode_, room_sensor_.value(), ambient_heat_limit_)) {
        ESP_LOGW(TAG, "Ambient Limit Exceeded in HEAT mode: Room %.1f°C > Limit %.1f°C. Forcing shutdown.",
                 room_sensor_.value(), ambient_heat_limit_);
        driver_->pulse_stop();
        safety_.begin_cooldown();
    }

    // Priming timeout (facade-level; duration: PRIMING_DURATION_MS)
    if (is_priming_ && (now - priming_start_time_ >= PRIMING_DURATION_MS)) {
        ESP_LOGI(TAG, "Fuel priming timeout reached (%u seconds) - stopping priming",
                 (unsigned) (PRIMING_DURATION_MS / 1000));
        stop_priming();
    }

    // Refresh the room-sensor reading (physics guard: rejects NaN and
    // implausible DHT22 glitch samples, holds the last trusted value).
    if (external_temp_sensor_ != nullptr) {
        room_sensor_.refresh(external_temp_sensor_->state, now);
    }

    // Push the controller state to the driver, then give it the bus for this
    // tick (TX cadence + RX collection/parsing are the driver's business).
    Thermostat::Params params;
    params.approach_threshold = approach_threshold_;
    params.auto_shutdown_overshoot = auto_shutdown_overshoot_;
    params.auto_shutdown_hysteresis = auto_shutdown_hysteresis_;
    params.heat_mode_manual = profile_.heat_mode_manual;
    params.oversight_enabled = safety_.oversight();
    params.external_sensor_present = external_temp_sensor_ != nullptr;
    params.sensor_stale = room_sensor_.stale(now);

    ControlInputs inputs;
    inputs.output_fraction = output_domain::normalized_from_hz(thermostat_.pump_request(), profile_);
    inputs.fan_rpm = safety_.fan_rpm(inputs.output_fraction, driver_->telemetry());
    inputs.desired_temp = desired_temp_setting_;
    inputs.room_temp = room_sensor_.value();
    inputs.priming_active = is_priming_;
    driver_->update_inputs(inputs);

    if (driver_->loop(now)) {
        on_fresh_frame(params);
    }
}

void HeaterUart::on_fresh_frame(const Thermostat::Params &params) {
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
    if (safety_.in_cooldown()) {
        if (safety_.cooldown_complete(tlm.hx_temp)) {
            ESP_LOGI(TAG, "Cooldown complete: HX temp %.0f°C <= %.0f°C target. Fan stopping.",
                     tlm.hx_temp, profile_.cooldown_target_temp);
            safety_.end_cooldown();
        } else {
            ESP_LOGD(TAG, "Cooldown in progress: HX temp %.0f°C (target: %.0f°C), Fan at %d RPM",
                     tlm.hx_temp, profile_.cooldown_target_temp, profile_.cooldown_fan_rpm);
        }
        return;  // Skip thermostat logic during cooldown
    }

    // Room-sensor staleness is computed by the guard; the thermostat implements
    // the health gate (stop + block) and the standby/thermostat/safety machine.
    const bool stale = room_sensor_.stale(millis());
    const Thermostat::Result r =
        thermostat_.control(tlm, room_sensor_.value(), desired_temp_setting_, heater_mode_, params, millis());

    // Apply the thermostat's requests in v1 order (stop first, then start -
    // a later start wins in the single pending-command slot, as in v1).
    if (r.request_stop) driver_->request_stop();
    if (r.request_start) driver_->request_start();
    if (r.pulse_stop) driver_->pulse_stop();
    if (r.enter_cooldown) safety_.begin_cooldown();
}

void HeaterUart::update() {
    for (const auto &sensor_entry : sensors_) {
        const std::string &key = sensor_entry.first;
        sensor::Sensor *sensor = sensor_entry.second;

        if (key == "current_temperature")
            sensor->publish_state(room_sensor_.value());
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
            binary_sensor->publish_state(thermostat_.in_auto_shutdown());
        else if (key == "standby_active")
            binary_sensor->publish_state(thermostat_.in_standby());
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
            pump_number_->publish_state(thermostat_.user_request());
        } else {
            pump_number_->publish_state(thermostat_.pump_request());
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
    thermostat_.seat_request(profile_.pump_freq_initial);
    thermostat_.reset_adjust_timer(millis());  // Reset adjustment timer
    ESP_LOGI(TAG, "Pump frequency set to %.1f Hz for ignition", thermostat_.pump_request());
}

void HeaterUart::turn_off() {
    ESP_LOGI(TAG, "Queuing STOP command - entering cooldown");
    driver_->request_stop();
    safety_.begin_cooldown();  // Start cooldown sequence
    thermostat_.set_auto_shutdown(false);  // Manual turn off clears auto-shutdown state
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
    thermostat_.set_request(frequency);  // live request AND user setpoint (manual-HEAT authority)
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
                         ? thermostat_.user_request()
                         : thermostat_.pump_request();
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
            safety_.begin_cooldown();
        }
        thermostat_.set_standby(false);
        thermostat_.set_auto_shutdown(false);

    } else if (mode == HeaterMode::AUTO) {
        // Check if we should start or enter standby
        float room_temp = room_sensor_.value();
        float threshold = desired_temp_setting_ - auto_shutdown_hysteresis_;

        if (is_room_sensor_stale()) {
            // No blind starts: the room temperature is stale (sensor dead or
            // never read). Park in standby; the thermostat's standby check
            // starts the heater once fresh readings arrive. If the heater is
            // running, stop it (burning on blind data).
            ESP_LOGW(TAG, "Mode Auto: room sensor stale - no blind start, parking in standby");
            thermostat_.set_standby(true);
            if (driver_->telemetry().on || driver_->on_requested()) {
                driver_->request_stop();
                safety_.begin_cooldown();
            }
        } else if (room_temp < threshold) {
            // Cold enough - start heater
            ESP_LOGI(TAG, "Mode Auto: room %.1f°C < %.1f°C threshold, starting heater",
                     room_temp, threshold);
            driver_->request_start();
            thermostat_.seat_request(profile_.pump_freq_initial);
            thermostat_.reset_adjust_timer(millis());
            thermostat_.set_standby(false);
        } else {
            // Too warm - enter standby, wait for temp drop
            ESP_LOGI(TAG, "Mode Auto: room %.1f°C >= %.1f°C threshold, entering standby",
                     room_temp, threshold);
            thermostat_.set_standby(true);
            // If heater is running, stop it
            if (driver_->telemetry().on || driver_->on_requested()) {
                driver_->request_stop();
                safety_.begin_cooldown();
            }
        }
        thermostat_.set_auto_shutdown(false);

    } else if (mode == HeaterMode::ON) {
        // Force on - start heater regardless of temperature
        ESP_LOGI(TAG, "Mode On: forcing heater start");
        driver_->request_start();
        // Start level on a cold start. Manual-HEAT profiles start at FULL POWER and
        // ramp down as the room approaches the requested temperature
        // (thermostat control); everything else keeps the ignition-rate default.
        thermostat_.seat_request(profile_.heat_mode_manual ? profile_.pump_freq_max
                                                           : profile_.pump_freq_initial);
        ESP_LOGI(TAG, "Mode On: start level %.1f Hz (%.1f = full power)", thermostat_.pump_request(),
                 profile_.pump_freq_max);
        thermostat_.reset_adjust_timer(millis());
        thermostat_.set_standby(false);
        thermostat_.set_auto_shutdown(false);
    }
}

// ==================== ROOM SENSOR (facade wrapper) ====================

bool HeaterUart::refresh_room_temperature() {
    if (external_temp_sensor_ == nullptr) return false;
    return room_sensor_.refresh(external_temp_sensor_->state, millis());
}

bool HeaterUart::is_room_sensor_stale() const {
    if (external_temp_sensor_ == nullptr) return false;
    return room_sensor_.stale(millis());
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

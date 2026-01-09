#include "heater_climate.h"
#include "esphome/core/log.h"

namespace esphome {
namespace heater_uart {

static const char *TAG = "heater_uart.climate";

void HeaterClimate::setup() {
  if (this->parent_ != nullptr) {
    this->parent_->set_climate(this);
    ESP_LOGI(TAG, "Heater climate initialized");
  }
}

void HeaterClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "Heater Climate:");
  ESP_LOGCONFIG(TAG, "  Supported modes: OFF, HEAT, AUTO");
  ESP_LOGCONFIG(TAG, "  Temperature range: %d-%d°C", TEMP_MIN, TEMP_MAX);
  ESP_LOGCONFIG(TAG, "  Temperature step: 0.5°C");
}

climate::ClimateTraits HeaterClimate::traits() {
  auto traits = climate::ClimateTraits();

  // Supported modes
  traits.set_supported_modes({
    climate::CLIMATE_MODE_OFF,
    climate::CLIMATE_MODE_HEAT,
    climate::CLIMATE_MODE_AUTO
  });

  // Temperature range and step
  traits.set_visual_min_temperature(TEMP_MIN);
  traits.set_visual_max_temperature(TEMP_MAX);
  traits.set_visual_temperature_step(0.5f);

  // Feature flags - supports current temperature and action states
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_ACTION);

  return traits;
}

void HeaterClimate::control(const climate::ClimateCall &call) {
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "Parent HeaterUart not set!");
    return;
  }

  // Handle mode change
  if (call.get_mode().has_value()) {
    climate::ClimateMode climate_mode = *call.get_mode();
    HeaterMode heater_mode;

    switch (climate_mode) {
      case climate::CLIMATE_MODE_OFF:
        heater_mode = HeaterMode::OFF;
        ESP_LOGI(TAG, "Setting heater mode to OFF");
        break;
      case climate::CLIMATE_MODE_HEAT:
        heater_mode = HeaterMode::ON;
        ESP_LOGI(TAG, "Setting heater mode to ON (force on)");
        break;
      case climate::CLIMATE_MODE_AUTO:
        heater_mode = HeaterMode::AUTO;
        ESP_LOGI(TAG, "Setting heater mode to AUTO (thermostat control)");
        break;
      default:
        ESP_LOGW(TAG, "Unsupported climate mode requested");
        return;
    }

    this->parent_->set_heater_mode(heater_mode);
    this->mode = climate_mode;
  }

  // Handle target temperature change
  if (call.get_target_temperature().has_value()) {
    float target = *call.get_target_temperature();
    ESP_LOGI(TAG, "Setting target temperature to %.1f°C", target);
    this->parent_->set_desired_temperature(target);
    this->target_temperature = target;
  }

  // Publish updated state
  this->publish_state();
}

void HeaterClimate::loop() {
  if (this->parent_ == nullptr) {
    return;
  }

  bool state_changed = false;

  // Get current state from parent
  HeaterMode current_mode = this->parent_->get_heater_mode();
  bool on_off_state = this->parent_->get_on_off_state();
  bool auto_shutdown = this->parent_->is_in_auto_shutdown();
  bool standby = this->parent_->is_in_standby();
  float current_temp = this->parent_->get_current_temperature();
  int desired_temp = this->parent_->get_desired_temperature();

  // Update climate mode if heater mode changed
  if (current_mode != this->last_heater_mode_) {
    switch (current_mode) {
      case HeaterMode::OFF:
        this->mode = climate::CLIMATE_MODE_OFF;
        break;
      case HeaterMode::ON:
        this->mode = climate::CLIMATE_MODE_HEAT;
        break;
      case HeaterMode::AUTO:
        this->mode = climate::CLIMATE_MODE_AUTO;
        break;
    }
    this->last_heater_mode_ = current_mode;
    state_changed = true;
  }

  // Update climate action based on heater state
  climate::ClimateAction new_action;
  if (!on_off_state) {
    new_action = climate::CLIMATE_ACTION_OFF;
  } else if (standby || auto_shutdown) {
    new_action = climate::CLIMATE_ACTION_IDLE;
  } else {
    new_action = climate::CLIMATE_ACTION_HEATING;
  }

  if (this->action != new_action) {
    this->action = new_action;
    state_changed = true;
  }

  // Update current temperature
  if (!std::isnan(current_temp) && (std::isnan(this->last_current_temp_) ||
      std::abs(current_temp - this->last_current_temp_) > 0.1f)) {
    this->current_temperature = current_temp;
    this->last_current_temp_ = current_temp;
    state_changed = true;
  }

  // Update target temperature
  if (desired_temp != this->last_desired_temp_) {
    this->target_temperature = desired_temp;
    this->last_desired_temp_ = desired_temp;
    state_changed = true;
  }

  // Track other state changes
  if (on_off_state != this->last_on_off_state_) {
    this->last_on_off_state_ = on_off_state;
    state_changed = true;
  }

  if (auto_shutdown != this->last_auto_shutdown_) {
    this->last_auto_shutdown_ = auto_shutdown;
    state_changed = true;
  }

  if (standby != this->last_standby_) {
    this->last_standby_ = standby;
    state_changed = true;
  }

  // Only publish if something changed
  if (state_changed) {
    this->publish_state();
  }
}

}  // namespace heater_uart
}  // namespace esphome

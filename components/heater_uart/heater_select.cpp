#include "heater_select.h"
#include "esphome/core/log.h"

namespace esphome {
namespace heater_uart {

static const char *TAG = "heater_uart.select";

void HeaterModeSelect::setup() {
    if (this->parent_ != nullptr) {
        this->parent_->set_mode_select(this);
        // Publish initial state - default is Off
        this->publish_state("Off");
        ESP_LOGI(TAG, "Heater mode select initialized - default: Off");
    }
}

void HeaterModeSelect::dump_config() {
    ESP_LOGCONFIG(TAG, "Heater Mode Select:");
    ESP_LOGCONFIG(TAG, "  Options: Off, Auto, On");
}

void HeaterModeSelect::control(const std::string &value) {
    if (this->parent_ == nullptr) {
        ESP_LOGE(TAG, "Parent HeaterUart not set!");
        return;
    }

    HeaterMode mode;
    if (value == "Off") {
        mode = HeaterMode::OFF;
        ESP_LOGI(TAG, "Setting heater mode to Off");
    } else if (value == "Auto") {
        mode = HeaterMode::AUTO;
        ESP_LOGI(TAG, "Setting heater mode to Auto");
    } else if (value == "On") {
        mode = HeaterMode::ON;
        ESP_LOGI(TAG, "Setting heater mode to On");
    } else {
        ESP_LOGW(TAG, "Unknown mode: %s", value.c_str());
        return;
    }

    this->parent_->set_heater_mode(mode);
    this->publish_state(value);
}

}  // namespace heater_uart
}  // namespace esphome

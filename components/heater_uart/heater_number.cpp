#include "heater_number.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace heater_uart {

static const char *TAG = "heater_uart.number";

void HeaterTemperatureNumber::setup() {
    // Register this number with the parent for state sync
    if (this->parent_ != nullptr) {
        this->parent_->set_temperature_number(this);
        ESP_LOGI(TAG, "Registered temperature number with HeaterUart for state sync");
    }
}

void HeaterTemperatureNumber::dump_config() {
    LOG_NUMBER("", "Heater Temperature", this);
}

void HeaterTemperatureNumber::control(float value) {
    if (this->parent_ == nullptr) {
        ESP_LOGE(TAG, "Parent HeaterUart not set!");
        return;
    }

    // Round to nearest 0.5 degree and clamp
    float rounded = std::round(value * 2.0f) / 2.0f;
    if (rounded < TEMP_MIN) rounded = TEMP_MIN;
    if (rounded > TEMP_MAX) rounded = TEMP_MAX;

    ESP_LOGI(TAG, "Setting desired temperature to %.1f°C", rounded);
    this->parent_->set_desired_temperature(rounded);

    // Publish state immediately (optimistic)
    this->publish_state(rounded);
}

}  // namespace heater_uart
}  // namespace esphome

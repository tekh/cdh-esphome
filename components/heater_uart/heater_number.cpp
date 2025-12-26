#include "heater_number.h"
#include "esphome/core/log.h"

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

    // Clamp and round the value to an integer
    uint8_t temp = static_cast<uint8_t>(value);
    if (temp < TEMP_MIN) temp = TEMP_MIN;
    if (temp > TEMP_MAX) temp = TEMP_MAX;

    ESP_LOGI(TAG, "Setting desired temperature to %d°C", temp);
    this->parent_->set_desired_temperature(temp);

    // Publish state immediately (optimistic)
    this->publish_state(temp);
}

}  // namespace heater_uart
}  // namespace esphome

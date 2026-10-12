#include "heater_power_level_number.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace heater_uart {

static const char *TAG = "heater_uart.power_level_number";

void HeaterPowerLevelNumber::setup() {
    if (this->parent_ != nullptr) {
        this->parent_->set_power_level_number(this);
        ESP_LOGI(TAG, "Registered power level dial with HeaterUart (%s)",
                 this->parent_->get_profile().name);
        this->publish_state(this->parent_->get_power_level_display());
    }
}

void HeaterPowerLevelNumber::dump_config() {
    LOG_NUMBER("", "Heater Power Level", this);
}

void HeaterPowerLevelNumber::control(float value) {
    if (this->parent_ == nullptr) {
        ESP_LOGE(TAG, "Parent HeaterUart not set!");
        return;
    }

    if (!this->parent_->get_profile().has_power_level_control) {
        ESP_LOGW(TAG, "Power level is not commandable on %s (frequency-commanded hardware) - write ignored",
                 this->parent_->get_profile().name);
        this->publish_state(this->parent_->get_power_level_display());
        return;
    }

    int level = static_cast<int>(std::round(value));
    if (level < 1) level = 1;
    if (level > 10) level = 10;

    ESP_LOGI(TAG, "Power level dial -> %d", level);
    this->parent_->set_power_level(static_cast<uint8_t>(level));

    // Publish state immediately (optimistic)
    this->publish_state(static_cast<float>(level));
}

}  // namespace heater_uart
}  // namespace esphome

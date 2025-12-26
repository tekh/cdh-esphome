#include "heater_switch.h"
#include "esphome/core/log.h"

namespace esphome {
namespace heater_uart {

static const char *TAG = "heater_uart.switch";

void HeaterSwitch::setup() {
    // Register this switch with the parent for state sync
    if (this->parent_ != nullptr) {
        this->parent_->set_power_switch(this);
        ESP_LOGI(TAG, "Registered power switch with HeaterUart for state sync");
    }
}

void HeaterSwitch::dump_config() {
    ESP_LOGCONFIG(TAG, "Heater Switch:");
}

void HeaterSwitch::write_state(bool state) {
    if (this->parent_ == nullptr) {
        ESP_LOGE(TAG, "Parent HeaterUart not set!");
        return;
    }

    if (state) {
        ESP_LOGI(TAG, "Switch ON - sending START command");
        this->parent_->turn_on();
    } else {
        ESP_LOGI(TAG, "Switch OFF - sending STOP command");
        this->parent_->turn_off();
    }

    // Publish the state (optimistic - we assume command will work)
    this->publish_state(state);
}

}  // namespace heater_uart
}  // namespace esphome

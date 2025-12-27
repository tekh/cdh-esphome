#include "heater_auto_shutdown_switch.h"
#include "esphome/core/log.h"

namespace esphome {
namespace heater_uart {

static const char *TAG = "heater_uart.auto_shutdown_switch";

void HeaterAutoShutdownSwitch::setup() {
    // Register this switch with the parent for state sync
    if (this->parent_ != nullptr) {
        this->parent_->set_auto_shutdown_switch(this);
        ESP_LOGI(TAG, "Registered auto-shutdown switch with HeaterUart for state sync");

        // Publish initial state (enabled by default)
        this->publish_state(true);
    }
}

void HeaterAutoShutdownSwitch::dump_config() {
    ESP_LOGCONFIG(TAG, "Heater Auto-Shutdown Switch:");
}

void HeaterAutoShutdownSwitch::write_state(bool state) {
    if (this->parent_ == nullptr) {
        ESP_LOGE(TAG, "Parent HeaterUart not set!");
        return;
    }

    ESP_LOGI(TAG, "Auto-shutdown %s", state ? "enabled" : "disabled");
    this->parent_->set_auto_shutdown_enabled(state);

    // Publish state immediately
    this->publish_state(state);
}

}  // namespace heater_uart
}  // namespace esphome

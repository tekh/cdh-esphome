#include "heater_fan_probe_number.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace heater_uart {

static const char *TAG = "heater_uart.fan_probe_number";

void HeaterFanProbeNumber::setup() {
    if (this->parent_ != nullptr) {
        this->parent_->set_fan_probe_number(this);
        ESP_LOGI(TAG, "Registered fan speed probe with HeaterUart");
        // Publish initial state (0 = disabled)
        this->publish_state(this->parent_->get_fan_probe_rpm());
    }
}

void HeaterFanProbeNumber::dump_config() {
    LOG_NUMBER("", "Fan Speed Probe", this);
}

void HeaterFanProbeNumber::control(float value) {
    if (this->parent_ == nullptr) {
        ESP_LOGE(TAG, "Parent HeaterUart not set!");
        return;
    }

    int rpm = static_cast<int>(std::round(value));
    if (rpm < 0) rpm = 0;
    if (rpm > 65535) rpm = 65535;

    if (rpm > 0) {
        ESP_LOGW(TAG, "Fan probe ACTIVE at %d RPM - undocumented TX bytes 10-11 in use. "
              "Watch error_code and run_state; set back to 0 to restore protocol-default bytes.", rpm);
    } else {
        ESP_LOGI(TAG, "Fan probe disabled - TX bytes 10-11 back to 0x00");
    }
    this->parent_->set_fan_probe_rpm(static_cast<uint16_t>(rpm));

    // Publish state immediately (optimistic)
    this->publish_state(static_cast<float>(rpm));
}

}  // namespace heater_uart
}  // namespace esphome

#include "heater_pump_number.h"
#include "esphome/core/log.h"

namespace esphome {
namespace heater_uart {

static const char *TAG = "heater_uart.pump_number";

void HeaterPumpNumber::setup() {
    // Register this number with the parent for state sync
    if (this->parent_ != nullptr) {
        this->parent_->set_pump_number(this);
        ESP_LOGI(TAG, "Registered pump number with HeaterUart for state sync");

        // Publish initial state
        this->publish_state(this->parent_->get_pump_frequency_setting());
    }
}

void HeaterPumpNumber::dump_config() {
    LOG_NUMBER("", "Heater Pump Frequency", this);
}

void HeaterPumpNumber::control(float value) {
    if (this->parent_ == nullptr) {
        ESP_LOGE(TAG, "Parent HeaterUart not set!");
        return;
    }

    // One fuel dial per heater: level-commanded hardware (Vevor) has no pump-Hz
    // command at all - refuse the write rather than pretend.
    if (!this->parent_->get_profile().has_pump_hz_control) {
        ESP_LOGW(TAG, "Pump frequency is not commandable on %s (level-commanded hardware) - use the power level dial",
                 this->parent_->get_profile().name);
        this->publish_state(this->parent_->get_pump_frequency_setting());
        return;
    }

    // Clamp the value to the active profile's pump frequency range
    if (value < this->parent_->get_pump_freq_min()) value = this->parent_->get_pump_freq_min();
    if (value > this->parent_->get_pump_freq_max()) value = this->parent_->get_pump_freq_max();

    ESP_LOGI(TAG, "Setting pump frequency to %.1f Hz", value);
    this->parent_->set_pump_frequency(value);

    // Publish state immediately (optimistic)
    this->publish_state(value);
}

}  // namespace heater_uart
}  // namespace esphome

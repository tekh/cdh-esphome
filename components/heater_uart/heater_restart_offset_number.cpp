#include "heater_restart_offset_number.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace heater_uart {

static const char *TAG = "heater_uart.restart_offset";

void HeaterRestartOffsetNumber::setup() {
    if (this->parent_ == nullptr) {
        return;
    }
    this->parent_->set_restart_offset_number(this);

    const float value = this->parent_->get_auto_shutdown_hysteresis();
    ESP_LOGI(TAG, "Registered AUTO restart-offset dial (%.1f°C below the setpoint)", value);

    // The slider's own range is a UI choice (YAML min_value/max_value); the
    // YAML `auto_shutdown_hysteresis` key is the boot default. If the two
    // disagree, say so instead of silently publishing a value the dial cannot
    // represent (HA would clamp the handle and lie about the live offset).
    const float lo = this->traits.get_min_value();
    const float hi = this->traits.get_max_value();
    if (value < lo || value > hi) {
        ESP_LOGW(TAG,
                 "auto_shutdown_hysteresis %.2f°C is outside this dial's range %.1f-%.1f°C - widen min_value/max_value or the slider will misreport it",
                 value, lo, hi);
    }
    this->publish_state(value);
}

void HeaterRestartOffsetNumber::dump_config() {
    LOG_NUMBER("", "Heater AUTO Restart Offset", this);
}

void HeaterRestartOffsetNumber::control(float value) {
    if (this->parent_ == nullptr) {
        ESP_LOGE(TAG, "Parent HeaterUart not set!");
        return;
    }

    // 0.1°C resolution, clamped to the dial's declared range.
    float offset = std::round(value * 10.0f) / 10.0f;
    const float lo = this->traits.get_min_value();
    const float hi = this->traits.get_max_value();
    if (offset < lo) offset = lo;
    if (offset > hi) offset = hi;

    if (offset < 0.2f) {
        ESP_LOGW(TAG,
                 "Restart offset %.1f°C is very tight: AUTO restarts at room < target - %.1f and shuts down at room > target + overshoot, so the heater will short-cycle around the setpoint (ignition cycles cost glow-plug life and the Vevor ECM latches after repeated aborts). 0.2-1.0 is the useful range.",
                 offset, offset);
    }

    ESP_LOGI(TAG, "AUTO restart offset -> %.1f°C (restart below %.1f°C for a %.1f°C target)", offset,
             this->parent_->get_desired_temperature() - offset,
             this->parent_->get_desired_temperature());
    this->parent_->set_auto_shutdown_hysteresis(offset);

    // Publish state immediately (optimistic)
    this->publish_state(offset);
}

}  // namespace heater_uart
}  // namespace esphome

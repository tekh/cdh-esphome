#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "heater_uart.h"

namespace esphome {
namespace heater_uart {

class HeaterClimate : public climate::Climate, public Component {
 public:
  void set_parent(HeaterUart *parent) { this->parent_ = parent; }

  void setup() override;
  void dump_config() override;
  void loop() override;

 protected:
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

  HeaterUart *parent_{nullptr};

  // Track last known state to detect changes
  bool last_on_off_state_{false};
  float last_current_temp_{NAN};
  int last_desired_temp_{0};
  HeaterMode last_heater_mode_{HeaterMode::OFF};
  bool last_auto_shutdown_{false};
  bool last_standby_{false};
};

}  // namespace heater_uart
}  // namespace esphome

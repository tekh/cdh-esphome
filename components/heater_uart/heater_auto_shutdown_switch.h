#pragma once

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "heater_uart.h"

namespace esphome {
namespace heater_uart {

class HeaterAutoShutdownSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(HeaterUart *parent) { this->parent_ = parent; }

  void setup() override;
  void dump_config() override;

 protected:
  void write_state(bool state) override;
  HeaterUart *parent_{nullptr};
};

}  // namespace heater_uart
}  // namespace esphome

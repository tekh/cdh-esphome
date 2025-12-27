#pragma once

#include "esphome/core/component.h"
#include "esphome/components/select/select.h"
#include "heater_uart.h"

namespace esphome {
namespace heater_uart {

class HeaterModeSelect : public select::Select, public Component {
 public:
  void set_parent(HeaterUart *parent) { this->parent_ = parent; }

  void setup() override;
  void dump_config() override;

 protected:
  void control(const std::string &value) override;
  HeaterUart *parent_{nullptr};
};

}  // namespace heater_uart
}  // namespace esphome

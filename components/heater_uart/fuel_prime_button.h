#pragma once

#include "esphome/core/component.h"
#include "esphome/components/button/button.h"
#include "heater_uart.h"

namespace esphome {
namespace heater_uart {

class FuelPrimeButton : public button::Button, public Component {
 public:
  void set_parent(HeaterUart *parent) { this->parent_ = parent; }

 protected:
  void press_action() override {
    if (this->parent_ != nullptr) {
      this->parent_->start_priming();
    }
  }

  HeaterUart *parent_{nullptr};
};

}  // namespace heater_uart
}  // namespace esphome

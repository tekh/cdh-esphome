#pragma once

#include "esphome/core/component.h"
#include "esphome/components/number/number.h"
#include "heater_uart.h"

namespace esphome {
namespace heater_uart {

// Power-level dial (1-10) for heaters whose only fuel command is a power level.
//
// Vevor 2kW: this is THE fuel dial. There is no pump-frequency and no fan-RPM
// command in that protocol (fan probe on undocumented bytes was proven ignored),
// so on that hardware the honest control surface in HEAT mode is:
//   temperature setpoint + power level.
//
// BDAP 8kW: writes are refused (the profile's has_power_level_control is false
// there - that heater takes a real pump frequency), so the two fuel dials can
// never both be live on one device.
class HeaterPowerLevelNumber : public number::Number, public Component {
 public:
  void set_parent(HeaterUart *parent) { this->parent_ = parent; }

  void setup() override;
  void dump_config() override;

 protected:
  void control(float value) override;
  HeaterUart *parent_{nullptr};
};

}  // namespace heater_uart
}  // namespace esphome

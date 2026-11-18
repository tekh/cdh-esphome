#pragma once

#include "esphome/core/component.h"
#include "esphome/components/number/number.h"
#include "heater_uart.h"

namespace esphome {
namespace heater_uart {

// AUTO-mode restart offset (the "hysteresis"): how far BELOW the setpoint the
// room must fall before AUTO re-ignites after an auto-shutdown / while parked
// in standby. Runtime dial for `auto_shutdown_hysteresis` (the YAML key stays
// the boot default, exactly like the pump/power-level dials).
//
// Band arithmetic (all three live in Thermostat::Params):
//   shutdown  at room > target + auto_shutdown_overshoot   (overshoot 0.0)
//   restart   at room < target - auto_shutdown_hysteresis  (this entity)
// so the slider directly widens/narrows the dead band around the setpoint.
//
// 0.0 is accepted (the slider asks for it) but it collapses the band: AUTO
// would restart the instant the room drops under the setpoint and shut down
// again as soon as it crosses back, i.e. short-cycling at the target. control()
// logs a warning below 0.2 for that reason but does not override the operator.
class HeaterRestartOffsetNumber : public number::Number, public Component {
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

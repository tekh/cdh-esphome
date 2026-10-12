#pragma once

#include "esphome/core/component.h"
#include "esphome/components/number/number.h"
#include "heater_uart.h"

namespace esphome {
namespace heater_uart {

// Experimental fan-speed probe (Vevor 2kW).
//
// The documented Vevor 2kW protocol (zatakon RE) has NO fan command field:
// the controller frame carries only a power level (1-10) and a requested
// state, and every capture shows TX bytes 10-14 as 0x00 in all states.
// When a non-zero RPM is written here, send_vevor_frame() puts it as a
// big-endian uint16 into TX bytes 10-11 (mirroring the RX fan encoding,
// uint16 RPM at RX bytes 28-29) to test whether the ECM treats it as a
// fan target. 0 = disabled (protocol-default bytes are kept).
class HeaterFanProbeNumber : public number::Number, public Component {
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

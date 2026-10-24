#pragma once
#include <string>
#include <esphome/core/component.h>
namespace esphome {
namespace button {
class Button {
 public:
  virtual void press() = 0;
  void publish_state(bool) {}
};
}  // namespace button
}  // namespace esphome

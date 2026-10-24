#pragma once
#include <string>
namespace esphome {
namespace number {
class Number {
 public:
  virtual void control(float value) = 0;
  void publish_state(float) {}
};
}  // namespace number
}  // namespace esphome
// real ESPHome macro (host stub)
#define LOG_NUMBER(tag, name, obj) ((void) 0)

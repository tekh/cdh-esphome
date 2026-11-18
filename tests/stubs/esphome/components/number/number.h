#pragma once
#include <string>
namespace esphome {
namespace number {
// Minimal mirror of esphome/components/number/number_traits.h: the entity's own
// declared range. The restart-offset dial reads it to clamp writes and to warn
// when YAML `auto_shutdown_hysteresis` sits outside the slider's span.
class NumberTraits {
 public:
  void set_min_value(float min_value) { min_value_ = min_value; }
  float get_min_value() const { return min_value_; }
  void set_max_value(float max_value) { max_value_ = max_value; }
  float get_max_value() const { return max_value_; }
  void set_step(float step) { step_ = step; }
  float get_step() const { return step_; }

 protected:
  float min_value_ = 0.0f;
  float max_value_ = 100.0f;
  float step_ = 1.0f;
};

class Number {
 public:
  virtual void control(float value) = 0;
  void publish_state(float) {}
  NumberTraits traits;
};
}  // namespace number
}  // namespace esphome
// real ESPHome macro (host stub)
#define LOG_NUMBER(tag, name, obj) ((void) 0)

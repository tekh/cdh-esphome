#pragma once
#include <string>
namespace esphome {
namespace sensor {
class Sensor {
 public:
  float state = 0.0f;
  void publish_state(float) {}
};
}  // namespace sensor
}  // namespace esphome

#pragma once
#include <string>
namespace esphome {
namespace select {
class Select {
 public:
  virtual void control(const std::string &value) { (void) value; }
  void publish_state(const std::string &) {}
};
}  // namespace select
}  // namespace esphome

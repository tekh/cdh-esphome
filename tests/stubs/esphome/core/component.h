#pragma once
// Host syntax-check stub for esphome/core/component.h
#include <cstdint>

namespace esphome {

inline uint32_t millis() { return 0; }

class Component {
 public:
  virtual ~Component() = default;
  virtual void setup() {}
  virtual void loop() {}
  virtual void update() {}
  virtual void dump_config() {}
};

class PollingComponent : public Component {
 public:
  explicit PollingComponent(uint32_t update_interval = 5000) {}
  uint32_t get_update_interval() const { return 5000; }
  void set_update_interval(uint32_t) {}
};

}  // namespace esphome

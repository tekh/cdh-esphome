#pragma once
// Host syntax-check stub for esphome/components/uart/uart.h
#include <cstddef>
#include <cstdint>

#include <esphome/core/component.h>

namespace esphome {
namespace uart {

// Plain class in stub-land (real ESPHome's exact base list is irrelevant here;
// HeaterUart keeps the same base list that already compiled in v1).
class UARTDevice {
 public:
  int available() { return 0; }
  int read() { return 0; }
  void write_array(const uint8_t *data, size_t length) { (void) data; (void) length; }
  void flush() {}
};

}  // namespace uart
}  // namespace esphome

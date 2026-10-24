#pragma once

// Minimal fake ByteStream for the host driver tests: records what the driver
// writes (TX frames) and feeds scripted bytes as RX.

#include <cstdint>
#include <cstddef>
#include <vector>

#include "heater_protocol_driver.h"

struct FakeStream : public esphome::heater_uart::ByteStream {
  std::vector<uint8_t> in;        // scripted bytes the "heater" sends (RX side)
  std::vector<uint8_t> written;   // everything the driver transmitted (TX side)
  size_t pos = 0;

  int available() override { return static_cast<int>(in.size() - pos); }
  int read() override { return pos < in.size() ? static_cast<int>(in[pos++]) : -1; }
  void write_array(const uint8_t *data, size_t length) override {
    written.insert(written.end(), data, data + length);
  }
  void flush() override {}

  void feed(const std::vector<uint8_t> &bytes) { in.insert(in.end(), bytes.begin(), bytes.end()); }
};

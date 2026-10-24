#pragma once

// Vevor protocol driver (XMZ-D2 class): 0xAA/0x66 controller frames (16 bytes,
// additive checksum) and 0xAA/0x77 heater frames (56/57 bytes, length byte + 5),
// 4800 baud inverted. Owns: TX state machine (assert start/stop, poll),
// power-level encoding from the normalized fuel fraction (with open-loop
// altitude derating - the Vevor frame has no altitude field), RX framing,
// resync, checksum, glow-cluster decoding, error map, fan-probe bytes.
//
// Link-layer details: @notes/vevor-protocol.md.

#include "heater_protocol_driver.h"

namespace esphome {
namespace heater_uart {

class VevorDriver : public HeaterProtocolDriver {
 public:
  const char *name() const override { return "VEVOR (0xAA/0x66)"; }

  void begin(ByteStream *stream, const HeaterProfile &profile, uint8_t operating_voltage,
             uint16_t altitude) override;

  void update_inputs(const ControlInputs &inputs) override { inputs_ = inputs; }

  // One tick: collect/parse RX (with resync + partial-frame timeout), TX at 1 Hz.
  bool loop(uint32_t now) override;

  void request_start() override {
    pending_start_ = true;
    on_request_ = true;
  }
  void request_stop() override {
    pending_stop_ = true;
    on_request_ = false;
  }
  void pulse_stop() override { pending_stop_ = true; }
  bool on_requested() const override { return on_request_; }

  bool supports_priming() const override { return false; }  // ECM self-primes at start

  void set_fan_probe_rpm(uint16_t rpm) override { fan_probe_rpm_ = rpm; }
  uint16_t fan_probe_rpm() const override { return fan_probe_rpm_; }

  const Telemetry &telemetry() const override { return telemetry_; }
  const char *run_state_description() const override { return run_state_name(telemetry_.run_state); }
  const char *error_description() const override;

  // --- exposed for the host test harness (pure, no I/O) ---
  // Build the 16-byte controller frame. `command`/`requested_state` are the
  // TX bytes 2/9 (see take_command() for how loop() picks them).
  void build_frame(uint8_t frame[16], uint8_t command, uint8_t requested_state) const;
  // Decide this frame's command/requested-state from intents + telemetry
  // (mirrors the v1 state machine); returns them via the out params.
  void take_command(uint8_t *command, uint8_t *requested_state);
  // Parse a 52-60 byte heater frame into telemetry_ (checksum mismatch logs
  // but parses). Returns false on unexpected length.
  bool parse_frame(const uint8_t *frame, size_t length);

  // --- exposed for the host test harness ---
  static uint8_t additive_checksum(const uint8_t *data, size_t length);

 private:
  static const uint32_t TX_INTERVAL_MS = 1000;  // 1 Hz: controller talks first

  ByteStream *stream_ = nullptr;
  const HeaterProfile *profile_ = nullptr;
  uint16_t altitude_ = 0;

  ControlInputs inputs_{};
  Telemetry telemetry_{};

  bool on_request_ = false;
  bool pending_start_ = false;
  bool pending_stop_ = false;
  uint16_t fan_probe_rpm_ = 0;  // 0 = disabled (TX bytes 10-11 stay 0x00)

  // RX framing
  uint8_t rx_[64] = {0};
  int rx_index_ = 0;
  uint32_t rx_last_byte_ = 0;
  uint32_t last_tx_time_ = 0;
};

}  // namespace heater_uart
}  // namespace esphome

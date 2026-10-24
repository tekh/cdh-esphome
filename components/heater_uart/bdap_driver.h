#pragma once

// BDAP protocol driver - classic Chinese diesel heater bus: 0x76/0x16 24-byte
// frames, CRC-16/MODBUS, 25000 baud, half-duplex single-wire (we hear our own
// TX echo and discard it). Owns: TX frame build (start/stop, pump Hz, fan
// RPM, priming), RX parse, cadence (1 Hz), echo discard, RX timeout.
//
// The fan-matching computation in build_frame() is control policy that lives
// here TEMPORARILY (moved verbatim from the old build_tx_frame); Phase 3
// moves it into SafetyPolicy and passes the computed RPM via ControlInputs.

#include "heater_protocol_driver.h"

namespace esphome {
namespace heater_uart {

class BdapDriver : public HeaterProtocolDriver {
 public:
  const char *name() const override { return "BDAP (0x76/0x16)"; }

  void begin(ByteStream *stream, const HeaterProfile &profile, uint8_t operating_voltage,
             uint16_t altitude) override;

  void update_inputs(const ControlInputs &inputs) override { inputs_ = inputs; }

  // One tick: discard TX echo window, collect RX (with timeout), send at 1 Hz.
  bool loop(uint32_t now) override;

  void request_start() override {
    pending_command_ = CMD_START;
    on_request_ = true;
  }
  void request_stop() override {
    pending_command_ = CMD_STOP;
    on_request_ = false;
  }
  void pulse_stop() override { pending_command_ = CMD_STOP; }
  bool on_requested() const override { return on_request_; }

  bool supports_priming() const override { return true; }
  void set_fan_probe_rpm(uint16_t) override {}
  uint16_t fan_probe_rpm() const override { return 0; }

  const Telemetry &telemetry() const override { return telemetry_; }
  const char *run_state_description() const override { return run_state_name(telemetry_.run_state); }
  const char *error_description() const override;

  // --- exposed for the host test harness (pure, no I/O) ---
  // Build the 24-byte controller frame from the pushed inputs + telemetry.
  // `command` is the TX byte-2 value (pass CMD_NO_CHANGE / CMD_START / CMD_STOP
  // via the public constants below; loop() consumes the pending one).
  // Fan RPM is received pre-computed in ControlInputs (CombustionSafety,
  // facade) - the driver only encodes it.
  void build_frame(uint8_t frame[24], uint8_t command) const;
  // Parse a 24-byte heater status frame into telemetry_. Returns false on
  // invalid length (CRC mismatch logs but parses, as before).
  bool parse_frame(const uint8_t *frame, size_t length);
  // TX byte-2 command constants.
  static const uint8_t CMD_NO_CHANGE = 0x00;
  static const uint8_t CMD_STOP = 0x05;
  static const uint8_t CMD_START = 0xA0;

  // --- exposed for the host test harness ---
  static uint16_t crc16(const uint8_t *data, size_t length);
  // Echo-discard window (ms) right after TX; tests set 0.
  void set_echo_delay_ms(uint32_t ms) { echo_delay_ms_ = ms; }

 private:
  static const uint32_t TX_INTERVAL_MS = 1000;  // 1 Hz controller cadence
  static const uint32_t RX_TIMEOUT_MS = 200;    // heater answers in ~100-130 ms

  ByteStream *stream_ = nullptr;
  const HeaterProfile *profile_ = nullptr;
  uint8_t operating_voltage_ = 0;
  uint16_t altitude_ = 0;

  ControlInputs inputs_{};
  Telemetry telemetry_{};

  bool on_request_ = false;
  uint8_t pending_command_ = CMD_NO_CHANGE;

  // RX state machine
  uint8_t rx_frame_[24] = {0};
  int rx_frame_index_ = 0;
  bool awaiting_rx_ = false;
  uint32_t rx_timeout_ = 0;

  // TX echo handling (single-wire: we hear our own transmission).
  // Echo-discard window (ms) right after TX; tests set 0.
  uint32_t echo_delay_ms_ = 15;  // ~24 bytes @ 25000 baud = 9.6 ms + margin
  uint32_t last_tx_time_ = 0;
  int echo_bytes_left_ = 0;
  uint32_t echo_until_ = 0;
};

}  // namespace heater_uart
}  // namespace esphome

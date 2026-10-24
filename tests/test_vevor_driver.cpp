#include "tiny_test.h"

// VevorDriver host tests: TX state machine, level encoding + altitude derate,
// RX decode (state map, signed HX int16, glow cluster, error map), framing
// resync, echo ignore.

#include "fake_stream.h"
#include <cstring>

#include "vevor_driver.h"

using esphome::heater_uart::ControlInputs;
using esphome::heater_uart::HeaterProfile;
using esphome::heater_uart::RUN_STATE_COOLDOWN;
using esphome::heater_uart::RUN_STATE_GLOW_PREHEAT;
using esphome::heater_uart::RUN_STATE_IGNITED;
using esphome::heater_uart::RUN_STATE_OFF;
using esphome::heater_uart::RUN_STATE_RUNNING;
using esphome::heater_uart::Telemetry;
using esphome::heater_uart::VevorDriver;

static HeaterProfile vevor_profile() {
  HeaterProfile p{};
  p.name = "test-vevor";
  p.protocol = esphome::heater_uart::HeaterProtocol::VEVOR_UART;
  p.pump_freq_min = 0.5f;
  p.pump_freq_max = 2.5f;
  p.pump_freq_step = 0.1f;
  p.pump_freq_initial = 1.0f;
  p.pump_prime_freq = 2.5f;
  return p;
}

// Build a 57-byte heater status frame (length field 0x34) with the given values.
static std::vector<uint8_t> make_rx(uint8_t state, uint8_t level, uint8_t error,
                                    int16_t hx_x10, uint8_t supply_x10, uint8_t pump_x10,
                                    uint16_t fan_rpm, uint8_t fan_voltage) {
  std::vector<uint8_t> f(57, 0);
  f[0] = 0xAA;
  f[1] = 0x77;
  f[3] = 0x34;       // length field -> 57
  f[5] = state;
  f[6] = level;
  f[7] = error;
  f[11] = supply_x10;
  f[15] = fan_voltage;
  f[16] = (hx_x10 >> 8) & 0xFF;
  f[17] = hx_x10 & 0xFF;
  f[23] = pump_x10;
  f[24] = 81;        // glow cluster
  f[25] = 90;
  f[26] = 95;
  f[27] = 7;         // glow current (candidate)
  f[28] = (fan_rpm >> 8) & 0xFF;
  f[29] = fan_rpm & 0xFF;
  f[56] = VevorDriver::additive_checksum(f.data(), 57);
  return f;
}

TEST(vevor_tx_frame_layout) {
  const HeaterProfile p = vevor_profile();
  VevorDriver d;
  d.begin(nullptr, p, 0x78, 0);  // altitude 0: no derate
  ControlInputs in;
  in.output_fraction = 0.5f;  // -> level 6 (1 + round(0.5*9) = 6)
  in.desired_temp = 19.5f;
  d.update_inputs(in);

  uint8_t f[16];
  d.build_frame(f, 0x02, 0x08);
  CHECK(f[0] == 0xAA);
  CHECK(f[1] == 0x66);
  CHECK(f[2] == 0x02);
  CHECK(f[3] == 0x0B);
  CHECK(f[8] == 6);          // power level from fraction
  CHECK(f[9] == 0x08);
  CHECK(f[10] == 0x00);      // probe off
  CHECK(f[11] == 0x00);
  CHECK(f[15] == VevorDriver::additive_checksum(f, 16));
}

TEST(vevor_tx_level_encoding_extremes) {
  const HeaterProfile p = vevor_profile();
  VevorDriver d;
  d.begin(nullptr, p, 0x78, 0);
  uint8_t f[16];

  ControlInputs in;
  in.output_fraction = 0.0f;
  d.update_inputs(in);
  d.build_frame(f, 0x02, 0x08);
  CHECK(f[8] == 1);

  in.output_fraction = 1.0f;
  d.update_inputs(in);
  d.build_frame(f, 0x02, 0x08);
  CHECK(f[8] == 10);
}

TEST(vevor_tx_altitude_derate) {
  const HeaterProfile p = vevor_profile();
  VevorDriver d;
  d.begin(nullptr, p, 0x78, 762);  // sigma(762) ~ 0.929
  uint8_t f[16];

  // running: derate applies. fraction 0.9 * 0.929 = 0.836 -> level 1+round(7.52)=9
  const auto rx = make_rx(3, 5, 0, 1800, 132, 20, 2600, 10);
  CHECK(d.parse_frame(rx.data(), rx.size()));

  ControlInputs in;
  in.output_fraction = 0.9f;
  d.update_inputs(in);
  d.build_frame(f, 0x02, 0x08);
  CHECK(f[8] == 9);

  // NOT running (e.g. ignition): no derate -> level 1+round(8.1)=9... use 0.5:
  // fraction 0.5 -> level 1 + round(4.5) = 6 (round-half-up, v1 formula)
  const auto rx2 = make_rx(2, 5, 0, 1800, 132, 20, 2600, 10);  // state 2 = ignited
  CHECK(d.parse_frame(rx2.data(), rx2.size()));
  in.output_fraction = 0.5f;
  d.update_inputs(in);
  d.build_frame(f, 0x02, 0x08);
  CHECK(f[8] == 6);
}

TEST(vevor_tx_fan_probe_bytes) {
  const HeaterProfile p = vevor_profile();
  VevorDriver d;
  d.begin(nullptr, p, 0x78, 0);
  ControlInputs in;
  in.output_fraction = 0.5f;
  d.update_inputs(in);

  d.set_fan_probe_rpm(0);
  uint8_t f[16];
  d.build_frame(f, 0x02, 0x08);
  CHECK(f[10] == 0x00);
  CHECK(f[11] == 0x00);

  d.set_fan_probe_rpm(0x1234);
  d.build_frame(f, 0x02, 0x08);
  CHECK(f[10] == 0x12);
  CHECK(f[11] == 0x34);
}

TEST(vevor_command_state_machine) {
  const HeaterProfile p = vevor_profile();
  VevorDriver d;
  d.begin(nullptr, p, 0x78, 0);
  ControlInputs in;
  in.output_fraction = 0.5f;
  d.update_inputs(in);
  uint8_t cmd, state;

  // 1. pending start -> one-shot 0x06/0x06
  d.request_start();
  d.take_command(&cmd, &state);
  CHECK(cmd == 0x06);
  CHECK(state == 0x06);

  // 2. on-request, heater OFF -> keep asserting start
  d.take_command(&cmd, &state);
  CHECK(cmd == 0x06);
  CHECK(state == 0x06);

  // 3. on-request, heater RUNNING -> status poll 0x02/0x08
  const auto rx = make_rx(3, 5, 0, 1800, 132, 20, 2600, 10);
  CHECK(d.parse_frame(rx.data(), rx.size()));
  d.take_command(&cmd, &state);
  CHECK(cmd == 0x02);
  CHECK(state == 0x08);

  // 4. pulse stop (ambient limit), heater running -> assert stop, intent KEPT
  d.pulse_stop();
  d.take_command(&cmd, &state);
  CHECK(cmd == 0x06);
  CHECK(state == 0x05);
  CHECK(d.on_requested() == true);

  // 5. real stop (mode change) -> intent cleared
  d.request_stop();
  d.take_command(&cmd, &state);
  CHECK(cmd == 0x06);
  CHECK(state == 0x05);
  CHECK(d.on_requested() == false);

  // 6. heater keeps running with no intent -> persistent stop assertions
  d.take_command(&cmd, &state);
  CHECK(cmd == 0x06);
  CHECK(state == 0x05);

  // 7. heater entered cooldown -> idle status poll
  const auto rxc = make_rx(4, 1, 0, 1800, 132, 20, 1500, 8);
  CHECK(d.parse_frame(rxc.data(), rxc.size()));
  d.take_command(&cmd, &state);
  CHECK(cmd == 0x02);
  CHECK(state == 0x02);

  // 8. heater off -> idle status poll
  const auto rxo = make_rx(0, 0, 0, 1800, 132, 0, 0, 0);
  CHECK(d.parse_frame(rxo.data(), rxo.size()));
  d.take_command(&cmd, &state);
  CHECK(cmd == 0x02);
  CHECK(state == 0x02);
}

TEST(vevor_rx_parse_fields) {
  const HeaterProfile p = vevor_profile();
  VevorDriver d;
  d.begin(nullptr, p, 0x78, 0);

  const auto rx = make_rx(1, 7, 0x08, -123, 124, 15, 2400, 9);
  CHECK(d.parse_frame(rx.data(), rx.size()));
  const Telemetry &t = d.telemetry();
  CHECK(t.run_state == RUN_STATE_GLOW_PREHEAT);
  CHECK(t.on == true);
  CHECK(t.power_level_actual == 7);
  CHECK_NEAR(t.hx_temp, -12.3f, 1e-3);     // SIGNED int16 x 0.1
  CHECK_NEAR(t.supply_voltage, 12.4f, 1e-4);
  CHECK_NEAR(t.pump_hz, 1.5f, 1e-4);
  CHECK(t.fan_speed == 2400);
  CHECK_NEAR(t.fan_voltage, 9.0f, 1e-4);
  CHECK(t.error_code == 0x08);
  CHECK_NEAR(t.glow_a, 81.0f, 1e-4);
  CHECK_NEAR(t.glow_b, 90.0f, 1e-4);
  CHECK_NEAR(t.glow_c, 95.0f, 1e-4);
  CHECK_NEAR(t.glow_plug_current, 7.0f, 1e-4);
  CHECK(std::strcmp(d.error_description(), "E03 - Check Ignition") == 0);
}

TEST(vevor_rx_state_mapping) {
  const HeaterProfile p = vevor_profile();
  VevorDriver d;
  d.begin(nullptr, p, 0x78, 0);

  struct { uint8_t raw; uint8_t mapped; bool on; } cases[] = {
      {0, RUN_STATE_OFF, false},   {1, RUN_STATE_GLOW_PREHEAT, true},
      {2, RUN_STATE_IGNITED, true}, {3, RUN_STATE_RUNNING, true},
      {4, RUN_STATE_COOLDOWN, true}, {9, RUN_STATE_OFF, true},  // unknown state: OFF, but on=true (v1: on = raw != 0)
  };
  for (const auto c : cases) {
    const auto rx = make_rx(c.raw, 1, 0, 1800, 132, 20, 2600, 10);
    CHECK(d.parse_frame(rx.data(), rx.size()));
    CHECK(d.telemetry().run_state == c.mapped);
    CHECK(d.telemetry().on == c.on);
  }
}

TEST(vevor_rx_bad_length_rejected) {
  VevorDriver d;
  d.begin(nullptr, vevor_profile(), 0x78, 0);
  uint8_t f[40] = {0};
  CHECK(d.parse_frame(f, 40) == false);
}

TEST(vevor_loop_framing_and_echo) {
  const HeaterProfile p = vevor_profile();
  FakeStream stream;
  VevorDriver d;
  d.begin(&stream, p, 0x78, 0);
  ControlInputs in;
  in.output_fraction = 0.5f;
  in.desired_temp = 19.0f;
  d.update_inputs(in);

  // first tick: idle status request TX (16 bytes)
  CHECK(d.loop(1000) == false);
  CHECK(stream.written.size() == 16);
  CHECK(stream.written[0] == 0xAA);
  CHECK(stream.written[1] == 0x66);

  // garbage then our own echo then a heater frame: only the 0xAA 0x77 one parses
  const auto rx = make_rx(3, 5, 0, 1800, 132, 20, 2600, 10);
  stream.feed({0x55, 0x77});     // noise without start marker
  stream.feed(stream.written);   // our own TX echo (id 0x66 -> ignored)
  stream.feed(rx);
  CHECK(d.loop(1005) == true);   // the heater frame parsed
  CHECK(d.telemetry().run_state == RUN_STATE_RUNNING);
  CHECK(stream.written.size() == 16);  // cadence: no second TX within 1 s

  // implausible length field -> resync, frame dropped, driver keeps working
  stream.feed({0xAA, 0x77, 0x00, 0xFF, 0x01, 0x02});  // 0xFF+5 > 60 -> resync
  CHECK(d.loop(1100) == false);
  CHECK(d.telemetry().run_state == RUN_STATE_RUNNING);  // previous value kept
  CHECK(d.loop(2000) == false);   // next TX tick
  CHECK(stream.written.size() == 32);
}

TEST(vevor_partial_frame_timeout_resyncs) {
  const HeaterProfile p = vevor_profile();
  FakeStream stream;
  VevorDriver d;
  d.begin(&stream, p, 0x78, 0);

  stream.feed({0xAA, 0x77, 0x00, 0x34});  // header only, rest never arrives
  CHECK(d.loop(1000) == false);
  CHECK(d.loop(1100) == false);           // within 300 ms: still waiting
  CHECK(d.loop(1301) == false);           // timeout -> resync
  // after resync the driver accepts a fresh frame again
  const auto rx = make_rx(0, 0, 0, 1800, 132, 0, 0, 0);
  stream.feed(rx);
  CHECK(d.loop(1400) == true);
  CHECK(d.telemetry().run_state == RUN_STATE_OFF);
}

TEST(vevor_priming_unsupported) {
  VevorDriver d;
  CHECK(d.supports_priming() == false);
}

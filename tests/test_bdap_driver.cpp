#include "tiny_test.h"

// BdapDriver host tests: frame layout, parse, cadence, echo discard.
// The byte layout assertions are the protocol contract; feeding a corrupted
// frame asserts the validation behaviour.

#include "fake_stream.h"
#include "bdap_driver.h"

using esphome::heater_uart::BdapDriver;
using esphome::heater_uart::ControlInputs;
using esphome::heater_uart::HeaterProfile;
using esphome::heater_uart::RUN_STATE_RUNNING;
using esphome::heater_uart::Telemetry;

static HeaterProfile bdap_profile() {
  HeaterProfile p{};
  p.name = "test";
  p.protocol = esphome::heater_uart::HeaterProtocol::BDAP;
  p.pump_freq_min = 1.3f;
  p.pump_freq_max = 5.5f;
  p.pump_freq_step = 0.1f;
  p.pump_freq_initial = 1.8f;
  p.pump_prime_freq = 5.0f;
  p.fan_rpm_min = 1450;
  p.fan_rpm_max = 4500;
  p.ignition_fan_rpm = 2000;
  p.ignition_hx_threshold = 100.0f;
  p.cooldown_fan_rpm = 4000;
  p.fan_sensor = 0x01;
  p.glow_power = 0x05;
  return p;
}

// Build a syntactically valid heater status frame with the given values.
static std::vector<uint8_t> make_rx(uint8_t run_state, uint8_t on, uint16_t supply_x10,
                                    uint16_t fan_rpm, uint16_t hx, uint8_t pump_x10,
                                    uint8_t error) {
  std::vector<uint8_t> f(24, 0);
  f[0] = 0x76;
  f[1] = 0x16;
  f[2] = run_state;
  f[3] = on;
  f[4] = (supply_x10 >> 8) & 0xFF;
  f[5] = supply_x10 & 0xFF;
  f[6] = (fan_rpm >> 8) & 0xFF;
  f[7] = fan_rpm & 0xFF;
  f[10] = (hx >> 8) & 0xFF;
  f[11] = hx & 0xFF;
  f[15] = 0x05;  // glow current: BE bytes 14-15, 0.01 A units -> 0.05 A
  f[16] = pump_x10;
  f[17] = error;
  const uint16_t crc = BdapDriver::crc16(f.data(), 22);
  f[22] = (crc >> 8) & 0xFF;
  f[23] = crc & 0xFF;
  return f;
}

TEST(bdap_tx_frame_layout) {
  const HeaterProfile p = bdap_profile();
  BdapDriver d;
  d.begin(nullptr, p, 0x78, 750);
  ControlInputs in;
  in.output_fraction = 0.5f;                       // -> 3.4 Hz
  in.desired_temp = 19.5f;
  in.room_temp = 18.0f;
  d.update_inputs(in);

  uint8_t f[24];
  d.build_frame(f, BdapDriver::CMD_START);

  CHECK(f[0] == 0x76);      // start marker
  CHECK(f[1] == 0x16);      // protocol version
  CHECK(f[2] == BdapDriver::CMD_START);
  CHECK(f[3] == 18);        // room temp
  CHECK(f[4] == 20);        // desired temp (round(19.5) -> uint8)
  // pump bytes: 0.5 fraction -> 3.4 Hz -> 34 (0.1 Hz units)
  CHECK(f[5] == 34);
  CHECK(f[6] == 34);
  // fan bytes: ratio 0.5 -> 1450 + 0.5*(4500-1450) = 2975
  CHECK(f[7] == ((2975 >> 8) & 0xFF));
  CHECK(f[8] == (2975 & 0xFF));
  CHECK(f[9] == f[7]);
  CHECK(f[10] == f[8]);
  CHECK(f[11] == 0x78);     // 12 V
  CHECK(f[12] == 0x01);     // fan sensor (profile)
  CHECK(f[13] == 0x05);     // glow power (profile)
  CHECK(f[16] == 0x00);     // no priming
  CHECK(f[17] == 0x00);
  CHECK(f[20] == ((750 >> 8) & 0xFF));  // altitude BE
  CHECK(f[21] == (750 & 0xFF));
  // CRC self-consistency
  const uint16_t crc = BdapDriver::crc16(f, 22);
  CHECK(f[22] == ((crc >> 8) & 0xFF));
  CHECK(f[23] == (crc & 0xFF));
}

TEST(bdap_tx_priming_overrides_pump) {
  const HeaterProfile p = bdap_profile();
  BdapDriver d;
  d.begin(nullptr, p, 0x78, 750);
  ControlInputs in;
  in.output_fraction = 0.1f;
  in.priming_active = true;
  d.update_inputs(in);

  uint8_t f[24];
  d.build_frame(f, BdapDriver::CMD_NO_CHANGE);
  // prime rate 5.0 Hz -> 50 (0.1 Hz units) in pump bytes AND prime bytes
  CHECK(f[5] == 50);
  CHECK(f[6] == 50);
  CHECK(f[16] == 50);
  CHECK(f[17] == 0x5A);     // manual priming mode
}

TEST(bdap_tx_cooldown_fan) {
  const HeaterProfile p = bdap_profile();
  BdapDriver d;
  d.begin(nullptr, p, 0x78, 750);
  ControlInputs in;
  in.output_fraction = 0.9f;
  in.cooldown_active = true;
  d.update_inputs(in);

  uint8_t f[24];
  d.build_frame(f, BdapDriver::CMD_NO_CHANGE);
  CHECK(f[7] == ((4000 >> 8) & 0xFF));   // cooldown fan RPM
  CHECK(f[8] == (4000 & 0xFF));
}

TEST(bdap_tx_ignition_fan_hold) {
  const HeaterProfile p = bdap_profile();
  BdapDriver d;
  d.begin(nullptr, p, 0x78, 750);
  ControlInputs in;
  in.output_fraction = 1.0f;  // full fuel...
  d.update_inputs(in);
  // ...but heater just ignited with HX below threshold -> ignition fan speed
  // (state 4 injected via a crafted RX frame; hx 80 < 100 threshold)
  const auto rx = make_rx(4, 1, 120, 1500, 80, 18, 0);
  CHECK(d.parse_frame(rx.data(), rx.size()));

  uint8_t f[24];
  d.build_frame(f, BdapDriver::CMD_NO_CHANGE);
  CHECK(f[7] == ((2000 >> 8) & 0xFF));   // ignition_fan_rpm
  CHECK(f[8] == (2000 & 0xFF));
}

TEST(bdap_rx_parse_fields) {
  const HeaterProfile p = bdap_profile();
  BdapDriver d;
  d.begin(nullptr, p, 0x78, 750);

  const auto rx = make_rx(RUN_STATE_RUNNING, 1, 132, 2870, 245, 22, 2);
  CHECK(d.parse_frame(rx.data(), rx.size()));
  const Telemetry &t = d.telemetry();
  CHECK(t.run_state == RUN_STATE_RUNNING);
  CHECK(t.on == true);
  CHECK_NEAR(t.supply_voltage, 13.2f, 1e-4);
  CHECK(t.fan_speed == 2870);
  CHECK_NEAR(t.hx_temp, 245.0f, 1e-4);
  CHECK_NEAR(t.glow_plug_current, 0.05f, 1e-4);
  CHECK_NEAR(t.pump_hz, 2.2f, 1e-4);
  CHECK(t.error_code == 2);
}

TEST(bdap_rx_bad_length_rejected) {
  BdapDriver d;
  d.begin(nullptr, bdap_profile(), 0x78, 750);
  uint8_t short_frame[20] = {0};
  CHECK(d.parse_frame(short_frame, 20) == false);
}

TEST(bdap_rx_crc_mismatch_still_parses) {
  const HeaterProfile p = bdap_profile();
  BdapDriver d;
  d.begin(nullptr, p, 0x78, 750);
  auto rx = make_rx(RUN_STATE_RUNNING, 1, 132, 2870, 245, 22, 0);
  rx[22] ^= 0xFF;  // corrupt CRC
  CHECK(d.parse_frame(rx.data(), rx.size()) == true);  // logged, not dropped
}

TEST(bdap_loop_cadence_and_echo) {
  const HeaterProfile p = bdap_profile();
  FakeStream stream;
  BdapDriver d;
  d.begin(&stream, p, 0x78, 750);
  d.set_echo_delay_ms(15);

  ControlInputs in;
  in.output_fraction = 0.5f;
  in.desired_temp = 19.0f;
  in.room_temp = 18.0f;
  d.update_inputs(in);

  // t=1000: first TX (last_tx_time_=0)
  CHECK(d.loop(1000) == false);
  CHECK(stream.written.size() == 24);

  // echo arrives within the window -> discarded, not parsed as heater data;
  // the heater's answer right behind it parses.
  const auto rx = make_rx(RUN_STATE_RUNNING, 1, 132, 2870, 245, 22, 0);
  stream.feed(stream.written);  // our own TX echoes back (24 B)
  stream.feed(rx);              // then the heater's answer
  CHECK(d.loop(1005) == true);  // echo window consumes 24 B, then parses RX
  CHECK(d.telemetry().run_state == RUN_STATE_RUNNING);
  CHECK(d.telemetry().fan_speed == 2870);
  CHECK(stream.written.size() == 24);  // no second TX within the 1 s cadence

  // t=2000: next TX tick, START intent consumed once
  d.request_start();
  CHECK(d.loop(2000) == false);
  CHECK(stream.written.size() == 48);
  CHECK(stream.written[24 + 2] == BdapDriver::CMD_START);   // second frame byte 2

  // the heater answers; while awaiting the response there is no new TX
  stream.feed(rx);
  CHECK(d.loop(2500) == true);
  CHECK(stream.written.size() == 48);

  // t=3010: third tick; the one-shot START was consumed by the 2000 frame
  CHECK(d.loop(3010) == false);
  CHECK(stream.written.size() == 72);
  CHECK(stream.written[48 + 2] == BdapDriver::CMD_NO_CHANGE);  // one-shot consumed
}

TEST(bdap_pulse_stop_keeps_on_intent) {
  BdapDriver d;
  d.begin(nullptr, bdap_profile(), 0x78, 750);
  d.request_start();
  CHECK(d.on_requested() == true);
  d.pulse_stop();  // ambient-limit semantics: stop asserted, intent kept
  CHECK(d.on_requested() == true);
  d.request_stop();
  CHECK(d.on_requested() == false);
}

TEST(bdap_priming_supported_fan_probe_not) {
  BdapDriver d;
  CHECK(d.supports_priming() == true);
  d.set_fan_probe_rpm(1234);  // no-op on BDAP
  CHECK(d.fan_probe_rpm() == 0);
}

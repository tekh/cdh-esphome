#pragma once

// HeaterProtocolDriver - the seam between the controller and the wire
// (ROADMAP.md Phase 2).
//
// The controller (HeaterUart + future ControlPolicy/SafetyPolicy) works in a
// normalized domain: a fuel fraction 0.0-1.0, start/stop intents, and the
// shared run-state enumeration below. Each driver owns ONE protocol's facts:
// framing, cadence, echo handling, resync, checksums, byte encode/decode,
// error-code tables. Adding a heater that speaks a NEW protocol = one new
// driver; adding one that speaks an existing protocol = zero new driver code.
//
// Everything in this header is pure C++ (no ESPHome includes) so the drivers
// build in the host test harness.

#include <cstddef>
#include <cstdint>

#include "heater_profile.h"

namespace esphome {
namespace heater_uart {

// Byte-level transport the drivers talk to. Implemented by HeaterUart
// (delegating to its uart::UARTDevice) on the device; by a fake stream in the
// host tests.
class ByteStream {
 public:
  virtual ~ByteStream() = default;
  virtual int available() = 0;
  virtual int read() = 0;
  virtual void write_array(const uint8_t *data, size_t length) = 0;
  virtual void flush() = 0;
};

// Normalized run-state enumeration (the original BDAP values; the Vevor
// driver maps its states onto these). Shared by both drivers and the policy.
static const uint8_t RUN_STATE_OFF = 0;
static const uint8_t RUN_STATE_START_ACK = 1;
static const uint8_t RUN_STATE_GLOW_PREHEAT = 2;
static const uint8_t RUN_STATE_FAILED_IGNITION = 3;
static const uint8_t RUN_STATE_IGNITED = 4;
static const uint8_t RUN_STATE_RUNNING = 5;
static const uint8_t RUN_STATE_STOP_ACK = 6;
static const uint8_t RUN_STATE_POST_GLOW = 7;
static const uint8_t RUN_STATE_COOLDOWN = 8;

// Parsed heater status, protocol-normalized. Filled by the driver from RX
// frames; read by the controller for publishing and by the (future) policies.
struct Telemetry {
  uint8_t run_state = RUN_STATE_OFF;
  bool on = false;
  float supply_voltage = 0.0f;     // V
  int fan_speed = 0;               // RPM
  float fan_voltage = 0.0f;        // V
  float hx_temp = 0.0f;            // heat exchanger, °C
  float glow_plug_voltage = 0.0f; // V
  float glow_plug_current = 0.0f; // A
  // Vevor glow-cluster bytes 24-26 (ramp together, quantity unidentified);
  // published raw/unit-less. Zero on BDAP (real voltage/current above).
  float glow_a = 0.0f;
  float glow_b = 0.0f;
  float glow_c = 0.0f;
  float pump_hz = 0.0f;             // reported fuel rate
  int power_level_actual = 0;       // level the ECM reports running at (Vevor)
  int error_code = 0;
};

// Control state the controller pushes every loop, before loop(). The drivers
// encode it into their frames at their own cadence.
struct ControlInputs {
  float output_fraction = 0.0f;  // fuel request, normalized 0..1 (output_domain)
  uint16_t fan_rpm = 0;          // fan target, pre-computed by CombustionSafety (facade);
                                 // encoded by BDAP-class frames, ignored by level-only buses
  float desired_temp = 19.0f;    // °C setpoint (BDAP TX byte 4)
  float room_temp = 0.0f;        // last accepted room reading (BDAP TX byte 3)
  bool priming_active = false;   // fuel priming (BDAP prime bytes)
};

class HeaterProtocolDriver {
 public:
  virtual ~HeaterProtocolDriver() = default;

  virtual const char *name() const = 0;

  // Bind the driver to its transport and static configuration. Called once
  // from HeaterUart::setup() after the profile is resolved.
  virtual void begin(ByteStream *stream, const HeaterProfile &profile,
                     uint8_t operating_voltage, uint16_t altitude) = 0;

  // Push the controller's current state; called every loop before loop().
  virtual void update_inputs(const ControlInputs &inputs) = 0;

  // One scheduler tick: TX at protocol cadence + RX collection/parsing.
  // Returns true when at least one fresh RX frame was parsed this tick.
  virtual bool loop(uint32_t now) = 0;

  // Start/stop intents.
  // request_start(): persistent "want heat" intent + one-shot start command.
  // request_stop(): persistent stop - clears the on-intent (mode changes,
  //                 safety stops, auto-shutdown, sensor-fault stops).
  // pulse_stop():    one-shot stop assert WITHOUT clearing the on-intent.
  //                 Ambient-heat-limit semantics: in HEAT mode the heater
  //                 restarts once the room cools below the limit (the check
  //                 re-asserts every second while hot). Preserves the exact
  //                 v1 behaviour where heater_on_request_ stayed true.
  virtual void request_start() = 0;
  virtual void request_stop() = 0;
  virtual void pulse_stop() = 0;

  // Current persistent start intent ("the user wants heat"). Gates priming
  // and the stale-sensor stop, exactly as the v1 heater_on_request_ did.

  virtual bool on_requested() const = 0;

  // Capability: can this protocol prime the pump on command? (BDAP yes,
  // Vevor no - the ECM self-primes during every start.)
  virtual bool supports_priming() const = 0;

  // Experimental Vevor fan-speed probe (undocumented TX bytes 10-11);
  // no-op on other protocols.
  virtual void set_fan_probe_rpm(uint16_t rpm) = 0;
  virtual uint16_t fan_probe_rpm() const = 0;

  // Latest parsed status.
  virtual const Telemetry &telemetry() const = 0;

  // Human-readable descriptions of the latest run state / error code.
  virtual const char *run_state_description() const = 0;
  virtual const char *error_description() const = 0;
};

// Shared run-state names (normalized enumeration above). The original BDAP
// strings, byte-for-byte.
inline const char *run_state_name(uint8_t run_state) {
  switch (run_state) {
    case RUN_STATE_OFF:
      return "Off / Standby";
    case RUN_STATE_START_ACK:
      return "Start Acknowledge";
    case RUN_STATE_GLOW_PREHEAT:
      return "Glow Plug Pre-heat";
    case RUN_STATE_FAILED_IGNITION:
      return "Failed Ignition - Pause for Retry";
    case RUN_STATE_IGNITED:
      return "Ignited \xE2\x80\x93 Heating to Full Temp";
    case RUN_STATE_RUNNING:
      return "Running";
    case RUN_STATE_STOP_ACK:
      return "Stop Acknowledge";
    case RUN_STATE_POST_GLOW:
      return "Stopping - Post Run Glow Re-heat";
    case RUN_STATE_COOLDOWN:
      return "Cooldown";
    default:
      return "Unknown Run State";
  }
}

}  // namespace heater_uart
}  // namespace esphome

#include "vevor_driver.h"

#include <cmath>
#include <cstring>

#include "heater_log.h"
#include "heater_output.h"

namespace esphome {
namespace heater_uart {

[[maybe_unused]] static const char *TAG = "heater_uart.vevor";

// TX frame command bytes (byte 2) and requested-state bytes (byte 9).
static const uint8_t VEVOR_CMD_STATUS = 0x02;
static const uint8_t VEVOR_CMD_RUNSTOP = 0x06;
static const uint8_t VEVOR_STATE_OFF = 0x02;        // idle status request
static const uint8_t VEVOR_STATE_STOP = 0x05;
static const uint8_t VEVOR_STATE_START = 0x06;
static const uint8_t VEVOR_STATE_RUNNING = 0x08;    // running status request

// Vevor error codes (from the zatakon/vevor_heater_control reverse engineering;
// moved verbatim from heater_uart.cpp).
static const char *vevor_error_name(int code) {
  switch (code) {
    case 0x00: return "No Error";
    case 0x01: return "E10 - Startup Failure";
    case 0x02: return "E08 - Check Fuel Level";
    case 0x03: return "E01 - Supply Voltage Overrun";
    case 0x04: return "E04 - ?";
    case 0x05: return "E05 - ?";
    case 0x06: return "E04 - Fuel Pump Failure";
    case 0x07: return "E06 - Fan Failure";
    case 0x08: return "E03 - Check Ignition";
    case 0x09: return "E05 - Overheat";
    default:   return "Unknown Error Code";
  }
}

// Air-density ratio sigma(h) = rho/rho0 (International Standard Atmosphere).
// Fuel flow must scale by this factor at altitude or combustion runs rich
// (soot, HX fouling). e.g. 762 m -> ~0.929 (-7.1 %).
static float air_density_ratio(float altitude_m) {
  return std::pow(1.0f - 0.0065f * altitude_m / 288.15f, 4.2561f);
}

// Additive checksum: sum of bytes 2..len-2 (all but start/id/last), mod 256.
uint8_t VevorDriver::additive_checksum(const uint8_t *data, size_t length) {
  uint32_t sum = 0;
  for (size_t i = 2; i < length - 1; ++i) {
    sum += data[i];
  }
  return static_cast<uint8_t>(sum & 0xFF);
}

void VevorDriver::begin(ByteStream *stream, const HeaterProfile &profile, uint8_t /*operating_voltage*/,
                        uint16_t altitude) {
  stream_ = stream;
  profile_ = &profile;
  altitude_ = altitude;
  memset(rx_, 0, sizeof(rx_));
}

const char *VevorDriver::error_description() const { return vevor_error_name(telemetry_.error_code); }

bool VevorDriver::loop(uint32_t now) {
  bool fresh = false;

  // Continuously collect frames. Resync on 0xAA; ignore our own echo (id 0x66).
  while (stream_->available() > 0) {
    const uint8_t byte = static_cast<uint8_t>(stream_->read());
    rx_last_byte_ = now;
    if (rx_index_ == 0) {
      if (byte == 0xAA) {
        rx_[rx_index_++] = byte;
      }
      // else: discard byte, keep hunting for the start marker
    } else {
      rx_[rx_index_++] = byte;
      if (rx_index_ >= 4) {
        // Frame length = length-field byte + 5 (observed: 0x0B->16 controller,
        // 0x33->56 and 0x34->57 heater frames)
        const uint8_t expected = rx_[3] + 5;
        if (expected < 16 || expected > 60) {
          // Garbage length field - resync instead of overflowing the buffer
          HEATER_LOGW(TAG, "VEVOR: implausible length field 0x%02X, resyncing", rx_[3]);
          rx_index_ = 0;
          continue;
        }
        if (rx_index_ >= expected) {
          uint8_t frame[64];
          memcpy(frame, rx_, expected);
          rx_index_ = 0;
          if (frame[1] == 0x66) {
            HEATER_LOGD(TAG, "VEVOR: own echo ignored (controller id 0x66)");
            continue;
          }
          fresh = parse_frame(frame, expected) || fresh;
        } else if (rx_index_ > expected + 8) {
          HEATER_LOGW(TAG, "VEVOR: frame too long (%d bytes), resyncing", rx_index_);
          rx_index_ = 0;
        }
      }
    }
  }

  // Partial frame timeout - resync
  if (rx_index_ > 0 && (now - rx_last_byte_) > 300) {
    HEATER_LOGW(TAG, "VEVOR: partial frame timeout (%d bytes), resyncing", rx_index_);
    rx_index_ = 0;
  }

  // 1 Hz controller cadence - controller always talks first, heater answers.
  if (now - last_tx_time_ >= TX_INTERVAL_MS) {
    uint8_t command;
    uint8_t requested_state;
    take_command(&command, &requested_state);
    uint8_t frame[16];
    build_frame(frame, command, requested_state);

    HEATER_LOGD(TAG,
                "VEVOR TX: %02X %02X %02X %02X %02X %02X %02X %02X | %02X %02X %02X %02X %02X %02X %02X %02X",
                frame[0], frame[1], frame[2], frame[3], frame[4], frame[5], frame[6], frame[7],
                frame[8], frame[9], frame[10], frame[11], frame[12], frame[13], frame[14], frame[15]);
    stream_->write_array(frame, 16);
    stream_->flush();
    last_tx_time_ = now;
  }
  return fresh;
}

void VevorDriver::take_command(uint8_t *command, uint8_t *requested_state) {
  if (pending_start_) {
    *command = VEVOR_CMD_RUNSTOP;  // start
    *requested_state = VEVOR_STATE_START;
    pending_start_ = false;
    HEATER_LOGI(TAG, "VEVOR: sending START (power %d)",
                static_cast<int>(output_domain::level_from_normalized(inputs_.output_fraction)));
  } else if (pending_stop_) {
    *command = VEVOR_CMD_RUNSTOP;  // stop
    *requested_state = VEVOR_STATE_STOP;
    pending_stop_ = false;
    HEATER_LOGI(TAG, "VEVOR: sending STOP");
  } else if (on_request_) {
    if (telemetry_.run_state == RUN_STATE_OFF) {
      // keep asserting start until the heater comes up
      *command = VEVOR_CMD_RUNSTOP;
      *requested_state = VEVOR_STATE_START;
    } else {
      *command = VEVOR_CMD_STATUS;  // running status request
      *requested_state = VEVOR_STATE_RUNNING;
    }
  } else if (telemetry_.run_state != RUN_STATE_OFF && telemetry_.run_state != RUN_STATE_COOLDOWN) {
    // Heater has not yet entered OFF/cooldown: keep asserting STOP every second.
    // (Live finding: the Vevor ignores a one-shot 0x06/0x05 and only stops on a
    // persistent request until raw state becomes 0 (off) or 4 (stopping/cooling).)
    *command = VEVOR_CMD_RUNSTOP;
    *requested_state = VEVOR_STATE_STOP;
  } else {
    *command = VEVOR_CMD_STATUS;  // idle status request
    *requested_state = VEVOR_STATE_OFF;
  }
}

void VevorDriver::build_frame(uint8_t frame[16], uint8_t command, uint8_t requested_state) const {
  frame[0] = 0xAA;  // start marker
  frame[1] = 0x66;  // controller id
  frame[2] = command;
  frame[3] = 0x0B;  // controller frame length field
  frame[4] = 0x00;
  frame[5] = 0x00;
  frame[6] = 0x00;
  frame[7] = 0x00;

  // Byte 8: power level 1-10, from the normalized fuel request.
  // Altitude / air-density derating (Vevor frames have no altitude field -
  // unlike BDAP where the heater self-derates - so the host must cut fuel
  // itself). At altitude there is less air per stroke: uncorrected fuel runs
  // rich (soot, HX fouling, overheating). Derate by the ISA density ratio
  // sigma(h), applied only at stable combustion so ignition keeps its full
  // requested rate.
  float output = inputs_.output_fraction;
  if (telemetry_.run_state == RUN_STATE_RUNNING) {
    output = output_domain::clamp01(output * air_density_ratio(altitude_));
  }
  frame[8] = output_domain::level_from_normalized(output);

  frame[9] = requested_state;

  // Experimental fan-speed probe. The documented Vevor 2kW protocol has NO
  // fan command: the frame carries only power level (byte 8) and requested
  // state (byte 9), and every live capture shows bytes 10-14 as 0x00.
  // When fan_probe_rpm_ > 0 we write it as a BE uint16 into bytes 10-11,
  // mirroring the RX fan encoding (uint16 RPM at RX 28-29), to test whether
  // the ECM accepts a fan target. 0 = disabled: protocol-default bytes.
  // NOTE: bytes 10-11 MUST be zeroed explicitly when the probe is disabled,
  // otherwise uninitialised stack garbage goes onto the bus (observed 0xFB 0x3F).
  frame[10] = 0x00;
  frame[11] = 0x00;
  if (fan_probe_rpm_ > 0) {
    frame[10] = (fan_probe_rpm_ >> 8) & 0xFF;
    frame[11] = fan_probe_rpm_ & 0xFF;
  }

  frame[12] = 0x00;
  frame[13] = 0x00;
  frame[14] = 0x00;
  frame[15] = additive_checksum(frame, 16);  // additive checksum, last byte
}

bool VevorDriver::parse_frame(const uint8_t *frame, size_t length) {
  if (length < 52 || length > 60) {
    HEATER_LOGW(TAG, "VEVOR: unexpected frame length %d (expected 52-60)", (int) length);
    return false;
  }

  // Additive checksum over bytes 2..len-2. Log mismatch but keep parsing - the
  // bus is noisy and the reference implementations accept bad checksums too.
  const uint8_t received_csum = frame[length - 1];
  const uint8_t calculated_csum = additive_checksum(frame, length);
  if (received_csum != calculated_csum) {
    HEATER_LOGD(TAG, "VEVOR: checksum mismatch: rx 0x%02X, calc 0x%02X", received_csum, calculated_csum);
  }

  // Map Vevor state (byte 5) onto the shared run-state enumeration
  //   0=off, 1=glow, 2=ignited, 3=stable, 4=stopping/cooling
  const uint8_t vs = frame[5];
  switch (vs) {
    case 0: telemetry_.run_state = RUN_STATE_OFF; break;
    case 1: telemetry_.run_state = RUN_STATE_GLOW_PREHEAT; break;
    case 2: telemetry_.run_state = RUN_STATE_IGNITED; break;
    case 3: telemetry_.run_state = RUN_STATE_RUNNING; break;
    case 4: telemetry_.run_state = RUN_STATE_COOLDOWN; break;
    default: telemetry_.run_state = RUN_STATE_OFF; break;
  }
  telemetry_.on = (vs != 0);

  telemetry_.supply_voltage = frame[11] * 0.1f;  // V x10

  // Byte 15: fan voltage (integer volts, 0-16). Identified from a full-cycle
  // capture (649 frames): 0 when the fan is stopped, 8-11 at running speeds,
  // and the ceiling tracks the unit's own supply rail (ours 11 of 13.2 V; the
  // reference unit 16 of its 15.5 V rail). Fan RPM LAGS it - it is the
  // commanded drive, not a tach echo.
  telemetry_.fan_voltage = frame[15];

  // Bytes 24-27: glow-plug cluster. Non-zero ONLY while the glow is lit
  // (ignition glow and post-run glow), ramping up then zeroing - verified
  // across two glow episodes in the same capture.
  //   byte 27 -> glow current: range 4-12, matching the reference unit's
  //              glow-current byte (0-11 A during exactly these phases).
  //              CANDIDATE, not clamp-verified.
  //   bytes 24-26 -> ramp together into the 80-100 range; voltage vs
  //              temperature is NOT separated, so they are published RAW and
  //              unit-less.
  // NOTE: byte 13 is deliberately NOT decoded any more. On the XMZ-D2 it reads
  // 0 through the whole glow-preheat phase (glow definitively on) and 78-128
  // while running with the glow off, with only {0,78,88,98,108,118,128} ever
  // observed - so it is not glow current on this revision. See ISSUES.md #5.
  telemetry_.glow_plug_current = frame[27];
  telemetry_.glow_a = frame[24];
  telemetry_.glow_b = frame[25];
  telemetry_.glow_c = frame[26];

  // Byte 6: the level the ECM is actually running at. Feedback for the HEAT
  // start-at-max-then-ramp-down behaviour: the Power Level *dial* is the
  // user's request, this is what the heater is doing.
  telemetry_.power_level_actual = frame[6];

  telemetry_.hx_temp = static_cast<int16_t>((frame[16] << 8) | frame[17]) * 0.1f;  // int16, x10
  telemetry_.pump_hz = frame[23] * 0.1f;                                            // Hz x10
  telemetry_.fan_speed = (frame[28] << 8) | frame[29];                             // RPM
  telemetry_.error_code = frame[7];

  // Byte 14: status flags. bit0 = cooling (per reference); this unit sends
  // 0x80 even while idle (unknown meaning) - logged raw for future reverse
  // engineering rather than guessing.
  HEATER_LOGD(TAG,
              "VEVOR RX: State=%d (%s) on=%d, power=%d, V=%.1f, HX=%.0f C, pump=%.1f Hz, fan=%d RPM, "
              "err=%d, status=0x%02X%s",
              telemetry_.run_state, run_state_name(telemetry_.run_state), telemetry_.on, frame[6],
              telemetry_.supply_voltage, telemetry_.hx_temp, telemetry_.pump_hz, telemetry_.fan_speed,
              telemetry_.error_code, frame[14], (frame[14] & 0x01) ? " [COOLING]" : "");
  return true;
}

}  // namespace heater_uart
}  // namespace esphome

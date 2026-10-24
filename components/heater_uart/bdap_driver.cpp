#include "bdap_driver.h"

#include <cmath>
#include <cstring>

#include "heater_log.h"
#include "heater_output.h"

namespace esphome {
namespace heater_uart {

[[maybe_unused]] static const char *TAG = "heater_uart.bdap";

// CRC-16/MODBUS lookup table (moved verbatim from heater_uart.cpp).
static const uint16_t CRC16_TABLE[] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

uint16_t BdapDriver::crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    uint8_t index = data[i] ^ (crc & 0xFF);
    crc = (crc >> 8) ^ CRC16_TABLE[index];
  }
  return crc;
}

// BDAP error codes (moved verbatim from heater_uart.cpp).
static const char *bdap_error_name(int code) {
  switch (code) {
    case 0:  return "No Error";
    case 1:  return "No Error, But Started";
    case 2:  return "Voltage Too Low";
    case 3:  return "Voltage Too High";
    case 4:  return "Ignition Plug Failure";
    case 5:  return "Pump Failure \xE2\x80\x93 Over Current";
    case 6:  return "Too Hot";
    case 7:  return "Motor Failure";
    case 8:  return "Serial Connection Lost";
    case 9:  return "Fire Extinguished";
    case 10: return "Temperature Sensor Failure";
    default: return "Unknown Error Code";
  }
}

void BdapDriver::begin(ByteStream *stream, const HeaterProfile &profile, uint8_t operating_voltage,
                       uint16_t altitude) {
  stream_ = stream;
  profile_ = &profile;
  operating_voltage_ = operating_voltage;
  altitude_ = altitude;
  memset(rx_frame_, 0, sizeof(rx_frame_));
}

const char *BdapDriver::error_description() const { return bdap_error_name(telemetry_.error_code); }

bool BdapDriver::loop(uint32_t now) {
  bool fresh = false;

  // Discard our own TX echo (single-wire bus): during the window right after
  // TX, consume up to 24 bytes. (v1 blocked with delay(15) then discarded;
  // window-based is equivalent without stalling the main loop.)
  if (echo_bytes_left_ > 0) {
    if (now <= echo_until_) {
      while (stream_->available() > 0 && echo_bytes_left_ > 0) {
        stream_->read();
        echo_bytes_left_--;
      }
    } else {
      echo_bytes_left_ = 0;  // window over; anything left is treated as heater RX
    }
  }

  // Collect the heater's response.
  if (awaiting_rx_) {
    while (stream_->available() > 0) {
      const uint8_t byte = static_cast<uint8_t>(stream_->read());
      if (rx_frame_index_ == 0) {
        if (byte == 0x76) {
          rx_frame_[rx_frame_index_++] = byte;
        }
        // else: still looking for sync, discard
      } else {
        rx_frame_[rx_frame_index_++] = byte;
        if (rx_frame_index_ >= 24) {
          fresh = parse_frame(rx_frame_, 24) || fresh;
          awaiting_rx_ = false;
          rx_frame_index_ = 0;
        }
      }
    }
    if (now > rx_timeout_) {
      if (rx_frame_index_ > 0) {
        HEATER_LOGW(TAG, "RX timeout after %d bytes received", rx_frame_index_);
      } else {
        HEATER_LOGW(TAG, "RX timeout - no response from heater");
      }
      awaiting_rx_ = false;
      rx_frame_index_ = 0;
    }
    return fresh;
  }

  // 1 Hz controller cadence.
  if (now - last_tx_time_ >= TX_INTERVAL_MS) {
    const uint8_t command = pending_command_;
    pending_command_ = CMD_NO_CHANGE;  // one-shot: consumed by this build
    uint8_t tx_frame[24];
    build_frame(tx_frame, command);

    HEATER_LOGD(TAG, "TX: %02X %02X %02X %02X %02X %02X %02X %02X ...",
                tx_frame[0], tx_frame[1], tx_frame[2], tx_frame[3],
                tx_frame[4], tx_frame[5], tx_frame[6], tx_frame[7]);
    if (inputs_.priming_active) {
      HEATER_LOGI(TAG, "TX (PRIMING): %02X %02X %02X %02X %02X %02X %02X %02X ... %02X %02X",
                  tx_frame[0], tx_frame[1], tx_frame[2], tx_frame[3], tx_frame[4], tx_frame[5],
                  tx_frame[6], tx_frame[7], tx_frame[16], tx_frame[17]);
    }

    stream_->write_array(tx_frame, 24);
    stream_->flush();

    // Half-duplex single-wire: our own TX comes back as echo. Arm the
    // discard window, then wait for the heater's response.
    last_tx_time_ = now;
    echo_bytes_left_ = 24;
    echo_until_ = now + echo_delay_ms_;
    awaiting_rx_ = true;
    rx_timeout_ = now + RX_TIMEOUT_MS + echo_delay_ms_;
    rx_frame_index_ = 0;
  }
  return fresh;
}

void BdapDriver::build_frame(uint8_t frame[24], uint8_t command) const {
  memset(frame, 0, 24);

  // Byte 0-1: start marker + protocol version
  frame[0] = 0x76;
  frame[1] = 0x16;

  // Byte 2: command
  frame[2] = command;

  // Byte 3: current room temperature (fallback: desired, before first reading)
  frame[3] = (inputs_.room_temp > 0.0f)
                ? static_cast<uint8_t>(inputs_.room_temp)
                : static_cast<uint8_t>(std::round(inputs_.desired_temp));

  // Byte 4: desired temperature
  frame[4] = static_cast<uint8_t>(std::round(inputs_.desired_temp));

  // The controller's fuel request arrives normalized; convert once to the
  // profile's Hz domain and use it for the pump bytes AND the fan ratio.
  const float hz = output_domain::hz_from_normalized(inputs_.output_fraction, *profile_);

  // Bytes 5-6: pump frequency (0.1 Hz units; min = max = fixed Hz mode).
  // During priming, override with the profile's prime rate to run the pump
  // without starting the heater.
  // lround (not truncation): a stored 0.8f is really 0.79999998... and must
  // still encode as 0x08, never 0x07.
  const uint8_t pump_freq_raw =
      inputs_.priming_active
          ? static_cast<uint8_t>(std::lround(profile_->pump_prime_freq * 10.0f))
          : static_cast<uint8_t>(std::lround(hz * 10.0f));
  frame[5] = pump_freq_raw;
  frame[6] = pump_freq_raw;

  // Bytes 7-10: fan RPM (min = max = fixed RPM mode).
  // NOTE: control policy, moved verbatim from the old build_tx_frame - moves
  // into SafetyPolicy at Phase 3.
  uint16_t fan_rpm;
  if (inputs_.cooldown_active || telemetry_.run_state == RUN_STATE_COOLDOWN) {
    // Cooldown: run fan at high speed to cool the heat exchanger
    fan_rpm = profile_->cooldown_fan_rpm;
  } else if (telemetry_.run_state == RUN_STATE_IGNITED &&
             telemetry_.hx_temp < profile_->ignition_hx_threshold) {
    // State 4 (Ignited) with HX below threshold: hold moderate fan speed for
    // stable combustion while the pump ramps
    fan_rpm = profile_->ignition_fan_rpm;
    HEATER_LOGD(TAG, "Ignition: HX %.0f°C < %.0f°C, fan at ignition speed %u RPM",
                telemetry_.hx_temp, profile_->ignition_hx_threshold, fan_rpm);
  } else {
    // Normal operation: scale fan proportionally with fuel rate to keep
    // combustion air matched to fuel.
    const float pump_ratio = output_domain::clamp01(
        (hz - profile_->pump_freq_min) / (profile_->pump_freq_max - profile_->pump_freq_min));
    fan_rpm = static_cast<uint16_t>(profile_->fan_rpm_min +
                                    pump_ratio * (profile_->fan_rpm_max - profile_->fan_rpm_min));
  }
  frame[7] = (fan_rpm >> 8) & 0xFF;
  frame[8] = fan_rpm & 0xFF;
  frame[9] = (fan_rpm >> 8) & 0xFF;
  frame[10] = fan_rpm & 0xFF;

  // Byte 11: operating voltage (0x78 = 12 V, 0xF0 = 24 V)
  frame[11] = operating_voltage_;

  // Byte 12: fan sensor type; byte 13: glow plug power (profile constants)
  frame[12] = profile_->fan_sensor;
  frame[13] = profile_->glow_power;

  // Bytes 14-15: reserved

  // Bytes 16-17: manual pump priming (0x5A = priming mode per protocol spec)
  if (inputs_.priming_active) {
    frame[16] = static_cast<uint8_t>(std::lround(profile_->pump_prime_freq * 10.0f));
    frame[17] = 0x5A;
  }

  // Bytes 18-19: unknown/reserved

  // Bytes 20-21: altitude, big endian (the BDAP heater self-derates from this)
  frame[20] = (altitude_ >> 8) & 0xFF;
  frame[21] = altitude_ & 0xFF;

  // Bytes 22-23: CRC-16/MODBUS over bytes 0-21, MSB first
  const uint16_t crc = crc16(frame, 22);
  frame[22] = (crc >> 8) & 0xFF;
  frame[23] = crc & 0xFF;
}

bool BdapDriver::parse_frame(const uint8_t *frame, size_t length) {
  if (length != 24) {
    HEATER_LOGW(TAG, "Invalid RX frame length: %d bytes (expected 24)", (int) length);
    return false;
  }

  // Validate CRC - log mismatch but keep parsing (some heaters have CRC issues)
  const uint16_t received_crc = (frame[22] << 8) | frame[23];
  const uint16_t calculated_crc = crc16(frame, 22);
  if (received_crc != calculated_crc) {
    HEATER_LOGW(TAG, "RX CRC mismatch: received 0x%04X, calculated 0x%04X", received_crc, calculated_crc);
  }

  // Byte 0: 0x76 start marker; byte 1: protocol version
  // Byte 2: run state; byte 3: on/off state (1 = on)
  telemetry_.run_state = frame[2];
  telemetry_.on = frame[3] == 1;

  // Bytes 4-5: supply voltage (big endian, 0.1 V units)
  telemetry_.supply_voltage = ((frame[4] << 8) | frame[5]) * 0.1f;

  // Bytes 6-7: fan speed RPM (big endian)
  telemetry_.fan_speed = (frame[6] << 8) | frame[7];

  // Bytes 8-9: fan voltage (big endian, 0.1 V units)
  telemetry_.fan_voltage = ((frame[8] << 8) | frame[9]) * 0.1f;

  // Bytes 10-11: heat exchanger temperature (big endian)
  telemetry_.hx_temp = static_cast<float>((frame[10] << 8) | frame[11]);

  // Bytes 12-13: glow plug voltage (big endian, 0.1 V units)
  telemetry_.glow_plug_voltage = ((frame[12] << 8) | frame[13]) * 0.1f;

  // Bytes 14-15: glow plug current (big endian, 0.01 A units)
  telemetry_.glow_plug_current = ((frame[14] << 8) | frame[15]) * 0.01f;

  // Byte 16: pump frequency (0.1 Hz units)
  telemetry_.pump_hz = frame[16] * 0.1f;

  // Byte 17: error code
  telemetry_.error_code = frame[17];

  HEATER_LOGD(TAG, "RX: State=%d (%s), OnOff=%d, Fan=%d RPM, HX=%.0f°C, Pump=%.1fHz",
              telemetry_.run_state, run_state_name(telemetry_.run_state), telemetry_.on,
              telemetry_.fan_speed, telemetry_.hx_temp, telemetry_.pump_hz);
  return true;
}

}  // namespace heater_uart
}  // namespace esphome

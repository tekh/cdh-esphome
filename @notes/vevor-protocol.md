# Vevor diesel heater communication — protocol reference & interface guide

Status: **interfacing phase** (2026-09-02). The comms layer is implemented in
`components/heater_uart/` and bench-verification is in progress.

## The short version

The Vevor 2 kW (XMZ-D2) heater does **not** speak the classic Chinese diesel heater
(BDAP) protocol that the 8 kW Jeabong used. The symptoms seen before this work —
ESP transmitting perfect BDAP frames (`76 16 …` @ 25000 baud) with `RX timeout - no
response from heater` — were caused by a **protocol mismatch**, not by wiring:

| | BDAP (Jeabong 8kW) | Vevor (XMZ-D2 2kW) |
|---|---|---|
| Baud | 25000 | **4800** |
| Start / IDs | `0x76 0x16` | `0xAA 0x66` (controller) / `0xAA 0x77` (heater) |
| Checksum | CRC-16/MODBUS (bytes 0-21) | **additive sum of bytes 2..len-2, mod 256** |
| Control | pump Hz + fan RPM | **power level 1-10** |
| Response | 24 bytes | **56 bytes** (byte 3 = `0x33`) |
| Device-ID echoes | none | controller echoes its own frame (filter on `0x66`) |

Protocol reverse-engineered independently by three parties; our implementation
follows zatakon's working ESPHome component (see References).

## Hardware

- Physical link: same 3-wire harness as BDAP — **black = GND, red = +5 V, blue = data**.
- The bus is **open-drain, idle ≈ 4-5 V, noisy**, half-duplex, and logicsigned inverted.
- Solid-state reference interface (from zatakon): an NPN/PNP transistor pair +
  a pull-up resistor between the data line and **5 V**; the RX transistor connects to
  the **3.3 V rail** (not 5 V!), and both ESPHome UART pins get `inverted: true`.
  **Do not connect ESP32 GPIOs directly to the bus** — it is not 5 V tolerant.
- The original 8 kW-era bench circuit (1 k Ω in series with TX, RX on the same node)
  may work for a first read; test with `inverted: false` first, then `true`, then
  build the transistor stage (see Troubleshooting).
- If the Vevor's original controller/display pod is still attached to the bus it will
  fight the ESP for the line — unplug it during bench tests.

## Protocol

### Link layer

- 4800 baud, 8N1, half-duplex. The controller is always the talker; the heater only
  answers a valid frame (it stays silent otherwise — never announces).
- Cadence: 1 Hz while controlling; slower idle poll is acceptable when off
  (reference lib polls every 60 s when idle; our component currently keeps 1 Hz).
- Frames start at `0xAA`. Byte 3 carries a length field: `0x0B` = controller frame
  (16 bytes), `0x33` = heater frame (56 bytes). Frame end = checksum byte.
- Checksum: `sum(frame[2 .. len-2]) & 0xFF` in the last byte. The reference
  implementations accept frames even when the checksum is bad (noisy bus) — we log
  mismatches rather than dropping frames.

### Controller → heater (16 bytes)

| Byte | Value | Meaning |
|---|---|---|
| 0 | `0xAA` | start marker |
| 1 | `0x66` | controller id |
| 2 | `0x02` / `0x06` | command: status request / start-or-stop |
| 3 | `0x0B` | length field |
| 4-7 | `0x00` | unknown |
| 8 | 1-10 | power level |
| 9 | `0x02`/`0x05`/`0x06`/`0x08` | requested state: off / cooling / start / running |
| 10-14 | `0x00` | unknown |
| 15 | csum | additive checksum |

Command/state selection (as implemented in `send_vevor_frame()`):
- start: `cmd 0x06, state 0x06` (re-asserted every second while off and wanted on)
- stop: `cmd 0x06, state 0x05`
- running status: `cmd 0x02, state 0x08`
- idle status: `cmd 0x02, state 0x02`

### Heater → controller (56 bytes)

| Byte | Value | Meaning |
|---|---|---|
| 0 | `0xAA` | start marker |
| 1 | `0x77` | heater id |
| 2 | `0x02` | command |
| 3 | `0x33` | length field |
| 5 | 0-4 | state: 0 off, 1 glow pre-heat, 2 ignited, 3 stable combustion, 4 stopping/cooling |
| 6 | 1-10 | actual power level |
| 7 | 0x00-0x09 | error (see table below) |
| 11 | ×0.1 | input voltage (V) |
| 13 | | glow plug current (A) |
| 14 | 0/1 | cooling-down flag |
| 16-17 | int16 ×0.1 | heat exchanger temperature (°C, signed) |
| 20-21 | uint16 | state duration (s) |
| 23 | ×0.1 | pump frequency (Hz) |
| 28-29 | uint16 | fan speed (RPM) |
| 46-49 | const | `35 04 11 23`-ish constants |
| 55 | csum | additive checksum |

Error codes (byte 7): `0x01` E10 startup fail, `0x02` E08 fuel level, `0x03` E01
overvoltage, `0x04` E04 (glow circuit), `0x05` E05, `0x06` E04 pump failure,
`0x07` E06 fan failure, `0x08` E03 ignition, `0x09` E05 overheat.

### Power levels, not pump Hz

The heater is commanded in **power levels 1-10**; there is no direct pump-Hz or
fan-RPM control. Our component keeps the shared thermostat logic in the Hz domain
(`pump_freq_setting_`) and converts at the protocol boundary:

```
level = 1 + round((hz - pump_freq_min) / (pump_freq_max - pump_freq_min) * 9)   // clamped 1..10
```

Conversely the reported pump frequency (byte 23 ÷ 10) feeds the pump frequency
sensor. Effectively: level 1 ≈ 0.5 Hz and level 10 ≈ 2.5 Hz on the 2 kW profile.

## Implementation map (cdh-esphome)

| File | Symbol | Role |
|---|---|---|
| `heater_profile.h` | `HeaterProtocol` | `BDAP` or `VEVOR_UART`, plus baud/inverted hints per profile |
| `heater_profile.h` | `profile_vevor_2kw()` | Vevor preset: protocol VEVOR_UART, 4800 baud, inverted, pump 0.5-2.5 Hz, fan 900-3000 RPM, HX critical 205°C (**provisional**) |
| `heater_uart.cpp` | `standalone_loop()` | dispatches to `vevor_loop()` when profile protocol is VEVOR_UART |
| `heater_uart.cpp` | `vevor_loop()` | RX collection (sync `0xAA`, length from byte 3, echo `0x66` filtered), 1 Hz TX cadence, ambient-limit check |
| `heater_uart.cpp` | `send_vevor_frame()` | 16-byte `AA 66` frame; start/stop/status state machine; Hz→level |
| `heater_uart.cpp` | `parse_vevor_rx_frame()` | 56-byte `AA 77` parse; state remap to BDAP enum; voltage/HX/pump/fan/errors; cooldown; then `thermostat_control()` |
| `heater_uart.cpp` | `thermostat_control()` | shared thermostat/safety/auto-restart (BDAP path unchanged, moved verbatim) |
| `heater_uart.cpp` | `calc_checksum()` | additive checksum |
| `heater_uart.cpp` | `vevor_error_map` | byte 7 → E01..E10 strings |
| `start_priming()` | | refuses on VEVOR_UART (no documented prime command) |

BDAP path (Jeabong) is untouched; `thermostat_control()` is the only shared code.

## Bench procedure (safe first test)

1. `lab-heater-butler.yaml`: `heater_model: vevor_2kw`, `baud_rate: 4800` (done).
   Leave the mode select at **Off** — the firmware then only sends idle status
   requests (`cmd 0x02, state 0x02`); the heater stays off.
2. Unplug the Vevor's original controller pod. Flash. Watch logs (DEBUG level).
3. Expect, every second:

```
VEVOR TX: AA 66 02 0B 00 00 00 00 | 03 02 00 00 00 00 00 12
VEVOR RX: State=0 (Off / Standby) on=0, power=1, V=13.4, HX=.. C, pump=.. Hz, fan=.. RPM, err=0
```

   (level 3 = 1.0 Hz ignition rate at idle poll; checksum `12` = `02+0B+03+02`.
   The `uart.debug:` sniff block logs the raw bytes both directions.)
4. Success = steady `VEVOR RX:` lines. Then: set a target temp, switch mode to
   Auto/Heat, watch states 1→2→3 and HX/pump/fan climb, test stop → state 4 cooling.

## Troubleshooting ladder

1. **No RX at all** → add `inverted: true` to both UART pins (transistor circuit) —
   or remove it (direct 1 k Ω tap). Only one of the two will see valid bytes.
2. **Still silent** → check the blue wire with a scope/logic analyser:
   - a 210 µs bit-time UART burst = 4800 baud Vevor UART → wiring problem, build the
     transistor + 5 V pull-up stage (RX transistor to 3.3 V rail).
   - **12 ms pulses** instead of UART = this unit is the **ZM-series PWM variant**
     (e.g. ZM8001) — different protocol entirely (RMT-based, 8 bits TX / 16 bits RX),
     not supported by this component; stop, and we pivot.
3. **RX present but "checksum mismatch" logged** → normal on a noisy bus; frames
   still parse.
4. **Frames interleaved/garbled** → the original controller pod is probably still
   attached; unplug it.
5. **ESP32 running hot / no boot** → a GPIO is directly exposed to the ~5 V bus;
   build the transistor interface before any further testing.

## Variants NOT supported by this protocol layer

- Newer Vevor **Bluetooth/CO-sensor** models (BLE or "AA66 encrypted" packets).
- **ZM-series** units (PWM 12 ms-bit protocol).
- Any heater that does not answer `0xAA 0x66` status requests at 4800 baud.

## References

- zatakon / esphome-vevor-heater — working ESPHome component (our TX state machine,
  parse layout and checksum follow it): `github.com/zatakon/esphome-vevor-heater`
- zatakon / vevor_heater_control — full byte-level RE tables + hardware notes:
  `github.com/zatakon/vevor_heater_control`
- Alexander Krause, "VEVOR diesel heater protocol" (Hackaday.io project 195170) —
  independent 4800 baud / `AA 66 02` confirmation, checksum = sum of bytes after [1]
- Afterburner (Ray Jones) — V9 protocol PDF in this folder (`@notes/`) for the
  BDAP side, and its wiki photos of *incompatible* controllers
- This repo's `@notes/bluetoothheater-master/` — reference firmware (read-only)
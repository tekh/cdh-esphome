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
| Response | 24 bytes | **56 or 57 bytes** (byte 3 `0x33` → 56, `0x34` → 57 — the XMZ-D2 2 kW unit uses `0x34`) |
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
- Frames start at `0xAA`. Byte 3 carries a length field: frame length = **byte 3 + 5**
  (observed: `0x0B` → 16-byte controller frame, `0x33` → 56-byte heater frame,
  `0x34` → 57-byte heater frame). Frame end = checksum byte.
- Checksum: `sum(frame[2 .. len-2]) & 0xFF` in the last byte. **Verified live** against
  the XMZ-D2 2 kW: `sum(bytes 2..55) = 0x58` matches the received checksum byte.
  Mismatches are logged, not fatal (noisy bus, same as the reference implementation).

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
- stop: `cmd 0x06, state 0x05` — **must be re-asserted every second until the heater
  enters state 0 (off) or 4 (stopping/cooling)**; a one-shot stop is ignored (live
  finding: heater just drops to minimum power and keeps running). Once the state
  machine moves, idle status (`0x02/0x02`) resumes.
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
| `heater_uart.cpp` | `parse_vevor_rx_frame()` | `AA 77` frame parse (56/57 bytes, length byte + 5); state remap to BDAP enum; voltage/HX/pump/fan/errors; cooldown; then `thermostat_control()` |
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

## Live observations (XMZ-D2 2kW, first controlled run, 2026-09-02)

Confirmed field decodes (via controlled ignition through this component):

| Field | Observed | Meaning confirmed |
|---|---|---|
| byte 5 (state) | `0x02` while heating up | raw 2 = ignited (zatakon mapping OK) |
| byte 6 (power) | echoes our requested level (3 idle → 5 running) | level byte confirmed both directions |
| byte 11 (voltage) | `0x77` = 119 → 11.9 V | V ×10, reports ~0 while off, live under load |
| bytes 16-17 (HX) | `0x0104` = 260 → 26.0 °C | signed int16 ×10 confirmed |
| bytes 20-21 | state-duration s; resets on state change, ticks 1/s | "state duration" interpretation |
| bytes 28-29 (fan) | `0x05D2` = 1490 RPM | confirmed |
| checksum | tracks changing bytes frame-to-frame | verified live |

Open questions (this unit): bytes 12-14 (`22 A6 80`, middle byte drifts `0x74..0xA6`)
are **not** glow current on this unit (would read as nonsensical 116-166 A; parser now
keeps that sensor at 0); glow data lives elsewhere/undecoded. Tail bytes ~33-55 carry
a second ticking counter and several near-constant temperature-looking readings
(`23 12 21 17 01 1B`) — undecoded.

### Protocol vocabulary is closed (reference capture, zatakon log_start_running_stop.txt)

487 controller frames from a full start→running→stop session use exactly 6 combos:
`02/02` idle, `02/08` running status, `06/06` start, `06/05` stop, power 1-10.
**No prime command exists** - priming is not a protocol operation; the pump
self-primes during every normal start (glow phase). The Fuel Prime button refusal
is correct behaviour, not a gap. Glow plug voltage is **not reported** in the frame;
glow plug **current** is byte 13 on the reference Vevor (their capture: 6-11 A
(`0x06..0x0B`) during states 2 and 4 - ignition warm-up and post-run glow - 0
otherwise; state-1 glow frames are scarce), but the XMZ-D2 puts 116-166 (`0x74..0xA6`)
in that slot at all times, so the parser sanity-gates it (publishes ≤ 40 A, logs
outliers). Bytes 8-9 = `03 FB` are generic "on" flags (0x00 when off).

Additional reference confirmations: RX byte 14 **bit0 = cooling** (STOPPING frame
shows `14:01`; the XMZ-D2 adds bit7 `0x80` as a status flag); the stop phase lights
up a telemetry cluster (byte 13 = `08`, bytes 24-27 = `03 05 03 08`, byte 53 = `14`)
of undecoded meaning. Reference unit frames: 0x33/56 bytes (XMZ-D2: 0x34/57).

### State-3 / full-power data (same first run)

- Stable combustion (~4 min after ignition): **level 10 (max)** → pump **2.4-2.5 Hz**
  (`0x18`/`0x19` — pump byte confirmed, it populated before state 3 too), fan
  **~3500-4200 RPM** (0x0D97 / 0x1066), HX plateau **~145-160 °C** (0x05AE = 145.4),
  supply **12.9-13.1 V** (0x82/0x83; 0x77 = 11.9 V during glow sag), err 0.
- byte 4 = `0x01` while running (0 while off) - matches reference "all 1s when on".
- Profile feedback: fan_rpm_max 3000 underestimates reality (~4200 observed, though
  the heater self-manages fan on this protocol - value is informational); the
  provisional hx_temp_low=145 collides with low-power desire (soot override forced
  pump up while HX < 145 at the approach boundary) - candidate for lowering to
  ~110-120 after an idle/low-power data point; HX critical 205 untouched.
- Cooldown (mode Off): expect STOP frame (06/05), heater self-manages purge, our
  cooldown-complete log fires when HX <= cooldown_target_temp (55 provisional).

## Altitude / air-density fueling

The Vevor frame has **no altitude field** (BDAP heaters receive altitude in-frame and
self-derate — see `build_tx_frame()` bytes 20-21; the Vevor must be derated by the host).
Implemented (option A, research-recommended): open-loop derating at the TX boundary in
`send_vevor_frame()` — the level sent is scaled by the ISA air-density ratio
σ(h) = (1 − 0.0065·h/288.15)^4.2561, applied only at stable combustion
(`RUN_STATE_RUNNING`), so ignition keeps its full requested rate.

| Altitude | σ = ρ/ρ₀ | fuel derate |
|---|---|---|
| 500 m | 0.953 | −4.7 % |
| 1000 m | 0.907 | −9.3 % |
| 1500 m | 0.864 | −13.6 % |
| 2000 m | 0.822 | −17.8 % |

The bench is at **762 m** → σ ≈ 0.929 (−7.1 %), i.e. at level 10 the unit now commands
level 9 (~7 % less fuel). Source of σ: the existing `altitude:` YAML (m); planned
upgrade: a BME280 baro sensor feeding live σ = (P/101325)·(288.15/(T+273.15)) with
`altitude_` as fallback. The HX soot-safety loop is deliberately left as a reactive
backstop, not the primary mechanism (it would fight derating).

## The 1.6 Hz / ~3200 RPM floor — and why no fan control exists (2026-10-12)

Observed: with `Pump Frequency Control` at 0.5 Hz the heater reports pump
1.6 Hz, fan ~3200 RPM, level 1. The ECM appears to "ignore" anything below
1.6 Hz.

**It isn't.** The protocol only carries power levels 1-10 (TX byte 8); the
0.5-2.5 Hz number is this component's facade mapped linearly to levels
(`send_vevor_frame()`). Level 1 is the lowest commandable state, and the ECM
runs level 1 at pump ≈1.6 Hz / fan ≈3200 RPM. There is no sub-level-1
command. Verified sources:

- zatakon/vevor_heater_control RE table: controller→heater frame has only
  level (byte 8) + requested state (byte 9) with non-zero values; bytes 4-7
  and 10-14 observed 0x00 in **every** captured state (start/running/stop).
- zatakon/esphome-vevor-heater `buildSendFrame()`: same, unknowns hard-zeroed.
- Fan speed is RX-only (uint16 RPM, RX bytes 28-29) — the ECM sets fan from
  level; the protocol has no fan command field at all (unlike BDAP, where
  fan RPM is a real TX field).

**Experiment: fan-speed probe (undocumented bytes 10-11) — RESULT (2026-10-12).**
Added `heater_fan_probe_number` (entity `number.lab_heater_fan_speed_probe`,
0 = disabled / protocol-default bytes; >0 = BE uint16 RPM written into TX
bytes 10-11, mirroring the RX fan encoding) and ran the probe at the
level-1 floor:

| step (RPM) | TX bytes 10-11 | pump | fan | HX | error |
|---|---|---|---|---|---|
| baseline | 00 00 | 1.6 Hz | 3201 | 160.8 | No Error |
| 2400 | 09 60 (verified on wire) | 1.6 Hz | 3205 | 157.5 | No Error |
| 1900 | 07 6C | 1.6 Hz | 3217 | 156.7 | No Error |
| 1500 | 05 DC | 1.6 Hz | 3193 | 156.9 | No Error |
| off | 00 00 | — (cooldown) | 2501 (purge) | 87↓ | No Error |

**Verdict: the ECM ignores TX bytes 10-11.** Pump stayed at exactly 1.6 Hz
(sensor `last_changed` never moved during the 8 min experiment), fan within
its normal ±30 RPM wander, zero errors. Combined with the level-1 floor and
the constant pump speed (~1.5-1.6 Hz at level 1 AND level 10 — see below),
the conclusion is firm: **there is no protocol path below 1.6 Hz on this
heater.** The 1.6 Hz pump is a near-constant baseline; the power level
modulates fuel output per stroke, not pump speed. If sub-floor heat output
is ever wanted, the only lever is duty-cycling level 1 (intermittent
run/off) in the component's thermostat — not the wire protocol.

Probe entity kept (harmless at 0, useful if a future Vevor generation or
better RE reveals what bytes 10-14 are). Default is 0 = protocol-default
bytes.

**Bug found during the experiment (fixed + re-flashed same day):** the first
build left TX bytes 10-11 uninitialised on the stack when the probe was 0 —
raw wire capture showed garbage (FB 3F / FC 3F / FF FF) every 5 s (correlated
with the pump-adjustment tick; floats like 0x3FFC0000 = 1.0f), with valid
checksums. The ECM ignored them (heated normally), but the fix is explicit
zero-init of frame[10]/[11] before the probe write; post-fix capture shows
clean `00 00`.

**Bonus finding — pump speed is level-independent (live data):** during
ignition (state 4, level 10) the pump dosed at 1.5 Hz and in stable
combustion (state 5, level 1) at 1.6 Hz. The ECM holds pump speed near
1.5-1.6 Hz across the whole level range; fan moves with state/level
(~1500 ignition → ~3200 at level 1 steady → ~2500 cooldown purge). The
"pump frequency" in the RX frame is therefore a poor proxy for output;
power level + HX temp are the real operating points. The profile's
`hx_temp_low` = 145 soot override still fights low-power mode (it forced
pump to level 10 until HX > 145°C this run) — the earlier note's proposal
to lower it to ~110-120 remains open.

Method used: mode→Heat, pump_control 0.5 (level 1), wait for state 5 +
pump at floor (HX > 145°C so the soot override releases), then probe
2400→1900→1500 RPM with 60-90 s samples and a raw TX capture via
`esphome logs` (proves the probe bytes are actually on the wire before
interpreting a negative). Restore: probe 0, mode Off.

## Deploy path (lab-heater)

Repo (this dir's parent) is source of truth; hoas holds a copy:
`/homeassistant/esphome/lab-heater-butler.yaml` +
`/homeassistant/esphome/components/heater_uart/`. Compile/upload inside the
HAOS addon container:

```bash
ssh hoas 'docker exec -w /data app_5c53de3b_esphome /usr/local/bin/esphome compile /config/esphome/lab-heater-butler.yaml'
ssh hoas 'docker exec -w /data app_5c53de3b_esphome /usr/local/bin/esphome upload  /config/esphome/lab-heater-butler.yaml'
```

Workdir `/data` (build cache `/data/build/lab-heater`), HA config mounted at
`/config`. Device: lab-heater.iot.stmartin.one (10.12.30.157).

## vevor_2kw_v2 profile — user-owned HEAT (2026-10-12)

`heater_model: vevor_2kw_v2` = identical hardware mapping to v1, but in HEAT
mode the firmware sends the user's level verbatim in state 5: no approach
control, no HX-soot pump override. AUTO is unchanged. Still active in v2 HEAT
(protection, not tuning): ignition warm-up ramp (state 4), HX-too-hot pump
decrease, HX-critical stop, configurable ambient limit. Revert = one YAML line
(`vevor_2kw` for v1, `jeabong_8kw` for the 8kW BDAP unit) + reflash.

Why it exists: v1's HEAT mode modulated the level under the user (approach
control) and its HX-soot override forced the pump up whenever HX < 145 °C with
a requested reduction — the exact behaviour that fights a low-power setpoint.

Bench-verified on the real unit (HX started cold, ~33 °C):

| check | result |
|---|---|
| state 4 (ignition) | TX level 0x0A, RX power 10, pump 1.7→2.2 Hz, fan 2847→3398 RPM, HX 52→65 → warm-up ramp works |
| state 5, setpoint 0.5 Hz | TX level 0x01, RX power 1, pump 1.6 Hz, fan ~3200, HX 147 → **user's level verbatim** |
| setpoint 1.2 Hz | TX level 0x04, RX power 4, pump 1.8 Hz, fan 3500 → live change honoured |
| back to 0.5 Hz | TX level 0x01, RX power 1, pump 1.6 Hz, fan 3200 |
| pump number entity | held 0.5 through the state-4 ramp → sync fix works (no clobber) |
| overrides | zero "HX soot safety override" lines in state 5 |

### Control surface on v2 (one fuel dial per heater)

HEAT-mode dials for the 2kW device: **target temperature + power level** — and
nothing else. Implemented 2026-10-12:

- new `Power Level` number entity (1-10, step 1) — converts to the profile's Hz
  domain internally (`set_power_level()`), so the thermostat, the facade mapping
  and the TX boundary are unchanged. Verified on the wire with the heater off:
  dial 6 → TX byte8 `0x06`, dial 2 → `0x02`, checksums valid.
- the pump-frequency dial is REMOVED from this device's YAML (it belongs to the
  8kW BDAP device, where the frame really carries pump Hz).
- the fan probe entity is removed from the YAML too (experiment closed: bytes
  ignored). The entity type stays in the component for a future re-test.
- `get_power_level_display()` mirrors the user's setpoint on manual-HEAT
  profiles, otherwise the firmware's live request — so in AUTO the dial moves on
  its own, in v2 HEAT it moves only when the user moves it.
- runtime guards (`has_pump_hz_control` / `has_power_level_control` /
  `has_fan_rpm_control` profile flags) refuse writes for a control the active
  hardware does not have, so the two fuel dials can never both be live even if
  the YAML declares the wrong one.
- pump Hz and fan RPM remain READ-ONLY sensors: the window into what the ECM
  decided, not levers.
- `Fuel Prime` button refuses on Vevor (no documented prime command; the heater
  self-primes during every start) — 8kW only.

**Correction to the earlier "pump speed is level-independent" note:** that was
two points confounded by a state change (level 10 in state 4 vs level 1 in
state 5). With a clean single-state comparison the ECM DOES couple to the
level — fan strongly (3200 RPM at level 1 → 3500 at level 4) and the pump
weakly (1.6 → 1.8 Hz). The 1.6 Hz figure is a level-1 steady-state value, not
a fixed constant.

**Level ↔ façade mapping (measured):** 0.5 Hz → level 1, 1.0 Hz → 3,
1.2 Hz → 4, 2.5 Hz → 10.

**HX equilibrium at level 1 in current room conditions ≈ 154 °C**, i.e. level
1 holds the HX above the 145 °C soot threshold — the override's trigger zone
is not reachable at minimum power here (it mattered mainly for cold-start
transients and for reductions while HX was low).

### Calibration: why a frozen level→output curve is not worth building

Ambient and intake-air conditions move the mapping: cold intake air is denser
(more O2 per stroke), a cold room changes HX heat rejection, so "level 3 =
X kW" measured in October says nothing about January. Durable instead:

1. **Protocol/ECM behaviour facts** — obedience, level↔RX mapping, ramp
   behaviour, pump/fan coupling. Firmware-determined, season-independent.
   This is what the table above records.
2. **Combustion proxy + closed loop** — control on HX temperature (the profile
   already has `hx_temp_target`) rather than on a fixed level; the level then
   becomes whatever holds the target in current conditions, self-correcting
   for ambient, altitude, fuel quality and HX fouling.
3. **Live density correction** — measure intake-air temperature (+ baro) and
   derate with the same σ formula already used for altitude, instead of
   freezing a curve to one season's air.

## Byte-level RE of the glow / fan slots — full ignition cycle (2026-10-12)

Captured our XMZ-D2 across a complete off → glow-preheat → ignited → running →
stopping cycle (649 heater frames over 16:43:31-16:54:28, raw `<<<` sniff,
1 Hz; timestamps verified monotonic so no mixed-writer corruption). Scripts:
`scratch/parse_cycle.py`, `scratch/parse_cycle2.py`; data
`scratch/vevor-cycle-full.log`.

**Fan voltage — IDENTIFIED: byte 15.** Integer, 0-16. 0 = fan stopped; 8-11 at
our running speeds; the maximum tracks the unit's own supply rail (ours peaks 11
of 13.2 V, the reference unit peaked 16 of its 15.5 V). RPM *lags* it — it is the
commanded drive, not a tach echo:

| b15 | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| median RPM | 1503 | 1507 | 1991 | 1998 | 2504 | 2503 | 2823 | 3204 | 3204 | 3490 | 3506 | 3609 |

**Glow cluster — IDENTIFIED as bytes 24-27, non-zero ONLY while the glow is
lit.** Two episodes captured, both ramping smoothly and then zeroing:

| episode | duration | peak b24 / b25 / b26 / b27 | state |
|---|---|---|---|
| ignition glow | ~103 s | 101 / 93 / 83 / 12 | ignited |
| post-run glow | ~40 s | 79 / 89 / 78 / 10 | stopping |

- **b27 is the glow-current candidate**: range 4-12, exactly matching the
  reference unit's glow-current byte (their byte 13: 0-11 A during the same two
  phases).
- **b24/b25/b26 ramp together (0→~100)** and are the voltage/temperature
  candidates; individual assignment is still open (with no amp clamp, they can't
  be separated by magnitude alone).
- The reference unit shows the same cluster (b24-27 zero except in states 2 and
  4) *and* a current value at its byte 13 — so the two revisions expose the same
  physics in different slots.

**byte 13 on our unit is NOT glow current — our parser is wrong here.** Proof:
it is 0 through the whole glow-preheat phase (glow definitively on) and non-zero
(78-128) while running with the glow off. Its distinct values are only
{0, 78, 88, 98, 108, 118, 128} — a 10-step ladder (equivalently 0,-10..-50 in
offset-128, or a ×10 fixed-point of an integer 8..13). Quantity unidentified.
Current parser maps byte 13 → glow current (gated ≤40) → should be replaced by
the b24-27 cluster + byte 15 fan voltage.

**Other revision drift (ours vs the reference unit):**
- byte 52-53: glow-phase values 0-410/420 on the reference, always 0 on ours.
- byte 14: 0x80 constant on ours; 0x00/0x01 (cooling flag) on the reference.
- bytes 8-9: 00/EE on ours; 03/FB on the reference.
- byte 23 (pump Hz ×0.1): 1.6 at level 1 on ours; 3.8-4.0 at level 3 on theirs —
  pump speed *does* move with level on both, more strongly on the reference unit.

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
# Heater UART ESPHome Component

This project is a custom ESPHome component designed to interface with heaters using UART communication. It provides a comprehensive integration for monitoring and controlling various aspects of heater operation, including temperature, fan speed, voltage, and error diagnostics.

## Features

- **Full UART Frame Parsing**: Processes and decodes 48-byte frames from the heater's UART communication.
- **Standalone Control Mode**: ESP32 can act as the heater controller with full thermostat capability.
- **Monitoring Sensors**:
  - Current Temperature
  - Desired Temperature
  - Fan Speed
  - Supply Voltage
  - Heat Exchanger Temperature
  - Glow Plug Voltage
  - Glow Plug Current
  - Pump Frequency
  - Fan Voltage
- **Control Entities**:
  - **Number**: Desired Temperature (8-35°C), Pump Frequency (1.3-5.5 Hz)
  - **Select**: Mode Selection (Off/Auto/On)
- **Binary Sensors**:
  - On/Off State
  - Auto Shutdown Active
  - Standby Active
  - Priming Active
- **Text Sensors**:
  - Run State
  - Error Code
- **Button**:
  - Fuel Prime (60s pump-only run for bleeding air; **BDAP only** - the Vevor
    protocol has no prime command and its ECM self-primes, so the button is a
    no-op on `vevor_2kw_*` and just logs a warning)
- **Automatic Temperature Control**:
  - Auto-shutdown when target temperature is reached
  - Auto-restart when temperature drops below threshold
  - Intelligent approach control to prevent overshoot
- **Mappings for Readability**:
  - Human-readable descriptions for error codes and run states.

## Operational Mode: Standalone (the only mode)

The ESP32 acts as the primary heater controller with full control capabilities:

- **Full Control**: ESP32 sends UART frames to the heater at 1 Hz intervals
- **Thermostat Operation**: Automatic temperature control with intelligent heating/cooling cycles
- **Temperature Sensing**: Uses external temperature sensor (recommended) or heat exchanger temperature as fallback
- **Voltage Support**: Supports both 12V and 24V heaters
- **Multi-Model Support**: Selectable heater profile (see below) - pump/fan/HX limits adapt to the connected heater
- **Safety Features**: Built-in cooldown sequences and heat exchanger temperature protection

```yaml
heater_uart:
  heater_model: vevor_2kw  # required - which heater is connected
  operating_voltage: "12V"  # or "24V"
  temperature_sensor: external_temp_sensor_id  # Optional but recommended
```

> **LCD Controller Mode (passive sniffing of an existing LCD controller ↔ heater bus)
> was removed 2026-10-24** — abandoned before it was ever validated against real
> hardware (no BDAP device target). The last code carrying it lives under the git tag
> `lcd-injection-final` if it is ever needed again.

## Heater Models

The component supports multiple heaters via preset profiles. Every heater-specific tuning
parameter (fuel pump Hz range, combustion fan RPM range, heat exchanger safety thresholds,
glow plug power, temperature range, priming rate) comes from the selected profile - the
control logic itself is heater-agnostic.

The `heater_model` option is **required** and must name the physical heater connected:

```yaml
heater_uart:
  heater_model: jeabong_8kw  # required
```

| `heater_model` | Heater | Protocol | Baud | Pump freq (Hz) | Fan (RPM) | HX critical | Status |
|---|---|---|---|---|---|---|---|
| `jeabong_8kw` | Jeabong 8kW | BDAP `0x76/0x16` | 25000 | 1.3 - 5.5 | 1450 - 4500 | 265 °C | Validated on the original reference hardware |
| `vevor_2kw` | Vevor 2kW (XMZ-D2) | Vevor `0xAA/0x66/0x77` | 4800 | 0.5 - 2.5 | 900 - 3000 | 205 °C | Same hardware as v2; the firmware-modulated HEAT policy is the unexercised part |
| `vevor_2kw_v2` | Vevor 2kW (XMZ-D2), user owns the level in HEAT | Vevor `0xAA/0x66/0x77` | 4800 | 0.5 - 2.5 | 900 - 3000 | 205 °C | **In production** (lab-heater-butler.yaml); bench-verified incl. fault guards; open tuning items in ISSUES.md #5-10 |

> `vevor_2kw` and `vevor_2kw_v2` are the same hardware with identical tuning
> values; the profile differs only in the HEAT-mode policy (`heat_mode_manual`).
> Keep one as a one-line revert target when experimenting with the other. Any
> value that later proves wrong gets fixed in the preset (and can be
> overridden per-deployment in YAML meanwhile — see Tuning Overrides below).

### Capabilities per model

What each heater can actually *do* differs by protocol and policy, not just by
numbers:

| Capability | `jeabong_8kw` | `vevor_2kw` | `vevor_2kw_v2` |
|---|---|---|---|
| Fuel dial | pump frequency (Hz) | power level 1–10 | power level 1–10 |
| Combustion fan dial | fan RPM exists in the protocol (dial not yet implemented — ISSUES #2) | not in the protocol | not in the protocol |
| Fuel Prime button | **works** — 60 s pump-only run (bleeds air) | **no-op** — ECM self-primes on every start; pressing logs a warning | **no-op** — same |
| HEAT mode behaviour | firmware modulates like AUTO | firmware modulates like AUTO | **user-owned level**: sent verbatim, ramps down toward target |
| Combustion-safety authority | the **firmware** — its safety stack is always on (cannot be disabled) | the **ECM** — the firmware's extra HX stack can be disabled via `firmware_safety_oversight: false` | the **ECM** — same opt-out |
| Altitude handling | sent in-frame, ECM derates | open-loop fuel derate at the TX boundary (σ(h)) | open-loop fuel derate (σ(h)) |
| Error reporting | BDAP error byte | E-codes (E01–E10) | E-codes (E01–E10) |
| Validation status | validated on the original hardware (parked, no device target) | same hardware as v2; its unexercised part is the firmware-modulated HEAT policy | **in production** (lab device) |

Safety that applies to **every** model regardless of the above (not
removable by any setting): the room-sensor guard (glitch rejection, boot
consensus, 10-minute staleness cutoff) and the `ambient_heat_limit` shutdown in
HEAT mode. AUTO-mode behaviour is identical across models — only the fuel
dial and the policy flags differ.

### Vevor protocol notes (interfacing phase)

The Vevor bus is **not** the classic BDAP protocol:

- **Baud 4800**, half-duplex, controller talks first at 1 Hz, heater answers with a
  56/57-byte frame (`AA 77 ... 0x33/0x34 ...`; length byte + 5, additive checksum =
  sum of bytes 2..len-2 mod 256 — verified live on the XMZ-D2 2 kW).
- Controller frames are 16 bytes: `AA 66 <cmd> 0x0B ... <power 1-10> <state> ... <csum>`.
  Power is a **level 1-10**, not pump Hz — the component converts pump Hz ↔ level at the
  protocol boundary, so all thermostat logic stays in the Hz domain.
- The bus is open-drain, idle ≈ 4-5 V, **inverted logic**: the reference interface is a
  transistor pair (RX transistor to 3.3 V rail!) plus a pull-up to 5 V, with
  `inverted: true` on both UART pins. **Do not connect ESP32 GPIOs directly to the bus.**
  If your bench uses the direct 1k-ohm tap from the 8 kW era, test with `inverted: false`
  first, then `true`, then build the transistor stage if the line is still silent.
- Implemented per the zatakon reverse engineering
  (`github.com/zatakon/esphome-vevor-heater`): states 0=off, 1=glow, 2=ignited,
  3=stable, 4=stopping/cooling; errors `E01/E03/E04/E05/E06/E08/E10`; HX temperature is a
  signed int16 × 0.1 °C; pump frequency byte = Hz × 10; fan speed = RPM big-endian.
- Supported heaters: Vevor XMZ-D2 2 kW (5 kW also reported). Newer Bluetooth/CO-sensor
  models and the ZM-series (PWM) units use **different** protocols — not supported here.
- **Altitude/air density**: the Vevor frame has no altitude field (BDAP heaters
  self-derate from the altitude bytes in their frame). This component therefore
  derates fuel **open-loop** at the TX boundary: the requested power level is scaled
  by the ISA air-density ratio σ(h) (`1 - 0.0065·h/288.15)^4.2561`), applied only at
  stable combustion so ignition keeps its full rate. `altitude:` (m) is the source;
  a BME280 baro sensor for live σ is the planned upgrade.
- Fuel priming is **not supported** on the Vevor protocol (no documented pump-prime
  command); the Fuel Prime button logs a warning instead.

> 📄 Full protocol byte-layout, hardware notes and the bench/test procedure live in
> [`@notes/vevor-protocol.md`](@notes/vevor-protocol.md).

> Note: the `type: pump` number entity's `min_value`/`max_value` in your YAML should match the
> active profile's pump frequency range (jeabong_8kw: 1.3-5.5, vevor_2kw: 0.5-2.5).

## Adding a New Heater (Heater N)

The architecture is built so that heaters add as *data*, not code. The cost
scales with how much of the hardware is new:

### Same protocol, new model (e.g. a 5 kW Vevor)

One file each, zero policy/thermostat changes:

1. **`components/heater_uart/heater_profile.h`**
   - one new `HeaterModel` enum entry;
   - one preset factory `profile_xxx()` setting **every** field of
     `HeaterProfile` (start from the nearest existing profile and change what
     differs, the way `profile_vevor_2kw_v2()` does);
   - one `case` in `profile_for()`.
2. **`components/heater_uart/__init__.py`** — one line in the `cv.enum`
   `heater_model:` mapping (key must match the YAML name exactly, lower-case).
3. **`tests/test_profiles.cpp`** — add the model to `ALL_MODELS`; the existing
   invariant tests then cover the new preset at `make -C tests` time.
4. **This README** — one row in the heater table above *and* one column in the
   "Capabilities per model" table (dials, priming, HEAT policy, safety
   authority — read from the profile's capability flags, not from memory).

Then, per device YAML:

```yaml
uart:
  baud_rate: <profile.uart_baud_rate>   # must match the profile
  # pin inversion must match profile.uart_inverted (Vevor bus: true)
heater_uart:
  heater_model: <the new key>
```

No recompiles for bench tuning: the Phase-4 **Tuning Overrides** keys cover
the preset, and `validate_profile_invariants()` reverts any override that
would break a profile invariant. Once a value proves itself at the bench, it
gets folded into the preset factory (and the README row).

### New protocol (a bus we have not seen)

Everything above, plus one driver — this is the only step that requires
protocol knowledge:

1. **`HeaterProtocol`**: one new enum value in `heater_profile.h`.
2. **One driver class** implementing `HeaterProtocolDriver`
   (`heater_protocol_driver.h`); follow `bdap_driver.h/.cpp` as the template
   (`begin()`, `loop()`, `request_start()/request_stop()/pulse_stop()`,
   `on_requested()`, `set_output()` taking `ControlInputs`, `telemetry()`,
   run-state + error-code maps, checksums/echo/cadence all in the driver).
   The policy layer speaks only the normalized 0.0-1.0 fuel domain via
   `heater_output.h` — the new driver encodes that into whatever its protocol
   says (Hz, level, duty — no policy changes).
3. **Facade wiring**: one `case` in the driver switch in `heater_uart.cpp`
   `setup()`, and the `.cpp` added to the `files` list in
   `heater_uart.json`.
4. **Host tests** (same commit): frame-layout goldens, TX state-machine
   traces and RX decode rules using `tests/fake_stream.h` — the pattern in
   `tests/test_bdap_driver.cpp` / `test_vevor_driver.cpp`.

Everything else (thermostat, safety stack, room-sensor guard, entities,
priming refusal policy) is protocol-agnostic and already runs the new
profile.

## Auto Mode

When using standalone mode with the mode select set to "Auto", the component provides intelligent thermostat control:

### How Auto Mode Works

1. **Heating Phase**: When room temperature is below target, the heater starts and increases heat output
2. **Approach Control**: As temperature approaches the target, heating rate automatically slows to prevent overshoot
3. **Auto-Shutdown**: When the room crosses target + `auto_shutdown_overshoot` with the pump at its minimum, the heater shuts down and enters cooldown mode (with the lab setting of 0.0 this is the setpoint itself)
4. **Standby Monitoring**: While off, the component continuously monitors room temperature
5. **Auto-Restart**: When temperature drops below the restart threshold, the heater automatically starts again

### Configuration Parameters

Fine-tune the Auto mode behavior with these parameters:

- `auto_shutdown_overshoot` (schema default: 0.5°C, lab-heater: 0.0): Temperature above target before auto-shutdown triggers. 0.0 = shut off as soon as the room crosses the setpoint (2026-10-20: with 0.5 the heater idled at min level indefinitely when the room's min-output equilibrium sat below target+0.5)
- `auto_shutdown_hysteresis` (schema default: 1.5°C, lab-heater: 1.0): Temperature drop below target needed before auto-restart (the re-engage offset)
- `approach_threshold` (default: 1.0°C): Distance from target when approach control begins
- `ambient_heat_limit` (default: 26.0°C): HEAT mode safety limit - forces shutdown if room > X°C (range: -40 to 80°C)
- `firmware_safety_oversight` (default: true): Set `false` ONLY on heaters whose ECM handles combustion safety itself (the Vevor profiles) to let the ECM be the sole safety authority. Ignored with a warning on firmware-safety hardware (the 8kW).

Example configuration:
```yaml
heater_uart:
  auto_shutdown_overshoot: 0.0
  auto_shutdown_hysteresis: 1.0
  approach_threshold: 1.0
  ambient_heat_limit: 26.0  # HEAT mode safety limit (°C)
```

### Tuning Overrides (per-key, over the model preset)

Every tuning value below is optional; anything omitted keeps the selected
model's compiled, bench-validated preset. Combos that would break a profile
invariant (e.g. `pump_freq_min >= pump_freq_max`, or HX thresholds out of
`low < target < high < critical` order) are **reverted to the preset at boot
with a warning** — a typo can never silently disable a safety net.

| Group | Keys | Unit |
|---|---|---|
| Fuel pump | `pump_freq_min`, `pump_freq_max`, `pump_freq_step`, `pump_freq_initial`, `pump_prime_freq` | Hz |
| Combustion fan (BDAP) | `fan_rpm_min`, `fan_rpm_max`, `ignition_fan_rpm`, `cooldown_fan_rpm`, `ignition_hx_threshold` | RPM / °C |
| Heat exchanger | `hx_temp_low`, `hx_temp_target`, `hx_temp_high`, `hx_temp_critical`, `cooldown_target_temp` | °C |
| Frame constants | `fan_sensor`, `glow_power` | protocol byte |
| Setpoint range | `temp_min`, `temp_max` | °C |
| Cadence | `pump_adjust_interval`, `priming_duration` | seconds |
| Aggressiveness | `thermostat_gap_far`, `thermostat_gap_mid`, `thermostat_step_mult_far`, `thermostat_step_mult_mid`, `thermostat_os_big`, `thermostat_os_mid`, `thermostat_os_mult_big`, `thermostat_os_mult_mid` | °C / × |

```yaml
heater_uart:
  heater_model: vevor_2kw
  pump_freq_min: 0.6   # e.g. a soft-running pump refuses 0.5
  pump_adjust_interval: 3   # s - more responsive thermostat
```

Not exposed on purpose: model-identity flags (`heat_mode_manual`,
`ecm_owned_safety`, the `has_*_control` capability flags) — those are the
model, not tuning; protocol timing and framing (drivers); room-sensor physics
guard constants (DHT22 facts, not heater tuning).

### Mode Options

The mode select entity provides three operating modes:

- **Off**: Heater is forced off (enters cooldown if running)
- **Auto**: Thermostat-controlled operation with auto-shutdown and auto-restart
- **On**: Heater is forced on, bypassing thermostat control. Safety limits still apply including the ambient heat limit.

### State after reboot

Settings are **not persistent**: after any reboot (reflash or power loss) the
mode select resets to **Off** and the target temperature to **19.0**
(component defaults, 2026-10-20; the target default was 22.0 before).
Re-select Auto to resume thermostat operation.

### Room-sensor fault cutoff (standalone mode)

If the external room sensor (DHT22) produces no **accepted** reading for
**10 minutes**, the thermostat is considered blind: a running heater is
**stopped immediately (all modes)** and no auto-start can occur until fresh
readings arrive (2026-10-20). A stale last value could otherwise hide an
overheat — the 26°C ambient limit reads that same stale value — and a stale
cold value could trigger a blind start. In Auto mode the heater parks in
standby during the fault and resumes normal thermostat behaviour once the
sensor recovers. Since 2026-10-21 the timer is refreshed only by readings
that pass the glitch guard below, so a sensor that is dead *or* stuck
emitting garbage is declared blind.

### Room-sensor glitch guard (standalone mode)

A DHT22 right after a failed read can emit a sample that is valid but
garbage (observed 2026-10-21: 18.4 °C against a 19.2 °C trend, which tripped
a false AUTO start). Every room-sensor sample now passes a physics guard
before the thermostat or the TX frames see it (2026-10-21):

- NaN / warning readings are rejected outright.
- A non-NaN reading more than **0.5 °C** from the last accepted reading
  (the observed field rate is ~0.3 °C/min, so this is 10× headroom per 10 s
  sensor interval) is treated as a glitch: it is rejected and the last
  trusted value is held.
- Rejected readings do **not** refresh the 10-minute fault-cutoff timer
  above, so a persistently bad sensor is declared blind just like a dead one.

**Boot baseline.** After a reboot, no single sample may become the reference:
the first 3 non-NaN readings (at least 5 s apart, so each is a distinct
10-s DHT22 conversion — ~30 s total) must agree within **2.0 °C**, and the
baseline is the **median** of the three. Until then the sensor counts as
stale, so AUTO keeps the heater off — the safe state right after a reboot,
when the relay has just de-energized. A window that fails the spread check is
discarded and retried with the next three readings. A log line
(`Room temp baseline … established (3-sample consensus, …)`) appears ~30 s
after boot when the baseline is set.

### Half-degree setpoints

The target temperature accepts 0.5°C steps. The thermostat logic runs on the
exact float value (e.g. 19.5) and the display shows it exactly (fixed
2026-10-20: the display previously truncated 19.5 to 19.0). The Vevor TX frame
carries the **rounded integer** °C in byte 4 (protocol constraint); the
heating behaviour follows the exact float target.

## Getting Started

### Installation

Update your ESPHome configuration to include this component.

### Example Configuration

This example shows a complete standalone mode configuration with full control
capabilities. The `uart:` block must match the selected profile - the BDAP
example below uses 25000 baud on one half-duplex pin; for a Vevor model use
`baud_rate: 4800` and the inverted-pin wiring described in the Vevor protocol
notes:

```yaml
external_components:
  - source: github://tekh/cdh-esphome@main
    components: [ heater_uart ]

# Standalone mode uses single-wire half-duplex UART
uart:
  baud_rate: 25000  # jeabong_8kw: 25000 | vevor_2kw(_v2): 4800 (inverted pins)
  tx_pin: GPIO17  # Use same pin for TX and RX
  rx_pin: GPIO17
  id: uart_bus
  rx_buffer_size: 512

heater_uart:
  update_interval: 5s
  id: heater
  heater_model: jeabong_8kw  # required - jeabong_8kw | vevor_2kw | vevor_2kw_v2
  operating_voltage: "12V"  # or "24V" depending on your heater
  altitude: 750  # meters above sea level
  auto_shutdown_overshoot: 0.0  # °C above target before shutdown (0.0 = at the setpoint)
  auto_shutdown_hysteresis: 1.0  # °C drop below target before auto-restart
  approach_threshold: 1.0  # °C from target to start slowing down
  temperature_sensor: room_temp  # External sensor (recommended)

sensor:
  # External temperature sensor (highly recommended for accurate control)
  - platform: dht
    pin: GPIO4
    model: DHT22
    temperature:
      id: room_temp
      name: "Room Temperature"
    humidity:
      name: "Room Humidity"
    update_interval: 10s

  # Heater monitoring sensors
  - platform: heater_uart
    current_temperature:
      name: "Current Temperature"
    desired_temperature:
      name: "Desired Temperature"
    fan_speed:
      name: "Fan Speed"
    supply_voltage:
      name: "Supply Voltage"
    heat_exchanger_temp:
      name: "Heat Exchanger Temperature"
    glow_plug_voltage:
      name: "Glow Plug Voltage"
    glow_plug_current:
      name: "Glow Plug Current"
    pump_frequency:
      name: "Pump Frequency"
    fan_voltage:
      name: "Fan Voltage"

number:
  - platform: heater_uart
    desired_temperature:
      name: "Set Temperature"
      id: heater_temp_setting
    pump_frequency:
      name: "Manual Pump Control"
      id: pump_control

select:
  - platform: heater_uart
    mode:
      name: "Heater Mode"
      id: heater_mode

text_sensor:
  - platform: heater_uart
    run_state:
      name: "Run State"
    error_code:
      name: "Error Code"

binary_sensor:
  - platform: heater_uart
    on_off_state:
      name: "On/Off State"
    auto_shutdown_active:
      name: "Auto Shutdown Active"
    standby_active:
      name: "Standby Mode"
    priming_active:
      name: "Fuel Priming Active"

button:
  - platform: heater_uart
    fuel_prime:
      name: "Fuel Prime"   # BDAP only - on vevor_2kw_* the ECM self-primes; the button logs a warning
```

### Notes

**General:**
- The UART baud rate and pin inversion must match the selected profile
  (`profile_.uart_baud_rate` / `profile_.uart_inverted`): 25000 / normal for
  the BDAP 8 kW, 4800 / inverted for the Vevor models.
- Ensure the `rx_pin` and `tx_pin` match your ESP32 or ESP8266 configuration.

**Standalone Mode:**
- Use single-wire half-duplex UART: `tx_pin` and `rx_pin` must be the **same GPIO pin**.
- External temperature sensor is highly recommended for accurate room temperature control.
- Without an external sensor, the component will use heat exchanger temperature as a fallback.
- In Auto mode, the heater will automatically manage itself in the band `desired_temperature - hysteresis` (auto-restart) to `desired_temperature + overshoot` (auto-shutdown; with the lab setting of 0.0 the band is [target-1.0, target]).
- In HEAT mode (On), the ambient heat limit provides additional safety by forcing shutdown if room temperature exceeds the configured threshold.

**Mode Select Usage:**
- **Off**: Forces heater off. Use this when you don't want the heater to run.
- **Auto**: Thermostat mode. Heater will auto-start when cold and auto-shutdown when warm.
- **On**: Forces heater on, bypassing thermostat. Safety limits (heat exchanger temperature) AND ambient heat limit still apply.

## Credits

This project is a heavily modified derivative of the work of the community
around Chinese diesel heater UART control:

- **[daoudeddy/cdh-esphome](https://github.com/daoudeddy/cdh-esphome)** — the
  upstream ESPHome component this codebase started from. The UART frame
  handling, entity structure and standalone thermostat concept are its
  inheritance; everything since the `arch_analysis` refactor (drivers,
policies, profiles, YAML tuning) is built on top of it.
- **[bidon583/bluetoothheater](https://github.com/bidon583/bluetoothheater)** —
  the classic "blue wire" reference implementation; a copy is kept
  read-only under `@notes/bluetoothheater-master/` for byte-layout reference.
- **[zatakon/esphome-vevor-heater](https://github.com/zatakon/esphome-vevor-heater)**
  — reverse engineering of the Vevor `0xAA/0x66/0x77` protocol that the
  `vevor_driver` implements (states, error codes, signed HX temperature).
- **"V9 - Hacking the Chinese Diesel Heater Communications Protocol"** (kept
  in `@notes/`) — the community document for the classic BDAP 0x76/0x16
  protocol.

Protocol evidence gathered on this project's own bench lives in
`@notes/vevor-protocol.md`.

## Development

### Code Structure

**Core component** (`components/heater_uart/`):
- **`heater_profile.h`**: all heater-specific tuning (pump Hz, fan RPM, HX
  thresholds, glow power, temp range) as compiled presets + capability flags,
  plus `validate_profile_invariants()` which reverts YAML overrides that
  break a profile invariant.
- **`heater_uart.cpp/h`**: the facade — entity registry/publishing, mode
  intent, wiring policy results to the driver.
- **`heater_policies.h/.cpp`**: pure, host-tested policies — `Thermostat`
  (AUTO/HEAT state machine incl. mode entry and auto-restart),
  `CombustionSafety` (HX safety stack, BDAP fan matching, cooldown monitor),
  `RoomSensorGuard` (DHT22 physics guard: boot consensus, glitch, staleness).
- **`heater_protocol_driver.h`**: the driver seam — `ByteStream`, `Telemetry`,
  `ControlInputs`, shared run-state names.
- **`bdap_driver.cpp/h`**: BDAP framing, cadence, echo discard, CRC-16,
  frame build/parse, run-state and error maps.
- **`vevor_driver.cpp/h`**: Vevor TX state machine, level encoding + ISA
  altitude derate, RX decode, error map.
- **`heater_output.h`**: the one fuel-domain conversion (normalized 0.0-1.0 ↔
  Hz ↔ level), host-tested.
- **`__init__.py`**: ESPHome schema incl. all Phase-4 tuning override keys.

**Sensor entities:** `sensor.py` (numeric: temperature, voltage, current,
glow cluster), `text_sensor.py` (run state, error code),
`binary_sensor.py` (on/off, auto-shutdown, standby, priming).

**Control entities:** `number.py` (temperature, pump frequency, power level,
fan probe), `select.py` (mode: Off/Auto/Heat), `button.py` (fuel priming,
BDAP-only — the Vevor protocol has no prime command); their C++
implementations are the `heater_*.cpp/h` files.

### Debugging

Enable debug logging in your ESPHome configuration to monitor the UART frames and parsed data:

```yaml
logger:
  level: DEBUG
```

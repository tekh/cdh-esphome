# PROJECT KNOWLEDGE BASE

**Generated:** 2026-04-19 (rewritten for the post-refactor layout 2026-10-24)
**Commit:** 5a9d9c3
**Branch:** main (the `arch_analysis` refactor branch, renamed in; the pre-refactor layout lives on `vevor_2kw`)

## OVERVIEW

ESPHome external component for UART communication with diesel heaters (BDAP 24-byte and Vevor 56/57-byte frames). Monitors/controls temperature, fan speed, voltage, diagnostics. Standalone thermostat mode with auto-shutdown — the only mode since the LCD injection path was removed (recover via git tag `lcd-injection-final`).

## STRUCTURE

```
.
├── components/heater_uart/    # Main component (C++ + Python)
│   ├── heater_uart.cpp/h      # Facade: entities, mode intent, policy/driver wiring
│   ├── heater_profile.h       # ALL heater-specific tuning (presets + capability flags)
│   │                           #   + validate_profile_invariants() (YAML-override repair)
│   ├── heater_protocol_driver.h  # Driver seam: ByteStream, Telemetry, ControlInputs
│   ├── bdap_driver.cpp/h      # BDAP protocol (0x76/0x16 @ 25000, CRC-16)
│   ├── vevor_driver.cpp/h     # Vevor protocol (0xAA/0x66/0x77 @ 4800, level encode + derate)
│   ├── heater_policies.cpp/h  # Pure policies: Thermostat, CombustionSafety, RoomSensorGuard
│   ├── heater_output.h        # The one fuel-domain conversion (0.0-1.0 ↔ Hz ↔ level)
│   ├── heater_log.h           # Log shim (ESP_LOG* on device, no-ops on host tests)
│   ├── __init__.py            # ESPHome schema incl. Phase-4 tuning override keys
│   ├── sensor.py              # 9 numeric sensors
│   ├── text_sensor.py         # 2 text sensors (run_state, error_code)
│   ├── binary_sensor.py       # 4 binary sensors
│   ├── number.py              # Temperature + pump frequency + power level + fan probe
│   ├── select.py              # Mode selection (Off/Auto/Heat)
│   ├── button.py              # Fuel priming button (BDAP-only)
│   ├── heater_number.cpp/h    # Temperature number entity
│   ├── heater_pump_number.cpp/h  # Pump frequency number entity
│   ├── heater_select.cpp/h    # Mode select entity
│   ├── fuel_prime_button.h    # Fuel prime button entity
│   └── heater_uart.json       # ESPHome component manifest (must list every .cpp)
├── tests/                     # Host-side test harness (make -C tests, plain g++)
├── ROADMAP.md                 # Refactor decisions + phase status
├── ISSUES.md                  # Open/parked/decided items with evidence
├── @notes/bluetoothheater-master/  # Reference implementation (read-only)
└── README.md                  # User docs incl. the "Adding a New Heater" checklist
```

## WHERE TO LOOK

| Task | Location | Notes |
|------|----------|-------|
| Add sensor | `sensor.py` + owning driver's `parse_frame()` + facade `update()` | Telemetry field + publish key |
| Modify frame parsing | `bdap_driver.cpp:parse_frame()` / `vevor_driver.cpp:parse_frame()` | Byte positions pinned by `tests/` |
| Add control entity | `number.py`/`select.py` + corresponding `.h/.cpp` | Follow HeaterNumber pattern |
| Thermostat logic | `heater_policies.cpp:Thermostat::control()` | AUTO/HEAT machine, host-tested |
| Mode entry decisions | `heater_policies.cpp:Thermostat::enter_mode()` | Shares threshold with control |
| Combustion safety | `heater_policies.cpp:CombustionSafety` | HX overrides, BDAP fan, cooldown |
| Room-sensor guard | `heater_policies.cpp:RoomSensorGuard` | Boot consensus, glitch, staleness |
| TX frame construction | `bdap_driver.cpp:build_frame()` / `vevor_driver.cpp:build_frame()` | Framing + encode |
| Fuel-domain conversion | `heater_output.h` | normalized ↔ Hz ↔ level, one definition |
| Fuel priming | `heater_uart.cpp:start_priming()/stop_priming()` | BDAP-only (Vevor refuses) |
| State mappings | `bdap_driver.cpp` / `vevor_driver.cpp` | run-state names, error tables |
| **Tune pump/fan/HX for a heater** | **`heater_profile.h` preset or per-key YAML overrides** | Overrides revert-to-preset when invariant-breaking |
| Add a heater model | `heater_profile.h` + `__init__.py` + README table | Enum entry, preset factory, `cv.enum` mapping; new protocol = one driver |

## CODE MAP

| Symbol | Type | Location | Role |
|--------|------|----------|------|
| `HeaterUart` | class | `heater_uart.h` | Facade: entities, mode intent, policy/driver wiring |
| `HeaterMode` | enum | `heater_policies.h` | OFF, AUTO, ON modes (control vocabulary) |
| `HeaterProtocolDriver` | interface | `heater_protocol_driver.h` | Protocol seam: ByteStream, Telemetry, ControlInputs |
| `BdapDriver` | class | `bdap_driver.h/.cpp` | 0x76/0x16 framing, cadence, echo, CRC-16, TX build/parse |
| `VevorDriver` | class | `vevor_driver.h/.cpp` | 0xAA/0x66 TX state machine, level encode + derate, RX decode |
| `Thermostat` | class | `heater_policies.h/.cpp` | Control policy: three-phase AUTO, HEAT variants, shutdown/restart |
| `CombustionSafety` | class | `heater_policies.h/.cpp` | Protection policy: HX overrides, BDAP fan, cooldown monitor |
| `RoomSensorGuard` | class | `heater_policies.h/.cpp` | DHT22 physics guard: boot consensus, glitch, staleness |
| `output_domain` | namespace | `heater_output.h` | The one fuel-domain conversion (host-tested) |
| `start_priming()` | method | `heater_uart.cpp` | Start fuel priming (BDAP-only) |
| `stop_priming()` | method | `heater_uart.cpp` | Stop fuel priming |

## CONVENTIONS

**Heater Profiles:**
- All heater-specific values (pump Hz, fan RPM, HX temps, glow power, temp range, priming rate) live ONLY in `heater_profile.h` presets — with optional per-key YAML overrides that revert-to-preset when invariant-breaking
- Thermostat policy constants (aggressiveness, cadence) live in `Thermostat::Params` defaults — YAML-overridable, never hardcoded in the .cpp
- Control logic (`heater_policies.cpp`, `heater_uart.cpp`, entities) reads exclusively from `profile_.*` / `params.*` — never hardcoded values
- `heater_model:` is REQUIRED in YAML — the profile is never implicit
- `vevor_2kw_v2` is the in-production profile (lab device); `vevor_2kw` shares
  its tuning values and differs only in the HEAT-mode policy (kept as the
  one-line revert target); open Vevor tuning items live in ISSUES.md #5-10

**UART Protocol:**
- Standalone-only since 2026-10-24 (LCD injection path removed; tag `lcd-injection-final`)
- BDAP: 24-byte TX and RX frames (`0x76` start, CRC-16/MODBUS bytes 22-23), 25000 baud
- Vevor: 16-byte TX / 56-57-byte RX frames (`0xAA 0x66` / `0xAA 0x77`, additive checksum), 4800 baud inverted
- Big-endian multi-byte: `(high << 8) | low`; Vevor HX temp is a signed int16 × 0.1 °C

**Sensor Registration:**
- Python calls `parent.set_sensor(key, sensor)` during setup
- C++ stores in `std::map<std::string, Sensor*>` by key
- Allows flexible config without recompiling C++

**Entity Pattern:**
- Python: Define schema in `*.py`, call `parent.set_X()`
- C++: Add member variable, setter, publish in `update()`

## COMPONENT REQUIREMENTS

```yaml
uart:
  baud_rate: 4800  # BDAP (jeabong_8kw) = 25000 | Vevor (vevor_2kw) = 4800, inverted pins
  tx_pin: GPIO17  # Same pin for half-duplex
  rx_pin: GPIO17
  rx_buffer_size: 512

heater_uart:
  heater_model: jeabong_8kw  # or vevor_2kw / vevor_2kw_v2 - required, selects the profile + protocol
  operating_voltage: "12V"  # or "24V"
```

> Baud/pin inversion must match the selected profile (`profile_.uart_baud_rate` /
> `profile_.uart_inverted`); the YAML `uart:` block cannot be set from the component.

## ANTI-PATTERNS (THIS PROJECT)

- **Never** hardcode pump/fan/HX limits in `heater_uart.cpp` or entities — put them in the `heater_profile.h` preset
- **Never** change baud rate from what the selected profile requires (25000 BDAP / 4800 Vevor)
- **Never** use separate TX/RX pins in standalone mode (half-duplex)
- **Don't** suppress frame validation errors - indicates wiring/protocol issues
- **Don't** run `vevor_2kw` profile for live control until its provisional values are bench-validated

## COMMON MODIFICATIONS

**Adding New Sensors:**
1. Add the sensor definition to `sensor.py` (name, unit, icon)
2. Add the value to `Telemetry` in `heater_protocol_driver.h`
3. Decode it in the owning driver's `parse_frame()` (BDAP and/or Vevor)
4. Publish it in the facade's `update()`

**Adding New Control Entities:**
1. Create a Python schema file (`number.py`/`select.py`/`button.py` etc.)
2. Create the C++ entity class following the `HeaterNumber` pattern (`.h`/`.cpp`)
3. Add a setter in `heater_uart.h` and register the entity schema in the platform module

**Adding a New Heater (see README "Adding a New Heater"):**
- Same protocol: one `HeaterModel` enum entry + preset factory + `profile_for()`
  case in `heater_profile.h`, one `cv.enum` line in `__init__.py`, one row in the
  README table, the model added to `ALL_MODELS` in `tests/test_profiles.cpp`
- New protocol: additionally one `HeaterProtocolDriver` implementation (+
  `heater_uart.json` files entry + host framing/decode tests, same commit)

**Modifying Frame Parsing:**
- BDAP: `bdap_driver.cpp:parse_frame()` - 0x76/0x16 24-byte frames @ 25000 baud
- Vevor: `vevor_driver.cpp:parse_frame()` - 0xAA/0x77 56/57-byte frames @ 4800 baud (additive checksum)
- Multi-byte values are big-endian: `(high << 8) | low`. Vevor HX temp is a **signed** int16 × 0.1 °C
- Vevor link-layer details: `@notes/vevor-protocol.md`

**Modifying Thermostat/Safety Logic:**
- Everything lives in `heater_policies.{h,cpp}` (`Thermostat` / `CombustionSafety` /
  `RoomSensorGuard`), is driven by the profile + `Thermostat::Params` (YAML-tunable),
  and is host-tested in `tests/test_policies.cpp`; pump Hz ↔ power level conversion
  happens only at the driver boundary (`heater_output.h`)

## COMMANDS

```bash
# Host tests - run before every commit (plain g++, no ESPHome needed)
make -C tests
tests/stub_check.sh   # from the repo root: syntax-checks every component TU

# Validate config (owner runs the device builds)
esphome compile config.yaml

# Debug logging
# Add to ESPHome config:
# logger:
#   level: DEBUG
```

## NOTES

- `@notes/` contains reference implementation (bluetoothheater-master) - read-only
- Standalone mode (the only mode): ESP32 is controller, sends frames at 1Hz
- LCD injection mode: REMOVED 2026-10-24 (untested, abandoned; code under tag `lcd-injection-final`)
- Auto mode uses external temp sensor (recommended) or heat exchanger temp as fallback

# PROJECT KNOWLEDGE BASE

**Generated:** 2026-04-19
**Commit:** 5a9ed63
**Branch:** full_control_w_autoshutoff

## OVERVIEW

ESPHome external component for UART communication with diesel heaters. Parses 48-byte frames to monitor/control temperature, fan speed, voltage, diagnostics. Supports standalone thermostat mode with auto-shutdown.

## STRUCTURE

```
.
├── components/heater_uart/    # Main component (C++ + Python)
│   ├── heater_uart.cpp/h      # Core class: frame parsing, control logic
│   ├── heater_profile.h       # ALL heater-specific tuning (pump/fan/HX limits per model)
│   ├── __init__.py            # ESPHome schema registration
│   ├── sensor.py              # 9 numeric sensors
│   ├── text_sensor.py         # 2 text sensors (run_state, error_code)
│   ├── binary_sensor.py       # 4 binary sensors
│   ├── number.py              # Temperature + pump frequency control
│   ├── select.py              # Mode selection (Off/Auto/Heat)
│   ├── button.py              # Fuel priming button
│   ├── heater_number.cpp/h    # Temperature number entity
│   ├── heater_pump_number.cpp/h  # Pump frequency number entity
│   ├── heater_select.cpp/h    # Mode select entity
│   ├── fuel_prime_button.h    # Fuel prime button entity
│   └── heater_uart.json       # ESPHome component manifest
├── @notes/bluetoothheater-master/  # Reference implementation (read-only)
└── README.md
```

## WHERE TO LOOK

| Task | Location | Notes |
|------|----------|-------|
| Add sensor | `sensor.py` + `heater_uart.cpp` | Add to both Python and C++ |
| Modify frame parsing | `heater_uart.cpp:parse_frame()` | Byte positions match hardware protocol |
| Add control entity | `number.py`/`select.py` + corresponding `.h/.cpp` | Follow HeaterNumber pattern |
| Thermostat logic | `heater_uart.cpp:standalone_loop()` | Auto mode, cooldown, pump control |
| Standalone RX parsing | `heater_uart.cpp:parse_rx_frame()` | 24-byte RX frame in standalone mode |
| TX frame construction | `heater_uart.cpp:build_tx_frame()` | Build 24-byte TX frame for heater |
| Fuel priming | `heater_uart.cpp:start_priming()`/`stop_priming()` | 60s pump-only run for bleeding air |
| State mappings | `heater_uart.cpp` static maps | `run_state_map`, `error_code_map` |
| **Tune pump/fan/HX for a heater** | **`heater_profile.h`** | Per-model preset profiles, selected by `heater_model:` YAML key |
| Add a heater model | `heater_profile.h` + `__init__.py` + README table | Enum entry, preset factory, `cv.enum` mapping |

## CODE MAP

| Symbol | Type | Location | Role |
|--------|------|----------|------|
| `HeaterUart` | class | `heater_uart.h:84` | Main component - frame parsing, control |
| `HeaterMode` | enum | `heater_uart.h:21` | OFF, AUTO, ON modes |
| `parse_frame()` | method | `heater_uart.cpp:208` | Parse 48-byte UART frame (LCD mode) |
| `standalone_loop()` | method | `heater_uart.cpp:443` | Thermostat control logic (standalone mode) |
| `build_tx_frame()` | method | `heater_uart.cpp:507` | Construct TX frame for heater |
| `send_standalone_frame()` | method | `heater_uart.cpp:611` | Send frame and manage RX timeout |
| `parse_rx_frame()` | method | `heater_uart.cpp:661` | Parse 24-byte RX frame in standalone mode |
| `start_priming()` | method | `heater_uart.cpp:951` | Start fuel priming (60s pump-only) |
| `stop_priming()` | method | `heater_uart.cpp:980` | Stop fuel priming |

## CONVENTIONS

**Heater Profiles:**
- All heater-specific values (pump Hz, fan RPM, HX temps, glow power, temp range, priming rate) live ONLY in `heater_profile.h` presets
- Control logic (`heater_uart.cpp`, entities) reads exclusively from `profile_.*` — never hardcoded values
- `heater_model:` is REQUIRED in YAML — the profile is never implicit
- `vevor_2kw` profile is PROVISIONAL until bench-validated (interfacing phase)

**UART Protocol:**
- LCD mode: 48-byte frames: TX (bytes 0-23) + RX (bytes 24-47)
- Standalone mode: Separate 24-byte TX and RX frames
- Start marker: `0x76` at bytes 0 and 24
- End marker: `0x00` at bytes 21 and 45
- Baud rate: 25000 (heater-specific)
- Big-endian multi-byte: `(high << 8) | low`

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
  heater_model: jeabong_8kw  # or vevor_2kw - required, selects the profile + protocol
  standalone_mode: true  # For thermostat control
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
2. Add the parsed-value member variable in `heater_uart.h`
3. Parse it in `parse_rx_frame()` (BDAP) and/or `parse_vevor_rx_frame()` (Vevor)
4. Publish it in `update()`

**Adding New Control Entities:**
1. Create a Python schema file (`number.py`/`select.py`/`button.py` etc.)
2. Create the C++ entity class following the `HeaterNumber` pattern (`.h`/`.cpp`)
3. Add a setter in `heater_uart.h` and register the entity schema in the platform module

**Modifying Frame Parsing:**
- BDAP: `parse_frame()`/`parse_rx_frame()` - 0x76/0x16 24-byte frames @ 25000 baud
- Vevor: `parse_vevor_rx_frame()` - 0xAA/0x77 56-byte frames @ 4800 baud (additive checksum)
- Multi-byte values are big-endian: `(high << 8) | low`. Vevor HX temp is a **signed** int16 × 0.1 °C
- Vevor link-layer details: `@notes/vevor-protocol.md`

**Modifying Thermostat/Safety Logic:**
- Everything lives in `thermostat_control()` (shared by both protocols) and is driven
  by the profile; pump Hz ↔ power level conversion happens only at the Vevor TX boundary

## COMMANDS

```bash
# Validate config
esphome compile config.yaml

# Debug logging
# Add to ESPHome config:
# logger:
#   level: DEBUG
```

## NOTES

- `@notes/` contains reference implementation (bluetoothheater-master) - read-only
- Standalone mode: ESP32 is controller, sends frames at 1Hz
- LCD mode: Passive monitoring of existing controller-heater communication
- Auto mode uses external temp sensor (recommended) or heat exchanger temp as fallback

# PROJECT KNOWLEDGE BASE

**Generated:** 2026-03-16
**Commit:** e1d361b
**Branch:** full_control_w_autoshutoff

## OVERVIEW

ESPHome external component for UART communication with diesel heaters. Parses 48-byte frames to monitor/control temperature, fan speed, voltage, diagnostics. Supports standalone thermostat mode with auto-shutdown.

## STRUCTURE

```
.
├── components/heater_uart/    # Main component (C++ + Python)
│   ├── heater_uart.cpp/h      # Core class: frame parsing, control logic
│   ├── __init__.py            # ESPHome schema registration
│   ├── sensor.py              # 9 numeric sensors
│   ├── text_sensor.py         # 2 text sensors (run_state, error_code)
│   ├── binary_sensor.py       # 3 binary sensors
│   ├── number.py              # Temperature + pump frequency control
│   ├── select.py              # Mode selection (Off/Auto/Heat)
│   └── button.py              # Fuel priming button
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
| State mappings | `heater_uart.cpp` static maps | `run_state_map`, `error_code_map` |

## CODE MAP

| Symbol | Type | Location | Role |
|--------|------|----------|------|
| `HeaterUart` | class | `heater_uart.h:85` | Main component - frame parsing, control |
| `HeaterMode` | enum | `heater_uart.h:18` | OFF, AUTO, ON modes |
| `parse_frame()` | method | `heater_uart.cpp` | Parse 48-byte UART frame |
| `standalone_loop()` | method | `heater_uart.cpp` | Thermostat control logic |
| `build_tx_frame()` | method | `heater_uart.cpp` | Construct TX frame for heater |

## CONVENTIONS

**UART Protocol:**
- 48-byte frames: TX (bytes 0-23) + RX (bytes 24-47)
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
  baud_rate: 25000
  tx_pin: GPIO17  # Same pin for half-duplex
  rx_pin: GPIO17
  rx_buffer_size: 512

heater_uart:
  standalone_mode: true  # For thermostat control
  operating_voltage: "12V"  # or "24V"
```

## ANTI-PATTERNS (THIS PROJECT)

- **Never** change baud rate from 25000 (hardware fixed)
- **Never** use separate TX/RX pins in standalone mode (half-duplex)
- **Don't** suppress frame validation errors - indicates wiring/protocol issues

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

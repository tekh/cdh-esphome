# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a custom ESPHome component that provides UART communication with heaters. The component parses 48-byte UART frames to monitor and control heater operations including temperature, fan speed, voltage, and diagnostics.

## Architecture

### Component Structure

The codebase follows ESPHome's external component architecture with C++ implementation and Python ESPHome integration:

**C++ Core (`heater_uart.cpp/h`)**:
- `HeaterUart` class extends `PollingComponent` and `UARTDevice`
- Frame parsing happens in `loop()` which reads UART data byte-by-byte
- Parsed values are stored in member variables and published in `update()`
- Uses dynamic sensor registration via `std::map` to store sensor/text_sensor/binary_sensor references by string keys

**Python ESPHome Integration**:
- `__init__.py`: Component configuration schema and registration
- `sensor.py`: Defines 9 numeric sensors (temperature, voltage, current, etc.)
- `text_sensor.py`: Defines 2 text sensors (run_state, error_code)
- `binary_sensor.py`: Defines 1 binary sensor (on_off_state)

### UART Frame Protocol

The component expects 48-byte frames structured as:
- Bytes 0-23: Command/TX frame (starts with 0x76, ends with 0x00 at byte 21)
- Bytes 24-47: Response/RX frame (starts with 0x76 at byte 24, ends with 0x00 at byte 45)

Frame parsing extracts:
- Current/desired temperature from command frame
- All sensor values from response frame (fan speed, voltages, currents, states)
- Run state and error codes are mapped to human-readable descriptions via static maps

### Key Implementation Details

**Frame Validation**:
- Looks for 0x76 start markers at bytes 0 and 24
- Checks for 0x00 end markers at bytes 21 and 45
- Resets frame buffer on invalid packets

**Update Interval**:
- Configurable via `update_interval` (default 5s)
- Sensors publish on update cycle, not immediately when frames arrive

**Sensor Registration Pattern**:
- Python code calls `parent.set_sensor(key, sensor)` during setup
- C++ stores sensors in maps keyed by string identifiers
- This allows flexible sensor configuration without recompiling C++

## Development Commands

### Testing Configuration

Since this is an ESPHome component, testing requires:
1. An ESPHome configuration file that references the component
2. ESPHome CLI tools installed

Example test command:
```bash
esphome compile test-config.yaml
```

### Debugging

Enable debug logging in ESPHome config:
```yaml
logger:
  level: DEBUG
```

This will show UART frame parsing details and validation warnings.

## Component Requirements

- **Baud Rate**: Must be 25000 (heater-specific)
- **UART Buffer**: Set `rx_buffer_size: 512` to handle frame buffering
- **Dependencies**: Requires `uart` component to be configured

## Common Modifications

**Adding New Sensors**:
1. Add sensor definition to Python sensor type file (sensor.py, text_sensor.py, or binary_sensor.py)
2. Add corresponding member variable in `heater_uart.h`
3. Parse the value in `parse_frame()` in `heater_uart.cpp`
4. Publish the value in `update()` in `heater_uart.cpp`

**Modifying Frame Parsing**:
- Frame structure is defined by the heater hardware
- Byte positions in `parse_frame()` correspond to heater protocol specification
- Multi-byte values use big-endian byte order: `(high_byte << 8) | low_byte`

**State Mappings**:
- `run_state_map` and `error_code_map` in `heater_uart.cpp` provide human-readable descriptions
- Add entries to these maps to support additional states/errors

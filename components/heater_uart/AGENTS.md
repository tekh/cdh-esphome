# HEATER_UART COMPONENT

ESPHome component for diesel heater UART communication.

## FILE ROLES

| File | Purpose |
|------|---------|
| `heater_uart.cpp/h` | Core: frame parsing, thermostat, UART handling |
| `__init__.py` | ESPHome schema, config validation |
| `sensor.py` | 9 numeric sensors (temp, voltage, current) |
| `text_sensor.py` | `run_state`, `error_code` human-readable |
| `binary_sensor.py` | `on_off_state`, `auto_shutdown_active`, `standby_active`, `priming_active` |
| `number.py` | Temperature + pump frequency controls |
| `select.py` | Mode: Off/Auto/Heat |
| `button.py` | Fuel priming button |
| `heater_*.cpp/h` | C++ implementations for number/select entities |

## ADDING ENTITIES

1. **Python**: Add schema in `*.py`, call `parent.set_X(key, entity)`
2. **C++**: Add member + setter in `heater_uart.h`, parse in `parse_frame()`, publish in `update()`

## FRAME LAYOUT

```
TX [0-23]:  0x76 | cmd | temp | ... | 0x00 | CRC
RX [24-47]: 0x76 | run_state | error | sensors... | 0x00 | CRC
```

Key byte positions defined in `heater_uart.cpp:parse_frame()`.

## THERMOSTAT (standalone_loop)

- **AUTO mode**: Start → approach control → auto-shutdown → standby → restart
- Parameters: `auto_shutdown_overshoot`, `auto_shutdown_hysteresis`, `approach_threshold`
- **HEAT mode safety**: Ambient temperature limit (`ambient_heat_limit`) - forces shutdown if room > 26°C (configurable, range: -40 to 80°C)
- **Fuel priming**: 60-second pump-only run for bleeding air from fuel lines (`start_priming()`/`stop_priming()`)
- Safety: Heat exchanger temp limits, cooldown sequence

## PUMP CONTROL

- Range: 1.3-5.5 Hz (`PUMP_FREQ_MIN` to `PUMP_FREQ_MAX`)
- Adjustment interval: 5s (`PUMP_ADJUST_INTERVAL_MS`)
- HX temp target: 250°C (`HX_TEMP_TARGET`)

## CONFIGURATION PARAMETERS

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `auto_shutdown_overshoot` | 0.5°C | 0.0 - 3.0°C | Temp overshoot before auto-shutdown (AUTO mode) |
| `auto_shutdown_hysteresis` | 1.5°C | 0.5 - 5.0°C | Temp drop before auto-restart (AUTO mode) |
| `approach_threshold` | 1.0°C | 0.5 - 3.0°C | Start reducing pump when within this of target |
| `ambient_heat_limit` | 26.0°C | -40 to 80°C | HEAT mode safety limit - force shutdown if room > X°C |

## CONSTANTS

| Constant | Value | Purpose |
|----------|-------|---------|
| `BAUD_RATE` | 25000 | Fixed by heater hardware |
| `STANDALONE_TX_INTERVAL_MS` | 1000 | 1 Hz frame rate |
| `STANDALONE_RX_TIMEOUT_MS` | 200 | RX response timeout |
| `HX_TEMP_CRITICAL` | 265°C | Emergency shutdown (heat exchanger) |
| `HX_TEMP_HIGH` | 255°C | Decrease pump (less fuel) |
| `HX_TEMP_TARGET` | 250°C | Target heat exchanger temp |
| `HX_TEMP_LOW` | 190°C | Increase pump (more fuel) |
| `COOLDOWN_FAN_RPM` | 4000 | Cooldown fan speed |
| `IGNITION_FAN_RPM` | 2000 | Fan speed during ignition (HX < 100°C) |
| `PUMP_FREQ_MIN` | 1.3 Hz | Minimum pump frequency |
| `PUMP_FREQ_MAX` | 5.5 Hz | Maximum pump frequency |
| `PUMP_FREQ_INITIAL` | 1.8 Hz | Initial pump frequency for ignition |
| `PUMP_FREQ_STEP` | 0.1 Hz | Pump adjustment step size |
| `PUMP_ADJUST_INTERVAL_MS` | 5000 | 5s between pump adjustments |
| `DEFAULT_ALTITUDE` | 750m | Default altitude (affects combustion) |

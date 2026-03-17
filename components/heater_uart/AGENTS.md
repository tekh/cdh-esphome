# HEATER_UART COMPONENT

ESPHome component for diesel heater UART communication.

## FILE ROLES

| File | Purpose |
|------|---------|
| `heater_uart.cpp/h` | Core: frame parsing, thermostat, UART handling |
| `__init__.py` | ESPHome schema, config validation |
| `sensor.py` | 9 numeric sensors (temp, voltage, current) |
| `text_sensor.py` | `run_state`, `error_code` human-readable |
| `binary_sensor.py` | `on_off_state`, `auto_shutdown_active`, `standby_active` |
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
- Safety: Heat exchanger temp limits, cooldown sequence

## PUMP CONTROL

- Range: 1.3-5.5 Hz (`PUMP_FREQ_MIN` to `PUMP_FREQ_MAX`)
- Adjustment interval: 5s (`PUMP_ADJUST_INTERVAL_MS`)
- HX temp target: 250°C (`HX_TEMP_TARGET`)

## CONSTANTS

| Constant | Value | Purpose |
|----------|-------|---------|
| `BAUD_RATE` | 25000 | Fixed by heater hardware |
| `STANDALONE_TX_INTERVAL_MS` | 1000 | 1 Hz frame rate |
| `HX_TEMP_CRITICAL` | 265°C | Emergency shutdown |
| `COOLDOWN_FAN_RPM` | 4000 | Cooldown fan speed |

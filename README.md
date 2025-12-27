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
- **Text Sensors**:
  - Run State
  - Error Code
- **Automatic Temperature Control**:
  - Auto-shutdown when target temperature is reached
  - Auto-restart when temperature drops below threshold
  - Intelligent approach control to prevent overshoot
- **Mappings for Readability**:
  - Human-readable descriptions for error codes and run states.

## Operational Modes

This component supports two operational modes:

### Standalone Mode (Primary)

In standalone mode, the ESP32 acts as the primary heater controller with full control capabilities:

- **Full Control**: ESP32 sends UART frames to the heater at 1 Hz intervals
- **Thermostat Operation**: Automatic temperature control with intelligent heating/cooling cycles
- **Temperature Sensing**: Uses external temperature sensor (recommended) or heat exchanger temperature as fallback
- **Voltage Support**: Supports both 12V and 24V heaters
- **Safety Features**: Built-in cooldown sequences and heat exchanger temperature protection

Enable standalone mode in your configuration:
```yaml
heater_uart:
  standalone_mode: true
  operating_voltage: "12V"  # or "24V"
  temperature_sensor: external_temp_sensor_id  # Optional but recommended
```

### LCD Controller Mode (Alternative)

In this mode, the ESP32 monitors the UART communication between an existing LCD controller and the heater:

- **Monitoring Only**: Reads and parses UART frames without interfering
- **No Control**: Control entities are not available
- **Passive Operation**: Suitable for adding smart home integration to existing setups

This is the default mode when `standalone_mode` is not specified or set to `false`.

## Auto Mode

When using standalone mode with the mode select set to "Auto", the component provides intelligent thermostat control:

### How Auto Mode Works

1. **Heating Phase**: When room temperature is below target, the heater starts and increases heat output
2. **Approach Control**: As temperature approaches the target, heating rate automatically slows to prevent overshoot
3. **Auto-Shutdown**: When target temperature is reached, the heater shuts down and enters cooldown mode
4. **Standby Monitoring**: While off, the component continuously monitors room temperature
5. **Auto-Restart**: When temperature drops below the restart threshold, the heater automatically starts again

### Configuration Parameters

Fine-tune the Auto mode behavior with these parameters:

- `auto_shutdown_overshoot` (default: 0.5°C): Temperature above target before auto-shutdown triggers
- `auto_shutdown_hysteresis` (default: 1.5°C): Temperature drop needed before auto-restart
- `approach_threshold` (default: 2.0°C): Distance from target when approach control begins

Example configuration:
```yaml
heater_uart:
  standalone_mode: true
  auto_shutdown_overshoot: 0.5
  auto_shutdown_hysteresis: 1.5
  approach_threshold: 2.0
```

### Mode Options

The mode select entity provides three operating modes:

- **Off**: Heater is forced off (enters cooldown if running)
- **Auto**: Thermostat-controlled operation with auto-shutdown and auto-restart
- **On**: Heater is forced on, bypassing thermostat control (safety limits still apply)

## Getting Started

### Installation

Update your ESPHome configuration to include this component.

### Example Configuration

This example shows a complete standalone mode configuration with full control capabilities:

```yaml
external_components:
  - source: github://daoudeddy/cdh-esphome@main
    components: [ heater_uart ]

# Standalone mode uses single-wire half-duplex UART
uart:
  baud_rate: 25000
  tx_pin: GPIO17  # Use same pin for TX and RX
  rx_pin: GPIO17
  id: uart_bus
  rx_buffer_size: 512

heater_uart:
  update_interval: 5s
  id: heater
  standalone_mode: true
  operating_voltage: "12V"  # or "24V" depending on your heater
  altitude: 750  # meters above sea level
  auto_shutdown_overshoot: 0.5  # °C above target before shutdown
  auto_shutdown_hysteresis: 1.5  # °C drop before auto-restart
  approach_threshold: 2.0  # °C from target to start slowing down
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
```

### Notes

**General:**
- The baud rate must be set to `25000` to communicate with the heater.
- Ensure the `rx_pin` and `tx_pin` match your ESP32 or ESP8266 configuration.

**Standalone Mode:**
- Use single-wire half-duplex UART: `tx_pin` and `rx_pin` must be the **same GPIO pin**.
- External temperature sensor is highly recommended for accurate room temperature control.
- Without an external sensor, the component will use heat exchanger temperature as a fallback.
- In Auto mode, the heater will automatically manage itself between `desired_temperature ± hysteresis`.

**Mode Select Usage:**
- **Off**: Forces heater off. Use this when you don't want the heater to run.
- **Auto**: Thermostat mode. Heater will auto-start when cold and auto-shutdown when warm.
- **On**: Forces heater on, bypassing thermostat. Safety limits (heat exchanger temperature) still apply.

## Development

### Code Structure

**Core Component:**
- **`heater_uart.h`**: Header file defining the component.
- **`heater_uart.cpp`**: Core logic for UART communication, frame parsing, thermostat control, and sensor management.
- **`__init__.py`**: Python module to integrate the component with ESPHome.

**Sensor Entities:**
- **`sensor.py`**: Numeric sensor definitions (temperature, voltage, current, etc.).
- **`text_sensor.py`**: Text sensor definitions (run state, error codes).
- **`binary_sensor.py`**: Binary sensor definitions (on/off, auto-shutdown, standby states).

**Control Entities:**
- **`number.py`**: Number entity definitions (temperature and pump frequency control).
- **`heater_number.h/cpp`**: Temperature number entity implementation.
- **`heater_pump_number.h/cpp`**: Pump frequency number entity implementation.
- **`select.py`**: Select entity definition (mode selection).
- **`heater_select.h/cpp`**: Mode select entity implementation.

### Debugging

Enable debug logging in your ESPHome configuration to monitor the UART frames and parsed data:

```yaml
logger:
  level: DEBUG
```

<objective>
Implement an automated shutdown/restart feature for the Chinese Diesel Heater ESPHome controller.

The heater should automatically shut down when:
1. The fuel pump frequency is at minimum (PUMP_FREQ_MIN = 1.3 Hz)
2. AND the room temperature exceeds the desired temperature by a configurable overshoot threshold

The heater should automatically restart when:
1. Room temperature drops a configurable amount below the desired temperature (hysteresis)
2. AND auto-shutdown is enabled
3. AND the heater was previously auto-shutdown (not manually turned off)

This feature must be toggleable by the user via a switch exposed to Home Assistant.
</objective>

<context>
This is an ESPHome component for controlling a Chinese Diesel Heater (8kW) in standalone mode.

Key files to modify:
- `./components/heater_uart/heater_uart.cpp` - Main implementation
- `./components/heater_uart/heater_uart.h` - Header with constants and class definition
- `./components/heater_uart/__init__.py` - ESPHome component configuration
- `./components/heater_uart/switch.py` - Switch platform implementation
- `./components/heater_uart/binary_sensor.py` - Binary sensor platform
- `./components/heater_uart/number.py` - Number platform (for temperature control)

Read AGENTS.md for project conventions and structure.

Current thermostat logic is in `parse_rx_frame()` around line 652-742 of heater_uart.cpp.

The heater uses these run states:
- RUN_STATE_OFF (0)
- RUN_STATE_RUNNING (5) - Normal operation, thermostat active
- RUN_STATE_COOLDOWN (8) - Fan running to cool heat exchanger

Existing cooldown logic uses `in_cooldown_` flag to run fan at high speed until HX temp drops.
</context>

<requirements>
1. **Auto-Shutdown Logic**
   - Trigger when: pump at PUMP_FREQ_MIN AND room_temp > (target_temp + overshoot_threshold)
   - Use a small epsilon (0.05f) when comparing pump frequency to account for floating point
   - Execute normal shutdown sequence (full cooldown with fan)
   - Set `in_auto_shutdown_` flag to track state
   - Log the auto-shutdown event at INFO level

2. **Auto-Restart Logic**
   - Trigger when: room_temp < (target_temp - restart_hysteresis) AND in_auto_shutdown_ is true
   - Only restart if auto_shutdown_enabled_ switch is on
   - Use existing turn_on() behavior: set pump to PUMP_FREQ_INITIAL, reset timers
   - Clear `in_auto_shutdown_` flag after restart
   - Log the auto-restart event at INFO level

3. **Configuration Options (YAML)**
   Add to `__init__.py`:
   - `auto_shutdown_overshoot` (float, default 0.5, range 0-3.0)
   - `auto_shutdown_hysteresis` (float, default 1.5, range 0.5-5.0)

   Add corresponding setters in header file.

4. **Enable/Disable Switch**
   Add a new switch type in `switch.py`:
   - Type: `auto_shutdown`
   - Class: `HeaterAutoShutdownSwitch`
   - Exposed to Home Assistant
   - Default state: enabled (true)
   - Add `set_auto_shutdown_switch()` method to HeaterUart class

5. **Auto-Shutdown Status Sensor**
   Add a new binary sensor in `binary_sensor.py`:
   - Key: `auto_shutdown_active`
   - Shows whether heater is currently in auto-shutdown state
   - Publish in update() when this state changes

6. **0.5 Degree Temperature Intervals**
   Modify temperature control to support half-degree precision:
   - In `number.py`: Change default step from 1 to 0.5
   - In `heater_uart.h`: Change `uint8_t desired_temp_setting_` to `float desired_temp_setting_`
   - In `set_desired_temperature()`: Accept float, validate range, round to nearest 0.5
   - In `build_tx_frame()`: Convert float to uint8_t for protocol (heater only accepts integers)
   - The protocol byte 4 only accepts integers, so round the float when building the frame
</requirements>

<implementation>
Follow the existing code patterns in the project.

**heater_uart.h additions:**
```cpp
// Auto-shutdown configuration
float auto_shutdown_overshoot_ = 0.5f;
float auto_shutdown_hysteresis_ = 1.5f;
bool auto_shutdown_enabled_ = true;
bool in_auto_shutdown_ = false;
switch_::Switch *auto_shutdown_switch_{nullptr};

// Setters
void set_auto_shutdown_overshoot(float offset) { this->auto_shutdown_overshoot_ = offset; }
void set_auto_shutdown_hysteresis(float hyst) { this->auto_shutdown_hysteresis_ = hyst; }
void set_auto_shutdown_enabled(bool enabled) { this->auto_shutdown_enabled_ = enabled; }
void set_auto_shutdown_switch(switch_::Switch *sw) { this->auto_shutdown_switch_ = sw; }
bool is_in_auto_shutdown() const { return in_auto_shutdown_; }

// Change desired_temp_setting_ from uint8_t to float
float desired_temp_setting_ = 22.0f;
```

**heater_uart.cpp - Auto-shutdown check in parse_rx_frame():**
Insert after the thermostat logic (around line 738), before applying new_freq:
```cpp
// Auto-shutdown: at minimum pump but room still above target + overshoot
if (pump_freq_setting_ <= PUMP_FREQ_MIN + 0.05f &&
    room_temp > target_temp + auto_shutdown_overshoot_ &&
    auto_shutdown_enabled_) {
    ESP_LOGI(TAG, "Auto-shutdown: room %.1f°C > target %.1f°C + %.1f°C overshoot at min pump",
             room_temp, target_temp, auto_shutdown_overshoot_);
    pending_on_off_command_ = CMD_STOP;
    heater_on_request_ = false;
    in_cooldown_ = true;
    in_auto_shutdown_ = true;
    return;
}
```

**heater_uart.cpp - Auto-restart check:**
Add near the beginning of parse_rx_frame(), after parsing but before thermostat logic:
```cpp
// Auto-restart check: heater is off and was auto-shutdown
if (!on_off_value_ && in_auto_shutdown_ && auto_shutdown_enabled_ && standalone_mode_) {
    float room_temp = current_temperature_value_;
    float target_temp = desired_temp_setting_;
    if (room_temp < target_temp - auto_shutdown_hysteresis_) {
        ESP_LOGI(TAG, "Auto-restart: room %.1f°C < target %.1f°C - %.1f°C hysteresis",
                 room_temp, target_temp, auto_shutdown_hysteresis_);
        pending_on_off_command_ = CMD_START;
        heater_on_request_ = true;
        pump_freq_setting_ = PUMP_FREQ_INITIAL;
        last_pump_adjust_time_ = millis();
        in_auto_shutdown_ = false;
    }
}
```

**Manual turn off clears auto-shutdown flag:**
In `turn_off()`, add:
```cpp
in_auto_shutdown_ = false;  // Manual turn off clears auto-shutdown state
```

**build_tx_frame - handle float temperature:**
Update byte 4 assignment:
```cpp
frame[4] = static_cast<uint8_t>(std::round(desired_temp_setting_));
```

**update() - publish auto_shutdown_active binary sensor:**
Add to the binary sensor loop:
```cpp
else if (key == "auto_shutdown_active")
    binary_sensor->publish_state(in_auto_shutdown_);
```

**Sync auto_shutdown_switch_ in update():**
```cpp
if (auto_shutdown_switch_ != nullptr) {
    auto_shutdown_switch_->publish_state(auto_shutdown_enabled_);
}
```
</implementation>

<verification>
Before declaring complete:

1. Run `esphome compile lab-heater-butler.yaml` to verify compilation
2. Verify the new config options appear in the YAML schema
3. Check that HeaterAutoShutdownSwitch class compiles correctly
4. Verify the binary sensor for auto_shutdown_active is added
5. Confirm temperature step is 0.5 in number.py

Test scenarios to verify (manual testing):
- Auto-shutdown triggers when at min pump and room > target + overshoot
- Auto-restart triggers when room < target - hysteresis after auto-shutdown
- Manual turn-off does NOT trigger auto-restart later
- Disabling auto_shutdown switch prevents both auto-shutdown and auto-restart
- Temperature can be set to values like 21.5, 22.0, 22.5
</verification>

<success_criteria>
- Code compiles without errors
- Auto-shutdown triggers correctly based on configurable thresholds
- Auto-restart only occurs after auto-shutdown, not after manual shutdown
- All new configs exposed in YAML with proper validation and defaults
- Switch and binary sensor visible in Home Assistant
- Temperature accepts 0.5 degree steps (8.0, 8.5, 9.0, ..., 35.0)
- Existing functionality (manual on/off, thermostat, cooldown) unchanged
- Manual turn off clears auto-shutdown state
</success_criteria>

# HEATER_UART COMPONENT

ESPHome component for diesel heater UART communication.

## ARCHITECTURE (ROADMAP.md Phase 2)

```
HeaterUart (facade: entities, intent, thermostat/safety policy, room sensor)
     │  ByteStream (transport interface)
     ▼
HeaterProtocolDriver ── BdapDriver (0x76/0x16 @ 25000) | VevorDriver (0xAA/0x66 @ 4800)
```

The facade works in a normalized domain (fuel fraction 0.0-1.0, shared run-state
enumeration, Telemetry struct); each driver owns its protocol's framing, cadence,
echo handling, checksums and byte encode/decode. Adding a heater that speaks an
EXISTING protocol = profile only; a NEW protocol = one new driver.

## FILE ROLES

| File | Purpose |
|------|---------|
| `heater_uart.cpp/h` | Facade: entity registry/publishing, mode intent, policy→driver wiring, priming (BDAP-only) |
| `heater_protocol_driver.h` | Driver seam: ByteStream, Telemetry, ControlInputs, interface, shared run-state names |
| `bdap_driver.cpp/h` | BDAP framing/cadence/echo/parse, CRC-16; encodes `ControlInputs` (fan RPM arrives pre-computed from `CombustionSafety`) |
| `vevor_driver.cpp/h` | Vevor framing, TX state machine, level encoding + altitude derate, RX decode, error map, fan probe |
| `heater_policies.h/.cpp` | Controller policies: Thermostat (AUTO/HEAT state machine incl. mode entry + auto-restart), CombustionSafety (HX overrides, BDAP fan, cooldown monitor), RoomSensorGuard (DHT22 physics guard) |
| `heater_profile.h` | Per-model tuning presets + capability flags + `validate_profile_invariants()` (YAML-override repair) |
| `heater_output.h` | The one fuel-domain conversion (normalized ↔ Hz ↔ level), host-tested |
| `heater_log.h` | Log shim: ESP_LOG* on device, no-ops on host |
| `__init__.py` | ESPHome schema incl. the Phase-4 per-key tuning overrides |
| `sensor.py` | Numeric sensors (temp, voltage, current, glow cluster) |
| `text_sensor.py` | `run_state`, `error_code` human-readable |
| `binary_sensor.py` | `on_off_state`, `auto_shutdown_active`, `standby_active`, `priming_active` |
| `number.py` | Temperature + pump frequency + power level + fan probe controls |
| `select.py` | Mode: Off/Auto/Heat |
| `button.py` | Fuel priming button (BDAP-only) |
| `heater_*.cpp/h` | C++ implementations for number/select entities |

Host tests: `tests/` (`make -C tests`, plain g++) pin frame layouts, the TX
state machines, decode rules and profile invariants - run before committing.

## ADDING ENTITIES

1. **Python**: Add schema in `*.py`, call `parent.set_X(key, entity)`
2. **C++**: Add member + setter in `heater_uart.h`, extend `Telemetry` if a new
   parsed value is needed, decode in the owning driver's `parse_frame()`,
   publish in the facade's `update()`

## FRAME LAYOUT (the standalone path is the only one since the LCD injection
path was removed 2026-10-24; recover via git tag `lcd-injection-final`)

```
BDAP   TX 24B:  0x76 0x16 | cmd | temps | pump Hz | fan RPM | ... | CRC16
BDAP   RX 24B:  0x76 0x16 | run_state | error | sensors... | CRC16
Vevor  TX 16B:  0xAA 0x66 | cmd | 0x0B | ... | power 1-10 | state | ... | additive csum
Vevor  RX 56/57B: 0xAA 0x77 | ... | level | HX (int16 x0.1C) | pump | fan | ... | csum
```

Key byte positions defined in the owning driver's `parse_frame()`
(`bdap_driver.cpp` / `vevor_driver.cpp`) and pinned by `tests/`.

## THERMOSTAT + SAFETY (`heater_policies.cpp`, pure + host-tested)

- **AUTO mode**: start → approach control → auto-shutdown → standby → restart;
  `Thermostat::enter_mode()` owns mode-entry decisions and shares
  `standby_threshold()` with the periodic machine
- Parameters: `auto_shutdown_overshoot`, `auto_shutdown_hysteresis`,
  `approach_threshold` + the Phase-4 aggressiveness/cadence keys
  (`Thermostat::Params`, YAML-overridable)
- **HEAT mode**: modulating (8kW/vevor_2kw) or user-owned level (vevor_2kw_v2,
  `heat_mode_manual`); ambient heat limit (`ambient_heat_limit`) forces
  shutdown if room > limit in both cases
- **CombustionSafety**: HX soot floor / too-hot / critical stop
  (removable via `firmware_safety_oversight: false` on `ecm_owned_safety`
  profiles ONLY), BDAP fan matching, cooldown monitor
- **Fuel priming**: `start_priming()`/`stop_priming()`, BDAP-only (the Vevor
  self-primes; the button refuses on it)
- **RoomSensorGuard**: NaN/glitch rejection, boot 3-sample consensus,
  staleness cutoff — not part of the removable safety stack

# BDAP Jeabong 8 kW (`jeabong_8kw`)

**PARKED since 2026-10-12 — no device target exists.** Nothing on this page is
bench-verified: the protocol code compiles and is host-tested, but no unit has
ever been connected. Treat every entry as *expected*, not *measured*
(`ISSUES BD-1`).

## Identity

| | |
|---|---|
| YAML model | `jeabong_8kw` |
| Protocol | BDAP, **25000 baud, non-inverted**, 24-byte TX and RX, CRC-16/MODBUS (bytes 22-23) |
| Fuel command | real **pump frequency in Hz** (`has_pump_hz_control: true`) — profile range 1.3-5.5 Hz |
| Fan command | real **fan RPM** in the frame; the firmware computes it (`CombustionSafety`), no dial (`ISSUES BD-2`) |
| Control authority | the **firmware** — `ecm_owned_safety: false`, so `firmware_safety_oversight: false` is ignored with a warning |
| Priming | **supported** (`start_priming()`), the only heater where the Fuel Prime button works (`ISSUES BD-4`) |
| Altitude | self-derated by the heater from the altitude fields in the frame — the host does not scale fuel |

## What differs from the Vevor, in one paragraph

The BDAP panel takes pump Hz and fan RPM directly and owns less of the sequence,
so the firmware is the combustion-safety authority here: the HX soot floor /
too-hot / critical stop and the fan-fuel matching stay armed and cannot be
disabled. There is no ignition fuel gate (`gate_fuel_before_ignition: false`) —
the heater sequences glow and fuel itself, so the TX request passes through
unchanged during a start.

## To bring this heater online

1. A device config in this repo with `heater_model: jeabong_8kw`, `uart:` at
   **25000** baud on the same half-duplex pin, pins **not** inverted.
2. Declare the pump dial (`number: - platform: heater_uart / type: pump`) — that
   entity is not in the 2 kW YAML and its absence looks like a bug to anyone
   reading only the current config (`ISSUES CF-3`).
3. Bench-validate the preset before any live control: pump range, fan range, HX
   thresholds and the glow power byte are all unproven on real hardware.
4. Then work through `ISSUES BD-1`, `BD-2` (fan dial policy) and close them with
   evidence.

## What has to be checked first on a real unit

- CRC-16 acceptance in both directions (a mismatch is logged, not fatal — so a
  silent heater can look "connected").
- The run-state and error-code tables (`bdap_driver.cpp`) against the panel's own
  display.
- Fan matching: the firmware scales RPM with fuel rate, forces ignition RPM
  during startup and cooldown RPM during purge. If the real fan is slower to
  respond, the HX may overshoot before the fan catches up.
- Whether the unit self-derates altitude the way the reference implementation
  does — if not, fuel will run rich at this bench's ~750 m.

## Open items

`ISSUES BD-1` (no device target), `BD-2` (fan-RPM dial: clamped-manual vs full
manual, policy undecided).

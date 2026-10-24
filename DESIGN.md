# DESIGN: Multi-Heater Architecture

The 2026-10-24 refactor of `components/heater_uart/` — why the code is shaped the
way it is. Phases 0–5 all complete, each device-verified on the bench (Vevor
behaviour unchanged at every step). The pre-refactor code lives on the
`vevor_2kw` branch; the deeper analysis and full phase history are in git
(history of `ROADMAP.md` / `ARCHITECTURE-ANALYSIS.md, merged here) and in the
Hindsight memory.

## Architecture

```
┌──────────────────────────────────────────────────────┐
│ HeaterUart (facade): entity registry, publishing,    │
│ mode intent, wiring policy results to the driver     │
├──────────────────────────────────────────────────────┤
│ Policies (pure C++, host-tested):                    │  normalized
│ Thermostat (AUTO/HEAT, mode entry, auto-restart)     │  output
│ CombustionSafety (HX stack, BDAP fan, cooldown)      │  domain 0.0–1.0
│ RoomSensorGuard (DHT22 physics guard)                │
├──────────────────────────────────────────────────────┤
│ ProtocolDriver (interface):                         │
│   BdapDriver | VevorDriver | <future>                │  framing, cadence,
│ start/stop/set_output(0..1)/telemetry                │  encode/decode
├──────────────────────────────────────────────────────┤
│ HeaterProfile (data): identity + protocol +          │  compiled presets,
│ capabilities + tuning — overridable per-key in YAML  │  §YAML below
└──────────────────────────────────────────────────────┘
```

The policy layer speaks only the **normalized fuel domain 0.0–1.0**
(`heater_output.h`); each driver encodes it into whatever its protocol says
(Hz for BDAP, level 1–10 for Vevor). No policy code knows a protocol exists.

## Decisions (2026-04-19 review)

| # | Decision |
|---|----------|
| 1 | Design for **6–10 heater variants**; ECM behavior of future units unknown — the design must not assume "Vevor-class" or "BDAP-class". |
| 2 | **Extract seams first, then open YAML** — YAML exposure over unextracted code would only move half the problem (the thermostat constants lived in 7 diverging copies in the .cpp). |
| 3 | Firmware combustion oversight **ON by default**, disableable in YAML only on `ecm_owned_safety: true` profiles (the ECM is the safety authority there; on BDAP-class hardware the firmware itself *is* the safeguard and cannot be disabled). |
| 4 | Policy-as-model entries (e.g. `vevor_2kw_v2`) are **proper**: one YAML line reverts bench behavior. New heaters get their own profile. |
| 5 | LCD injection mode **abandoned and deleted** (untested, BDAP-only); last code under tag `lcd-injection-final`. Standalone is the only mode. |
| 6 | YAML audience is **one operator**: permissive validation — range guards, warnings; hard errors only for structurally invalid values. |
| 7 | **Host-side test harness** (`tests/`, plain g++, no ESPHome): every extracted behavior ships with tests; run before every commit. |

## Why these seams exist

- **`heater_output.h` (normalized 0.0–1.0)** — the Vevor protocol has no pump-Hz
  command; the old design overloaded `pump_freq_min/max` as a level-encoding range
  and implemented the level↔Hz mapping **four times by hand**. A third heater
  (5 levels, a watt target…) would have broken the assumption baked into the code.
  The Hz↔level bridge now exists exactly once, behind the driver boundary.
- **`ProtocolDriver` interface** — pre-refactor, `HeaterUart` was a ~1500-line god
  class; a third protocol meant branches in 6+ methods (loop, TX build, RX parse,
  priming gate, echo handling, resync). Drivers own framing/cadence/checksums;
  the facade just applies policy results.
- **Pure policies in `heater_policies.{h,cpp}`** — the thermostat's aggressiveness
  constants lived in 7 subtly-diverging copies; with no test harness, the only
  validation was a bench flash. Now: one `Thermostat` (mode entry shares one
  `standby_threshold()` with the periodic machine), one `CombustionSafety`, one
  `RoomSensorGuard` — pure C++, 80+ host tests.
- **Per-key YAML overrides over compiled presets** (not full-YAML profiles) —
  presets stay the safe, bench-validated default; `validate_profile_invariants()`
  reverts any override that breaks a profile invariant to the preset with a
  warning, so a typo can never silently disable a safety net. Model-identity
  flags, protocol timing and sensor-physics constants are deliberately NOT
  user-facing.

## Adding a heater

Heaters add as **data**, not code: same protocol = one profile factory + enum
entry + `cv.enum` line + test list entry (README "Adding a New Heater" is the
contract); a new protocol adds exactly one `ProtocolDriver` implementation.

## Phase outcomes (all device-verified 2026-10-24)

| Phase | Outcome |
|---|---|
| 0 | Host test harness (`tests/`, tiny framework + ESPHome stubs) |
| 1 | Single output-domain mapping; LCD injection path deleted |
| 2 | `BdapDriver` / `VevorDriver` behind the interface; `heater_uart` → facade |
| 3 | Pure policies extracted; `Thermostat::enter_mode()` owns mode entry; `firmware_safety_oversight:` YAML key |
| 4 | Per-key YAML tuning overrides + invariant validation (revert-to-preset) |
| 5 | "Adding a New Heater" checklist + doc-set coherence |

Standing constraints that outlive the refactor: only `vevor_2kw_v2` is
bench-accessible, so BDAP (8kW) stays **parked / compile-verified only**
(ISSUES.md #1); the room-sensor staleness gate and the ambient HEAT-mode limit
are **not removable** anywhere.

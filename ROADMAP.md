# ROADMAP: Multi-Heater Architecture Refactor

**Companion to `ARCHITECTURE-ANALYSIS.md`** (analysis = why; this = what/when).
Status markers: ☐ not started · ◐ in progress · ● done.

## Decisions (from the 2026-04-19 review)

| # | Topic | Decision |
|---|-------|----------|
| 1 | **Scale** | Up to **6–10 heater variants**; ECM behavior of future units is **unknown** → the design must not assume "Vevor-class" or "BDAP-class"; it must adapt per-model. |
| 2 | **Direction** | **C then A**: extract the seams (protocol driver / safety policy / control policy / tuning) first, then open tuning keys as YAML **overrides** on top of compiled, bench-validated presets. |
| 3 | **Safety stack** | Firmware safety oversight is **ON by default on every heater**, and disableable in YAML **only where the ECM owns combustion safety** (`ecm_owned_safety: true`). On BDAP-class hardware (firmware is the safety authority) it cannot be disabled. |
| 4 | **Policy-as-model** | `vevor_2kw_v2`-style model entries are **proper** — one YAML line reverts bench behavior. New heaters (e.g. `vevor_5kw`) get their own profile; they may be very different from the 2 kW. |
| 5 | **LCD injection mode** | **Abandoned.** The whole non-standalone path (48-byte sniff + command injection) is deleted from live code; the last commit carrying it is tagged `lcd-injection-final` for recovery. Not kept in-tree: untested (no BDAP device target), BDAP-only, and the driver extraction restructures its surroundings so it would not drop back in cleanly anyway. |
| 6 | **YAML audience** | **One operator** (the owner). Validation stays permissive: `cv` range guards, hard errors only for structurally invalid values (min > max), warnings elsewhere. No hostile-input hardening. |
| 7 | **Testing** | **Yes** — host-side test harness (`tests/`, plain g++, no ESPHome dependency for extracted logic). Every phase adds tests for what it extracts. |

**Standing constraint:** only `vevor_2kw_v2` is bench-accessible. Every phase must
end flashable with Vevor behavior **unchanged**, verified by host tests + a bench
flash. BDAP changes are compile-verified only (stays parked per ISSUES.md #1).

## Target structure

```
┌──────────────────────────────────────────────────────┐
│ HeaterUart (thin): entity registry, publishing,       │
│ mode select, room-sensor health                      │
├──────────────────────────────────────────────────────┤
│ ControlPolicy (pure C++): thermostat, AUTO/HEAT/     │  normalized output
│ manual-HEAT ramp, auto-shutdown/standby/restart      │  domain 0.0–1.0
├──────────────────────────────────────────────────────┤
│ SafetyPolicy (pure C++): HX stack, ambient limit,    │  gated by
│ cooldown monitor — on unless `ecm_owned_safety` &&  │  capability
│ YAML opt-out                                        │
├──────────────────────────────────────────────────────┤
│ ProtocolDriver (abstract):                          │
│   BdapDriver | VevorDriver | future…                 │  framing, cadence,
│ start/stop/set_output(0..1)/telemetry                │  encode/decode
├──────────────────────────────────────────────────────┤
│ HeaterProfile (data): identity+protocol+capabilities│  compiled presets,
│ (compiled) + Tuning (overridable via YAML)           │  YAML §Phase 4
└──────────────────────────────────────────────────────┘
```

The Hz↔level bridge disappears behind the **normalized output domain 0.0–1.0**
(`heater_output.h`): policy computes a fuel fraction, each driver encodes it
(Hz for BDAP, level 1–10 for Vevor, whatever heater #7 speaks).

## Phases

### Phase 0 — host test harness ● (2026-10-24, 16 tests / 232 checks)
`tests/` with a header-only micro-framework (no network/deps) + `make -C tests`.
Guards: every subsequent phase's extracted logic lands with tests in the same
commit. Verification: `make -C tests` green.

### Phase 1 — de-duplication + dead-code removal ● (code done 2026-10-24; bench flash pending)
**1a. Single shared output mapping.** ✔ All four hand-copied level↔Hz↔ratio
conversions collapsed into `heater_output.h` free functions (host-tested);
ESPHome compile verified clean by the owner.
**1b. LCD injection removal.** ✔ Tag `lcd-injection-final` set, 48-byte
sniff/injection path and the `standalone_mode:` YAML key deleted, ambient-limit
check and priming timeout unified into `standalone_loop()` before the protocol
dispatch (`PRIMING_DURATION_MS` named constant).
Verification: tests green ✔, owner ESPHome compile ☐, bench flash ☐.

### Phase 2 — ProtocolDriver extraction ● (code done 2026-10-24; bench flash pending)
`BdapDriver` / `VevorDriver` behind one interface (`heater_protocol_driver.h`):
`begin()`, `loop()`, `request_start()/request_stop()/pulse_stop()`,
`on_requested()`, `set_output` via ControlInputs (normalized 0..1), telemetry
struct, error/run-state maps. `HeaterUart` is now a thin facade (entity
registry, publishing, mode intent, room-sensor guard, priming) + the
thermostat policy (Phase 3 moves it out). Drivers own framing, echo discard,
resync, checksums. Host tests: fake ByteStream, frame-layout goldens, TX state
machines, decode rules (37 tests / 382 checks).
Verification: tests green ✔, stub syntax check of facade+entities ✔, owner
ESPHome compile ☐, bench flash ☐.

### Phase 3 — SafetyPolicy / ControlPolicy extraction ◐ (heater_policies landed; bench flash pending)
- `heater_policies.h/.cpp` (pure C++, host-tested):
  - `Thermostat`: the whole `set_heater_mode()`-time and periodic machine —
    standby check, state-4 warm-up ramp, AUTO three-phase, HEAT modulating,
    manual-HEAT ramp-down, auto-shutdown/standby/restart — moved verbatim;
    the 7 subtly-diverging aggressiveness copies are now one set of named
    constants.
  - `CombustionSafety`: ambient HEAT-limit, BDAP fan matching (moved out of
    the driver — the driver now only encodes `ControlInputs.fan_rpm`),
    cooldown completion monitor.
  - `RoomSensorGuard`: NaN rejection, glitch-step rejection, boot 3-sample
    consensus, staleness — pure and directly tested.
- `standalone_mode:` guard removal fallout (Phase 1b) is complete.
Recovery/decisions recorded in ISSUES.md.
Remaining for this phase: owner ESPHome compile + bench flash (behavior
must be identical), then Phase 4 opens the loop knobs to YAML.

### Phase 4 — Tuning → YAML overrides ● (code done 2026-10-24; bench flash pending)
Done: per-key optional YAML overrides over the compiled preset, in two
groups: (a) profile tuning (pump freq min/max/step/initial/prime, fan RPMs,
HX thresholds, fan_sensor/glow_power bytes, temp range) — setters mutate the
copied profile before setup(); (b) thermostat policy (adjust interval,
priming duration, gap/overshoot thresholds + multipliers) — facade members
threaded through Thermostat::Params. `validate_profile_invariants()` (pure,
host-tested) reverts invariant-breaking combos to the preset at boot with a
warning. NOT exposed: pump_epsilon (cannot discriminate — pinned by test),
model-identity flags (heat_mode_manual, ecm_owned_safety, has_*_control),
protocol timing (drivers), RoomSensorGuard constants (sensor facts).
Verification: tests 81/586 ✔, stub-check ✔, owner compile ☐, bench ☐.

### Phase 5 — new-heater checklist ☐ (the maintainability payoff)
Adding heater N = one profile factory (+ YAML overrides), one enum entry,
one `cv.enum` line — and a `ProtocolDriver` **only if** the protocol is new.
Write this checklist into `README.md` when Phase 2 lands.

## Phase gates

- Every phase ends compilable and bench-flashable on the Vevor, behavior
  unchanged (regression risk is concentrated in BDAP, which is parked).
- No phase starts before the previous one's tests are green.
- `make -C tests` runs in CI-less peace of mind; run it before every commit.

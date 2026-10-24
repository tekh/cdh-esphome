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

### Phase 0 — host test harness ●
`tests/` with a header-only micro-framework (no network/deps) + `make -C tests`.
Guards: every subsequent phase's extracted logic lands with tests in the same
commit. Verification: `make -C tests` green.

### Phase 1 — de-duplication + dead-code removal ◐
**1a. Single shared output mapping.** All four hand-copied level↔Hz↔ratio
conversions collapse into `heater_output.h` free functions (host-testable).
**1b. LCD injection removal.** Tag `lcd-injection-final`, then delete the
48-byte sniff/injection path (and the `standalone_mode:` YAML key — standalone
becomes the only mode). Unify the ambient-limit check and priming timeout
duplicated across both protocol loops.
Verification: tests green, bench flash of `lab-heater-butler.yaml` behaves
identically on the Vevor.

### Phase 2 — ProtocolDriver extraction ☐
Extract `BdapDriver` / `VevorDriver` behind one interface: `begin()`,
`loop()`, `start()`, `stop()`, `set_output(float)`, `set_priming(bool)`,
telemetry struct, error/run-state maps. `HeaterUart` keeps a driver reference
chosen by `profile_.protocol` and loses all `if (protocol == …)` branches.
Drivers own framing, echo discard, resync, checksums — exactly the constants
that stay compiled (§ "anti-goal" in the analysis). Each driver's
encode/decode gets direct host tests (feed recorded frames, compare fields).

### Phase 3 — SafetyPolicy / ControlPolicy extraction ☐
- `SafetyPolicy`: HX floor/too-hot/critical, ambient limit, cooldown gate.
  Active on every profile by default; YAML `firmware_safety_oversight: false`
  accepted **only** when `profile_.ecm_owned_safety` is true (cv-enforced).
- `ControlPolicy`: the whole thermostat state machine lifted out of
  `HeaterUart`, pure C++ (inputs: telemetry + room temp + mode + tuning;
  output: normalized fuel fraction + on/off requests). This is where the
  hardcoded §3.3 tuneables (aggressiveness multipliers, adjust interval,
  priming duration, DHT22-guard constants) move into `Tuning` — as *compiled
  presets first*, YAML exposure is Phase 4.
- Room-sensor health guard (glitch/boot-consensus/staleness) becomes its own
  small pure class with direct tests.
Verification: heavy host tests (thermostat phases, manual-HEAT ramp,
auto-shutdown/restart, safety overrides, sensor guards) + bench flash.

### Phase 4 — Tuning → YAML overrides (Option A) ☐
Every `Tuning` key becomes an optional YAML override on top of the selected
preset (`overrides:` block or flat keys). C++ setters applied only for keys
present; members default to profile values. Structure errors (min ≥ max)
hard-fail in cv; out-of-band values warn in logs. `number.py` defaults come
from the active profile instead of disagreeing with it.

### Phase 5 — new-heater checklist ☐ (the maintainability payoff)
Adding heater N = one profile factory (+ YAML overrides), one enum entry,
one `cv.enum` line — and a `ProtocolDriver` **only if** the protocol is new.
Write this checklist into `README.md` when Phase 2 lands.

## Phase gates

- Every phase ends compilable and bench-flashable on the Vevor, behavior
  unchanged (regression risk is concentrated in BDAP, which is parked).
- No phase starts before the previous one's tests are green.
- `make -C tests` runs in CI-less peace of mind; run it before every commit.

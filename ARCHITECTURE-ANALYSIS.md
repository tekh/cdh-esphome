# Architecture Analysis: Multi-Heater Support at a Crossroad

**Branch:** analysed on `arch_analysis`, now `main` (renamed in 2026-10-24) · **Date:** 2026-04-19 · **Base:** `vevor_2kw` @ 7f6482b
**Scope:** top-to-bottom review of `components/heater_uart/` with two questions in mind:
(1) does the current design survive N more heaters, and (2) what would it take to move
**all tuneables into the YAML**?

> **Status: fully superseded — `ROADMAP.md` executed, all six phases complete and
> device-verified (2026-10-24).** Everything below describes the *pre-refactor*
> codebase (the god class, the four level↔Hz copies, the missing harness); it is
> kept as the "why" for every seam that now exists, and for the decision record
> in §6 → ROADMAP.md decisions #1-7.

---

## 1. What the codebase is today

### 1.1 Three axes are conflated in one enum

`HeaterModel` currently encodes three different things:

| Axis | Examples | What it actually selects |
|---|---|---|
| Hardware unit | `JEABONG_8KW`, `VEVOR_2KW` | tuning numbers + capability flags |
| Link protocol | BDAP vs VEVOR_UART (`profile_.protocol`) | framing, baud, cadence, encode/decode |
| **Behavior variant** | `VEVOR_2KW_V2` | one bool: `heat_mode_manual` |

`vevor_2kw_v2` is not a heater — it is the same physical unit with different firmware
*policy*. Making policy a model entry works for exactly one flag. Two flags with two
states each already means 4 enum entries per hardware unit; five heaters × a few policy
toggles = combinatorial explosion of "models" that are not models.

### 1.2 The profile struct is doing four jobs

`HeaterProfile` (207 lines) is the single source of truth for tuning — this is the
project's best architectural decision so far. But the struct mixes:

1. **Identity/protocol** (`name`, `protocol`, `uart_baud_rate`, `uart_inverted`)
2. **Capabilities** (`has_pump_hz_control`, `has_power_level_control`, `has_fan_rpm_control`)
3. **Policy** (`heat_mode_manual`)
4. **Tuning numbers** (pump Hz, fan RPM, HX thresholds, temp range, priming rate)

These change for different reasons and at different rates. Protocol facts are
*discovered* (reverse engineering), tuning numbers are *bench-validated per unit*,
policy is a *user/owner decision*, capabilities are *fixed by hardware*. Lumping them
means every new axis touches the same struct and the same profile factory.

### 1.3 Semantic overload: pump Hz on the Vevor is not pump Hz

The shared thermostat (`thermostat_control()`) works entirely in the pump-frequency
domain. On the Vevor there is no pump-Hz command — the TX frame carries a power level
1–10. The design bridges this by treating `pump_freq_min/max` as the *encoding range*
for levels: level = 1 + round(ratio·9). Consequences:

- On `vevor_2kw`, `pump_freq_min/max` are **not physical pump frequencies** — they are
  the ends of an abstract dial. The ignition HX ramp, soot floor, etc. all manipulate
  this fictional Hz value and only at the TX boundary does it become level 1–10.
- The level↔Hz mapping is implemented **four times**: `set_power_level()`,
  `get_power_level_display()`, the START log line in `send_vevor_frame()`, and
  `frame[8]` in `send_vevor_frame()`. They must stay in sync by hand.
- A third heater with, say, 5 levels, a percentage, or a watt target breaks the
  assumption baked into `1 + ratio * 9`.

The Hz-domain unification *worked* for two heaters, but it is a bridge, not a
foundation. The honest abstraction is a generic **normalized output domain 0.0–1.0**
(or explicit "output command" object) with per-protocol encoders.

### 1.4 `HeaterUart` is a god class

One class, ~70 member variables, 1522 lines, responsible for:

- LCD/injection-mode frame sniffing (BDAP-only, largely untested — ISSUES.md #1)
- BDAP standalone TX/RX (echo discard, half-duplex timing)
- VEVOR standalone TX/RX (state machine, additive checksum, resync)
- The entire thermostat + safety policy (AUTO/HEAT/manual-HEAT/auto-shutdown/standby)
- Room-sensor plausibility filtering (glitch guard, boot consensus, staleness)
- Fuel priming
- Sensor/entity publishing and state sync

Protocol dispatch is `if (profile_.protocol == VEVOR_UART) vevor_loop(); return;` at
the top of `standalone_loop()` plus scattered `if` checks (priming refused in
`start_priming()`, ambient limit checked *separately and duplicated* in both
`standalone_loop()` and `vevor_loop()`). With a third protocol this becomes switch
spaghetti across 6+ methods.

### 1.5 Safety policy is one-size-fits-all — but the heaters are not

The user's framing is exactly right and it shows in the code:

- **8 kW (BDAP):** firmware owns combustion safety. `build_tx_frame()` computes the fan
  RPM to match fuel; `thermostat_control()` enforces the HX soot floor (`hx_temp_low`),
  HX too-hot (`hx_temp_high`), HX critical stop, ignition ramp, cooldown purge. Remove
  the firmware's oversight and the hardware can soot up or overheat.
- **Vevor 2 kW:** the ECM owns combustion safety. The host sends level + requested
  state; the heater self-primes, self-derates, self-protects (E-codes). The firmware
  *also* runs the full HX-threshold stack on top of it.

Right now this duplication is mostly harmless (defense in depth), but it has real
costs: the Vevor profile carries `fan_rpm_min/max`, `ignition_fan_rpm`,
`cooldown_fan_rpm` — values that are **never used** by the Vevor path (no fan command
exists) and exist only so the struct is fillable. Bench-validating "provisional"
values that cannot affect behavior wastes effort; conversely someone tuning
`hx_temp_low` on a Vevor may believe they are protecting it when the ECM is already
doing that (or when the threshold is simply wrong for that unit — the Vevor HX temp is
a signed ×0.1 °C int16 with a completely different range than the 8 kW's raw uint16).

**Safety policy should be a capability, not a constant.** Something like
`firmware_owns_combustion_safety: true/false` deciding whether the HX stack, fan
matching, and ignition ramp logic engage at all.

---

## 2. Scaling assessment: does heater #3 kill this?

**Verdict: parameter scaling is solved; structure scaling is not.**

What scales well today:
- Adding a heater that speaks an *existing* protocol and wants *existing* behavior:
  one profile factory + one enum entry + one `cv.enum` line. Genuinely cheap, and the
  capability flags correctly refuse inapplicable dials at runtime. Good design.
- The sensor registry (string-keyed maps) means adding sensors never touches the C++
  class signature.

What breaks at 3+ heaters / 3+ protocols / more policy variants:
1. **Protocol code** — every protocol adds branches inside the one 1522-line class
   (loop, TX build, RX parse, priming gate, echo handling, resync, error map). Linear
   growth × high per-change risk, because all heaters share the file.
2. **Policy variants as models** — see §1.1. Combinatorial.
3. **Output-domain assumptions** — see §1.3. The `1..10` level and Hz-bridge are
   hard-coded in four places.
4. **Safety policy uniformity** — see §1.5. A self-protecting ECM heater still drags
   the full HX stack, and "tuning" it means editing thresholds the heater doesn't need.
5. **Testing** — there is no host-side test harness at all; every change is validated
   by bench flash. As permutation count grows, this is the actual unmaintainability
   risk, arguably ahead of the code structure itself.

So: not impossible to maintain, but the *seam* is missing. The code already has three
clean responsibilities (protocol driver, control policy, tuning data) that happen to
be compiled into one class. Extracting them is mostly mechanical and can be done
incrementally.

### Proposed seams (for discussion, not decided)

```
┌─────────────────────────────────────────────────────┐
│ HeaterUart (thin): entities, publishing, mode state │
├─────────────────────────────────────────────────────┤
│ ControlPolicy: thermostat, AUTO/HEAT behavior,      │  heater-agnostic,
│ room-sensor health, auto-shutdown/standby           │  works in normalized output
├─────────────────────────────────────────────────────┤
│ SafetyPolicy: HX stack, ambient limit, cooldown     │  enabled by capability flag
├─────────────────────────────────────────────────────┤
│ ProtocolDriver (interface):                         │
│   BdapDriver | VevorDriver | <future>               │  framing, cadence, encode/
│ start/stop/set_output(level 0-1)/get_telemetry      │  decode, echo, resync
├─────────────────────────────────────────────────────┤
│ HeaterTuning (data): per-model numbers + capability │  defaults compiled,
│ flags + policy flags                                │  overridable from YAML
└─────────────────────────────────────────────────────┘
```

Key idea: the interface between policy and protocol becomes `set_output(float 0..1)` +
normalized telemetry — the Hz-vs-level hack disappears behind it.

---

## 3. Tunables inventory: what lives where today

### 3.1 In `heater_profile.h` (compiled, per-model) — the "profile" values
pump freq min/max/step/initial/prime, fan RPM min/max/ignition/cooldown,
HX low/target/high/critical/cooldown-target, fan_sensor byte, glow_power byte,
temp min/max, protocol, baud, inversion, capability flags, `heat_mode_manual`.

**Any change here = edit C++ header + rebuild.** This is the main blocker for the
"all tuneables in YAML" goal.

### 3.2 In YAML already (good)
`heater_model`, `standalone_mode`, `operating_voltage`, `temperature_sensor`,
`temperature_backoff`, `altitude`, `auto_shutdown_overshoot`,
`auto_shutdown_hysteresis`, `approach_threshold`, `ambient_heat_limit`, `update_interval`.

### 3.3 Hardcoded in `heater_uart.cpp/h` — neither profile nor YAML

| Value | Where | Note |
|---|---|---|
| Thermostat aggressiveness multipliers 3×/2× and gap thresholds 5.0/2.0/1.0/0.5 °C | `thermostat_control()`, **7 separate occurrences** | Behavior tuning; the copies already differ subtly between branches — drift risk |
| Pump adjust interval 5 s | `PUMP_ADJUST_INTERVAL_MS` | Thermostat cadence |
| Priming duration 60 s | **duplicated** in BDAP loop (l.588) and Vevor loop (l.1258) | |
| Room-sensor physics guard: max step 0.5 °C, boot spread 2.0 °C, 3 boot samples, ≥5 s spacing, 10 min staleness | `heater_uart.h` | DHT22-specific |
| Setpoint rounding to 0.5 °C | `set_desired_temperature()` | Granularity |
| PUMP_EPSILON 0.05 Hz | auto-shutdown check | |
| TX cadence 1 Hz, RX timeout 200 ms, Vevor partial-frame timeout 300 ms, echo discard 15 ms, inject timeout 50 ms | protocol layer | Arguably should stay compiled (protocol facts, not user tuning) |
| ISA air-density constants | `air_density_ratio()` | Physics — keep in code |
| Entity min/max defaults in `number.py` (temp 8–35, pump 1–6) | don't match profile (0–30, 1.3–5.5); C++ clamps anyway | A third place the same numbers live — confusing |

**Finding:** "profile-only tuning" is already leaky — significant behavior tuning
(thermostat aggressiveness, cadences, priming, sensor-guard thresholds) is hardcoded
in the .cpp, some of it duplicated between the two protocol paths. Moving the profile
to YAML without also extracting these just moves half the problem.

---

## 4. Options for the YAML goal

### Option A — compiled presets + YAML overrides (recommended direction, for discussion)
`heater_model:` still selects a compiled, bench-validated preset; every tuning key
becomes individually overridable in YAML:

```yaml
heater_uart:
  heater_model: vevor_2kw
  overrides:            # all optional, all validated, all fall back to the preset
    pump_freq_min: 0.6
    hx_temp_critical: 210
    pump_adjust_interval: 3s
    thermostat_aggressiveness: ...   # once extracted from the .cpp
```

- C++: setters applied in `setup()` only for keys present in config; members default to
  profile values. Straightforward ESPHome pattern.
- Python `cv` does range validation; cross-field checks (e.g. `hx_low < hx_target <
  hx_high < hx_critical`) validated in a final validator so a typo can't remove a
  safety net silently.
- Presets remain the *safe default*; YAML is for refinement, not definition.

### Option B — fully YAML-defined profiles (presets moved to Python)
Maximum flexibility, but: every safety-critical number must be re-validated in Python,
nothing is safe-by-default, and the bench-validated presets live in a language with no
compiler checks. Higher risk for marginal gain — the set of supported heaters is
small and known.

### Option C — do the seam split first (§2), then A
Reordering: extract ProtocolDriver / Policy / Tuning, *then* open the tuning struct to
YAML. Costs one extra refactor pass; buys a much smaller diff surface for every future
heater, and YAML overrides then have a clean, validated target struct (the §3.3
hardcodes fall out of the policy extraction naturally).

### Anti-goal to keep in mind
Not every constant should be user-facing. Protocol framing, echo timing, checksums,
ISA physics belong to the driver layer and should stay compiled. YAML should expose
*tuning and policy*, not internals. Over-exposing would create configs that look
legitimate but quietly break a heater.

---

## 5. Risk register (independent of the chosen option)

1. **Unvalidated 8 kW path** (ISSUES.md #1) — any restructure ships blind for BDAP
   until a bench device exists. Regression risk concentrated exactly where the
   richest control logic lives.
2. **Four copies of the level↔Hz mapping** (§1.3) — already a drift hazard; any
   restructure should collapse them into one function first.
3. **Duplicated logic between protocol paths** — ambient limit check, priming
   timeout. Copy-paste divergence has begun.
4. **No tests** — the thermostat is now subtle (AUTO 3-phase, manual-HEAT ramp-down,
  boot consensus). A host-side harness over the extracted ControlPolicy would let
  heaters be added without bench-flashing every permutation. Probably the single
  highest-leverage investment.
5. **`number.py` defaults vs profile mismatch** — clamped in C++, so cosmetic, but it
  misleads readers.

---

## 6. Open questions (to decide the direction)

1. **Roadmap:** how many heaters, realistically? All Vevor-class (ECM-protected) or
   more dumb-BDAP-like units where firmware owns safety?
2. **Direction:** A (preset + overrides), B (full YAML), or C (split first, then A)?
3. **Safety capability:** introduce `firmware_owns_combustion_safety` (or similar) so
   the HX/fan/ignition stack engages only where the firmware is actually the
   safety authority — keeping defense-in-depth on the Vevor as an explicit opt-in?
4. **Policy variants:** replace `vevor_2kw_v2`-style model entries with a YAML policy
   key (e.g. `heat_mode: manual`), or keep model entries as one-line reverts?
5. **LCD injection mode:** still a target, or is standalone the only mode that
   matters going forward? Dropping injection would simplify the split noticeably.
6. **Audience of the YAML:** only you (relaxed validation, trust), or should a
   stranger's config be guaranteed safe (strict cross-field validation, hard
   errors on out-of-band values)?
7. **Testing appetite:** worth building a host-side test harness for the extracted
   policy as part of this, or is bench validation acceptable for the foreseeable
   set of heaters?

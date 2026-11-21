# ISSUES

Evidence ledger for this repo. Every item states what is known, what is missing
and what would close it. Protocol evidence and bench data live in
`@notes/vevor-protocol.md` (link it, never duplicate it); per-model behaviour
notes live in `docs/heater-notes/`. This file is the *issue* ledger, not a model
manual.

## Item IDs are scoped, and they are stable

| Prefix | Scope | Section |
|---|---|---|
| `CF-` | facade / policy / deployment - applies to every heater | Common |
| `V-`  | the Vevor XMZ-D2 2 kW (`vevor_2kw`, `vevor_2kw_v2`) | Vevor |
| `BD-` | the BDAP Jeabong 8 kW (`jeabong_8kw`) | BDAP |

The number inside an ID is the **original ledger number**, kept so git history,
commit bodies and older notes still resolve. An ID is never reused or
renumbered, so gaps in a sequence are normal. Cite an item as `ISSUES V-22`
(no `#`), a range as `ISSUES V-5...V-10`. A new item takes the next free number
in its scope - the index below is the fast path, the sections are in ID order.

Status markers: **OPEN** (actionable) - **PARKED** (deliberately deferred) -
**DECIDED** (settled, kept for context) - **FIXED** / **DEPLOYED** / **CLOSED**
(settled, with the evidence that closed it).

## Index

| ID | State | Item |
|---|---|---|
| `CF-3` | DECIDED | one fuel dial per heater, enforced at runtime |
| `CF-5` | DECIDED 2026-10-24 | LCD injection mode removed (recover: tag `lcd-injection-final`) |
| `CF-6` | DECIDED 2026-10-24 | combustion oversight is owner-selectable per device |
| `CF-11` | CLOSED 2026-10-24 | API key rotated and moved to secrets |
| `CF-12` | CHANGED 2026-10-20 | auto band: shutdown at the setpoint, re-engage at target - 1.0 |
| `CF-13` | FIXED 2026-10-20 | half-degree setpoints truncated in the display |
| `CF-14` | DEPLOYED 2026-10-21 | room-sensor fault cutoff (10 min blind -> stop) |
| `CF-15` | FIXED 2026-10-21 | DHT22 glitch samples drove the thermostat |
| `CF-16` | DEPLOYED 2026-10-24 | boot baseline needs a 3-sample consensus |
| `CF-17` | IMPLEMENTED 2026-10-24 | multi-heater refactor complete (Phases 0-5) |
| `CF-18` | DEPLOYED 2026-10-27 | AUTO would not restart while the bus was silent |
| `CF-20` | FIXED 2026-11-17 | HEAT->AUTO stop criterion killed a running burn |
| `CF-21` | ADDED 2026-11-17 | AUTO restart offset is a live slider |
| `BD-1` | PARKED | no BDAP device target: the 8 kW path is untested |
| `BD-2` | PARKED | 8 kW fan-RPM dial: protocol field exists, policy undecided |
| `BD-4` | DECIDED | Fuel Prime is BDAP-only |
| `V-5` | FIXED 2026-10-12 | parser no longer decodes byte 13 |
| `V-6` | PARTLY FIXED 2026-10-12 | `fan_voltage` decodes (byte 15); glow vs temperature not separated |
| `V-7` | DECIDED + IMPLEMENTED 2026-10-12 | HEAT starts at full power, ramps down to the user level |
| `V-8` | OPEN | `hx_temp_low = 145` sits above the level-1 equilibrium |
| `V-9` | OPEN | long-hold and transition behaviour untested |
| `V-10` | OPEN | level -> output is not calibrated (deliberate) |
| `V-19` | FIXED 2026-11-09 (bench) | fuel was dosed BEFORE the glow plug cycle |
| `V-22` | FIXED 2026-11-20 | E10 cooldown latch blocked a HEAT restart; START is now persistent |

---

## Common - facade, policy, deployment (CF)

Everything here holds for every heater: a driver or profile change that
contradicts one of these is wrong regardless of model.

CF-3. **DECIDED — one fuel dial per heater, enforced at runtime.** BDAP takes a real
   pump frequency (`has_pump_hz_control = true`); the Vevor takes a power level
   (`has_power_level_control = true`). Each dial refuses writes on hardware that
   lacks its control, so the two can never both be live. **The 8kW device YAML must
   declare `type: pump` — that entity is no longer in the 2kW YAML and may look
   "missing" to anyone reading only the current config.**

CF-5. **DECIDED 2026-10-24 — LCD injection mode removed.** The 48-byte sniff/injection
   path (`parse_frame`, `send_command`, `loop()` injection branch, `standalone_mode:`
   YAML key) was deleted: it was BDAP-only, never validated against real hardware
   (no device target, BD-1), and the multi-heater refactor (DESIGN.md, Phase 1b)
   restructures its surroundings so it would not drop back in cleanly. Standalone is
   now the only mode. Recovery: `git tag lcd-injection-final` (the commit just before
   removal). Closes: nothing to close — abandoned by owner decision, not fixed.

CF-6. **DECIDED 2026-10-24 — Combustion oversight is owner-selectable per device.**
   `heater_profile.h` now carries `ecm_owned_safety` (Vevor-class: true — the ECM
   self-primes, self-derates and reports E-codes; Jeabong/8kW: false — the
   firmware is the low-combustion-temperature authority). A new
   `firmware_safety_oversight:` YAML key (default **true**) removes the
   firmware's pump-modulation safety stack (HX soot floor / too-hot / critical
   stop) on request — but ONLY on `ecm_owned_safety` heaters; on firmware-safety
   hardware the flag is ignored with a warning. The room-sensor staleness gate
   and the ambient HEAT-mode limit are NOT part of the removable stack (owner
   requirements, 2026-10-20) and stay on everywhere. Implementation: Phase 3 of
   the multi-heater refactor (DESIGN.md) — `HeaterUart::thermostat_control()` became the `Thermostat` class
   (pure, host-tested in `tests/`).

CF-11. **CLOSED 2026-10-24 — API key rotated and moved to secrets.**
    The live config references `!secret` for the API encryption key, the
    WiFi credentials and the use_address; values live in an untracked,
    gitignored `secrets.yaml`. The old key that was public in git history has
    been **rotated in HA**, so the historical exposure no longer grants access.
    (The device is LAN-only regardless.) Nothing further to close.

CF-12. **CHANGED 2026-10-20 — Auto-mode band: shutdown at the setpoint, re-engage at target-1.0.**
    Field symptom "heater stays on": `auto_shutdown_overshoot: 0.5` required the
    room to exceed target+0.5 at min pump before shutting off; when the room's
    equilibrium at min power sat below that (observed 18.3-18.4C vs the 18.5C
    threshold on 2026-10-20), the heater idled at min level indefinitely. Now
    `overshoot: 0.0` (shut off as soon as the room crosses the setpoint at min
    pump) and `hysteresis: 1.0` (re-engage at target-1.0, user-chosen).
    Field-verified 2026-10-21: standby->start fired at a 18.4C reading against
    the 18.5C threshold (target 19.5 - 1.0) — that same firing also proved the
    internal target was the float 19.5 despite the display showing 19.0 (see
    CF-13). Shutdown half of the cycle at 19.5 was still in progress.
    (Separately: the two "stays on" runs on the evening of 2026-10-20 were in
    HEAT mode, which by design never auto-shuts off on the v2 profile — v2 HEAT
    is user-owned level. The AUTO-mode margin was the second, real bug.)

CF-13. **FIXED 2026-10-20 — half-degree setpoints truncated in the display.**
    `desired_temperature_value_` was an `int`, so a 19.5 request displayed as
    19.0 (number entity + desired_temperature sensor) while the thermostat
    already ran on the exact float `desired_temp_setting_`. Member is now
    `float`; the getter follows. Wire unchanged: Vevor TX byte 4 stays integer
    °C (rounded), per protocol convention.

CF-14. **DEPLOYED 2026-10-21 — room-sensor fault cutoff.**
    Requirement (user): DHT22 in warning state > 10 min -> heater off. A dead
    sensor leaves the thermostat on a stale last value (the NaN guard holds the
    last good reading), and the 26C ambient limit reads that same stale value —
    i.e. unmonitored burn. Now: no *accepted* room reading for 10 min -> a
    running heater stops immediately (all modes), all auto-start paths are
    blocked, and Auto mode parks in standby so it resumes normally on recovery.
    Also blocks blind starts from a stale cold value. Deployed 2026-10-21
    (rebuild + reflash), live-verified via HA entity freshness (room temp
    last_updated 02:36:01 UTC at a 02:37:02 UTC check, all heater entities
    available). Scope note (superseded by CF-15): staleness originally counted
    non-NaN readings only; the 2026-10-21 glitch guard extended it so rejected
    samples also fail to refresh the timer.

CF-15. **FIXED 2026-10-21 — DHT22 glitch samples drove the thermostat (physics guard).**
    Field incident: an 18.4C reading against a 19.2C trend (a valid-but-garbage
    DHT22 sample after a failed read) tripped a false standby->start in AUTO
    mode. Now every room-sensor sample passes `refresh_room_temperature()` —
    the single ingestion point on the standalone TX path (1 Hz) — before the
    thermostat or the TX frames see it:
    - NaN / warning readings are rejected outright.
    - A non-NaN sample more than `ROOM_TEMP_MAX_STEP` (0.5C) from the last
      accepted value is a glitch: rejected, the last trusted value is held, and
      the staleness timer is deliberately NOT refreshed (observed field rate
      ~0.3C/min, so 0.5C per 10s sensor interval is 10x headroom).
    - Rejections log at most one warning per 60 s; the guard is otherwise
      passive — it can only be observed when a real glitch arrives.
    Consequence for CF-14: a DHT22 that is dead (NaN) *or* stuck emitting
    garbage now both trip the 10-minute fault cutoff. Compile-verified
    (COMPILE_EXIT=0) and deployed 2026-10-21; live-verified via HA entity
    freshness (same check as CF-14).

CF-16. **DEPLOYED 2026-10-24 — boot baseline: one first sample could
    become the guard reference (3-sample consensus).**
    Gap in the CF-15 design: the first valid sample after boot was accepted
    unconditionally (no baseline to compare). A DHT22 misfiring on its first
    conversion after power-on would set the reference for every later step
    check. An HX-temperature cross-reference was evaluated and rejected: the
    HX floor is ~59C (even in standby), leaving no plausibility headroom for a
    room-temp cross-check (user observation). Fix (user-specified): the first
    `ROOM_TEMP_BOOT_SAMPLES` (3) non-NaN samples, at least
    `ROOM_TEMP_BOOT_MIN_SPACING_MS` (5 s) apart so each is a distinct 10-s DHT22
    conversion (~30 s total), must agree within `ROOM_TEMP_BOOT_MAX_SPREAD`
    (2.0C); the baseline is the **median** of the three (a mild outlier inside
    the spread is outvoted). A failed window is discarded and retried with the
    next three samples (self-correcting; persistent failure still hits the
    10-minute cutoff). Rationale: physical drift over 30 s is ~0.05C, so a
    2.0C spread is ~40x the real signal — a false reject is implausible, and
    its cost would only be 30 extra seconds in the safe state. During the
    window `last_valid_room_temp_time_` stays 0, so `is_room_sensor_stale()`
    is true and AUTO keeps the heater off — the safe state right after a
    reboot, when the relay has just de-energized. Residual known limitation:
    a *consistent* bad value (e.g. stuck 0.0) passes consensus — sampling
    alone cannot distinguish it from a true value, and no cross-reference is
    available. Compile-verified (COMPILE_EXIT=0, flash 49.9%) on 2026-10-21;
    deployed with the subsequent 2026-10-24 rebuild/flash (it is in every
    build since).

CF-17. **IMPLEMENTED 2026-10-24 — multi-heater architecture refactor complete
    (refactor Phases 0-5, all device-verified; DESIGN.md).** Landed on `main` (as the
    `arch_analysis` branch, renamed in 2026-10-24; the pre-refactor layout is
    preserved at tag `backup/vevor_2kw-pre-refactor` (the `vevor_2kw` branch,
    retired 2026-11-17) and the old `main` tip is tagged
    `backup/main-pre-replace`). Component layout changed
    significantly: `heater_uart.cpp` is now a facade over `heater_policies.*`
    (Thermostat / CombustionSafety / RoomSensorGuard, pure + host-tested) and
    per-protocol drivers (`bdap_driver.*` / `vevor_driver.*` behind
    `heater_protocol_driver.h`); the fuel-domain conversion is
    `heater_output.h`; every heater tuning value is a profile preset with
    per-key YAML overrides (`validate_profile_invariants()` reverts
    invariant-breaking combos to the preset). Adding heater N is data, not
    code — the "Adding a New Heater" checklist in the README is the contract.
    Host tests: `make -C tests` (81 tests / 586 checks) + `tests/stub_check.sh`
    run before every commit. Behaviour on the bench unit (vevor_2kw_v2) was
    verified unchanged at every phase's flash. Old layout recoverable at tag
    `backup/vevor_2kw-pre-refactor` (branch `vevor_2kw` retired 2026-11-17) and
    tag `lcd-injection-final` (LCD path).

CF-18. **DEPLOYED 2026-10-27 — AUTO would not restart while the Vevor bus was silent
    (frame-starved off-state recheck).**
    Field symptom: heater in AUTO, auto-shutdown active, room 18.7 C, target 21.0 C,
    hysteresis 1.0 -> restart threshold 20.0 C. The room sat 1.3 C below the
    threshold and the heater never started, including after raising the setpoint.
    Diagnosis: every telemetry mirror (fan_speed, fan_voltage, HX temp, run_state,
    supply_voltage) had been frozen for ~38 min while the DHT22 updated every
    ~1 min. The Vevor ECM stops answering status polls once fully off, so
    `driver_->loop()` never returned a fresh frame, `on_fresh_frame()` /
    `Thermostat::control()` never ran, and the auto-restart check never executed.
    The room-sensor guard was healthy — this was NOT a DHT22 fault.
    Fix: the standby->start and auto-restart checks were factored into
    `Thermostat::recheck_off_state()` (shared with `control()`, same
    `standby_threshold()`, same no-blind-start gate) and `loop()` now runs it on
    its own 1 Hz timer while off in AUTO, independent of RX frames, so a cooling
    room or a raised setpoint starts the heater within ~1 s.
    **New entity:** `binary_sensor.<device>_temperature_probe_failed` (device
    class `problem`) — ON when the room probe is blind (no accepted reading for
    >10 min) or still establishing its boot baseline (~30 s after reboot), the
    exact condition that suppresses AUTO restarts. Boot-time ON flips to OFF once
    the 3-sample consensus locks (confirmed live on the device). Host tests:
    86 tests / 597 checks (5 new recheck tests), `stub_check.sh` clean,
    `esphome config` valid. Deployed 2026-10-27 (md5-verified copy to hoas;
    owner compiled, flashed and confirmed the probe sensor and restart work).

CF-20. **FIXED 2026-11-17 — switching HEAT -> AUTO stopped a heater that was still
    under the setpoint.** Field report: heater burning in Heat mode, room below
    the desired temperature, selecting Auto sent STOP (cooldown, heater off).
    Cause: `Thermostat::enter_mode(AUTO)` used `standby_threshold()` —
    `target - auto_shutdown_hysteresis`, a whole 1.0 °C *below* the setpoint —
    as its stop condition whenever the heater was running. Any room reading in
    `[target-1.0, target)` therefore satisfied "too warm -> standby" and the
    mode switch shut the burn down, while the periodic machine would have kept
    heating (its shutdown criterion is pump at min AND room > target +
    `auto_shutdown_overshoot`, i.e. at/above the setpoint — CF-12). Two
    different thresholds for the same decision.
    Fix: mode entry splits on `running_intent` before touching temperature —
    a running heater (or a start already in flight) is *adopted*: no STOP, no
    new START, `in_standby_` cleared, and `control()` takes over at the next
    frame; it stops only on the shutdown criterion (room > target + overshoot),
    which parks the state in `in_auto_shutdown_` so the normal auto-restart
    (target - hysteresis, CF-12/CF-18) re-engages after cooldown. Not-running
    entry is unchanged (start below target-hysteresis, otherwise standby). Side
    benefit: a mode switch during ignition no longer cancels the start frame.
    Host tests: `enter_auto_while_running_below_target_keeps_burning`,
    `enter_auto_while_running_at_threshold_keeps_burning`,
    `enter_auto_adopts_start_in_flight`, and
    `enter_auto_warm_running_stops_into_auto_shutdown` (rewritten: it now
    asserts the `auto_shutdown_active` path instead of standby). 98 tests /
    645 checks, `stub_check.sh` clean.

CF-21. **ADDED 2026-11-17 — AUTO restart offset is a live slider.** Request: control
    the auto-mode restart offset (the 1.0 °C re-engage hysteresis) from HA,
    0.0-2.0 in 0.1 °C steps. New number entity `type: restart_offset`
    (`HeaterRestartOffsetNumber`, `heater_restart_offset_number.{h,cpp}`) writes
    through `HeaterUart::set_auto_shutdown_hysteresis()` — the same member
    `build_params()` hands to `Thermostat::Params`, so there is one source of
    truth and no new policy code. YAML `auto_shutdown_hysteresis:` stays the boot
    default; `update()` republishes the live value every cycle so a reboot (or a
    YAML change) cannot leave the slider lying. Rendered as an HA slider
    (`mode: slider`, unit °C); the dial's span is YAML-configurable
    (`min_value`/`max_value`/`step`) and `setup()` warns if the configured
    hysteresis falls outside it.
    Deliberately **not persisted** — the component still has no Preferences
    store (firmware rule: nothing survives a reboot; ESPHome 2026.10.2 `number`
    has no `restore_value`). A power blip therefore returns the offset to the
    YAML value, which is the documented behaviour.
    Band arithmetic for reference: shutdown at `target + auto_shutdown_overshoot`
    (0.0 on this unit = the setpoint, CF-12), restart at `target - offset`. So
    the two keys are the two halves of the dead band. 0.0 is accepted (the
    request) but collapses the band into setpoint short-cycling; `control()`
    warns below 0.2 and names the ignition-cycle cost (glow plug life, and the
    ECM's failed-start latch). Useful range 0.2-1.0.
    Verified: `esphome config` **and a full `esphome compile`** on the hoas
    add-on (ESPHome 2026.10.2, image built, 49.7 % flash) with the deployed
    files; host tests 99 / 649 with
    `restart_offset_dial_moves_the_reengage_threshold` (same room, offsets 1.0
    vs 2.0 -> start vs parked; 0.2 offset on the auto-restart path);
    `stub_check.sh` clean (its `number` stub grew `NumberTraits`/`traits`).

---

## 8kW / jeabong_8kw (BDAP) - PARKED 2026-10-12, revisit later (BD)

BD-1. **PARKED — no BDAP device target exists, so the whole 8kW path is untested.**
   No YAML in this repo sets `heater_model: jeabong_8kw`; the only device config
   (`lab-heater-butler.yaml`) runs `vevor_2kw_v2` at 4800 baud. Any 8kW change
   ships unvalidated until a device config exists (25000 baud, non-inverted pins,
   `heater_model: jeabong_8kw`). Closes when: an 8kW device config is committed
   and flashed against real hardware.

BD-2. **PARKED — 8kW fan-RPM dial: real protocol field, no entity, policy undecided.**
   The BDAP frame carries a genuine fan command (`build_frame()` frame[7..10] in
   `bdap_driver.cpp`; `build_tx_frame()` pre-refactor),
   min = max = fixed RPM) and the profile sets `has_fan_rpm_control = true`. What
   is missing is the safety policy, because the firmware currently *computes* the
   fan (scaled with the fuel rate in normal run, forced to ignition RPM during
   startup, cooldown RPM during purge). Two candidate behaviours:
   - clamped manual (recommended): user RPM honoured, bounded to a band around the
     computed value (e.g. ±25 %), and never applied during ignition/cooldown —
     airflow stays matched to fuel, so no soot/overheat risk;
   - full manual: exactly what the user sets, with only the HX cutouts as backstop.
   Closes when: the clamp policy is chosen and the entity + override path exist.

BD-4. **DECIDED — Fuel Prime is BDAP-only.** The Vevor layer refuses it (no documented
   prime command; that heater self-primes during every start). Not a gap.

---

## 2kW / vevor_2kw + vevor_2kw_v2 (XMZ-D2) - OPEN (V)

V-5. **FIXED 2026-10-12 — the parser no longer decodes byte 13.** It now decodes the
   slots the capture proved: **byte 15 → `fan_voltage`**, **byte 27 → glow current**
   (candidate), and **bytes 24-26 → `glow_a`/`glow_b`/`glow_c`** published raw and
   unit-less as "unverified". Verified live on the bench: during ignition glow
   a/b/c read 99/92/83 with current 11.0 A, all four returning to 0 when the glow
   extinguished; while running, fan voltage read 11.0 V at 3692 RPM.
   Evidence: 649-frame full cycle capture (the raw log is no longer retained),
   incl. the disproof of byte 13 (0 through the whole glow-preheat phase; 78-128 while
   running with the glow off; only ever {0,78,88,98,108,118,128} — a 10-step
   ladder, and it drifts in the off state with fan 0 and HX flat, i.e. it moves
   with no load → internal/self-measured quantity).
   **Still open:** which of bytes 24-26 is voltage vs temperature can't be
   separated without a second instrument; `glow_plug_voltage` therefore stays
   undecoded (0.0) rather than publishing a guess. Closes when: something
   independent pins the cluster members (amp clamp, DMM on the glow lead).

V-6. **PARTLY FIXED 2026-10-12 — `fan_voltage` now decodes (byte 15, verified reading
   11.0 V at 3692 RPM).** `sensor.lab_heater_glow_plug_voltage` still reads 0.0 and
   is deliberately left undecoded: pointing it at one of the glow-cluster bytes
   would be a guess dressed as a measurement. Closes with V-5.

V-7. **DECIDED + IMPLEMENTED 2026-10-12 — HEAT starts at full power, then ramps down.**
   Requirement (user): "starting from off, changing to heat should have a default of 10
   until the ramp down (close to temperature request)". Implemented on manual-HEAT
   profiles:
   - the ON transition commands **full power** (`pump_freq_max` = level 10) instead of
     the ignition-rate default; non-manual profiles (v1, 8kW) keep `pump_freq_initial`
   - state 4 (warm-up) holds full power by design
   - in state 5: below `target - approach_threshold` → hold full power; in the approach
     band → step down toward the **user's dial value** (the level the burn settles at);
     at/above the request → keep stepping down to the **profile floor** (anti-overshoot)
   - the HX-soot increase stays disabled in v2 HEAT (user owns the fine tuning); the
     HX-too-hot decrease and the HX-critical stop still apply
   Verified live: from Off → Heat the level read 10 through ignition, then stepped
   10 → 1 within ~35 s of entering Running with the room (21.4 °C) above the request
   (20 °C), no errors.
   **New sensor:** `sensor.lab_heater_power_level_actual` (Vevor RX byte 6) reports the
   level the ECM is running at, so the ramp is observable. The Power Level *dial* is the
   request; the sensor is reality. On BDAP the level byte is not decoded, so it stays 0
   there (8kW parked, see items 1-4).
   **Open sub-question:** the approach-band descent floors at the dial value, which is
   currently the stale 1.6 Hz default (reads as level 6). If the desired settle level is
   lower, either set the dial or change the default; flag if the descent should instead
   always run to the profile floor.

V-8. **OPEN — `hx_temp_low = 145` sits above the level-1 equilibrium.**
   Measured: at level 1 this room equilibrates the HX around **154 °C**, i.e. the
   soot-override trigger zone is barely reachable at minimum power (it mattered for
   cold transients and for reductions while HX was low). Candidate: lower to ~110-120.
   Note the override is *disabled by design* in v2 HEAT (user owns the level), so
   this only affects v1 HEAT / AUTO. Closes when: a value is chosen from more
   low-power data.

V-9. **OPEN — long-hold and transition behaviour untested.**
   No soak beyond ~10 minutes. Unknown: does the ECM hold a level for hours, does it
   drift, and are level changes sent *during* a transition queued or dropped? These
   are firmware-determined and season-independent, so they are worth testing once.
   Closes with: a multi-hour hold at a fixed level, and a level change timed inside
   ignition/cooldown.

V-10. **OPEN — level → output is not calibrated, deliberately.**
   A frozen level→kW curve is worthless across seasons (intake air density, room
   ΔT). The durable path is a closed loop on HX temperature plus a live density
   correction using intake-air temperature and baro — the same σ formula already
   used for altitude. The BME280 automatic-altitude upgrade (this item's sensor
   half) is the standing follow-up.

V-19. **FIXED 2026-11-09 (bench-verified) — fuel was dosed BEFORE the glow plug
    cycle.** Field symptom: the 2kW's fuel pump audibly ramped (and then cut)
    before the glow plug ever ran, leaving the chamber wet — white smoke, hard or
    silent start failures. Cause is on our side: the ECM scales its own pre-flame
    dosing from the **requested power level**, and v2 HEAT entry seats
    `pump_freq_max` (level 10) in the START frame **and** every 1 Hz poll, so the
    first frames after a start request ask for maximum fuel.
    Fix: `apply_ignition_fuel_gate()` (`heater_policies.h`, called from
    `HeaterUart::loop()` at the driver boundary). The wire request is held at
      - `pump_freq_pre_ignition` (floor, 0.5 Hz = level 1) while the glow is OFF,
      - `pump_freq_glow` (mid, 2.0 Hz = level 8) once the glow is lit — the ECM
        does not dose *at all* from a level-1 request, so flooring the glow cycle
        starves it (benched: level 3 -> pump 0.0 Hz, ECM aborted the start ~50 s
        in, no flame, err=0; silent, no E10 on the wire),
    and released to the thermostat request on glow-out (glow cluster bytes 24-27
    -> 0, the flame-established signal) or `RUNNING`.
    Measured on the wire, 2026-11-09 bench at level 8 (`/tmp/vevor-coldstart-b6.log`):
    | time | wire request | glow | pump |
    |---|---|---|---|
    | 13:42:30.6 | START, level 01 | off | 0.0 Hz |
    | 13:42:32-42 | polls, level 01 | off | 0.0 Hz — **pre-glow window: no fuel** |
    | 13:42:43.9 | — | **ON 3.0 A** | 0.0 Hz |
    | 13:42:42.7 | polls, level 08 | lit | 0.0 Hz for 68 s |
    | 13:43:51.5 | level 08 | lit | **1.5 Hz dosing starts** |
    | 13:44:23.9 | — | **OFF (flame)** | 1.5 -> 1.7 Hz |
    | 13:44:23.6 | released to level 0A | off | running |
    | 13:44:57.5 | cooldown: level 01 | off | 0.0 Hz |
    Idle polls are unaffected (no start intent). Vevor presets:
    `pump_freq_pre_ignition = pump_freq_min`, `pump_freq_glow = 2.0f`, flag
    `gate_fuel_before_ignition = true`. `jeabong_8kw` keeps the flag false — BDAP
    firmware sequences glow (state 2) then fuel (state 4) itself, so its frames
    stay byte-identical (host-pinned: `ignition_gate_off_profile_passthrough`).
    All three keys are YAML-overridable for A/B; `validate_profile_invariants()`
    reverts out-of-range values to the preset. Host tests 95 / 633,
    `stub_check.sh` clean.
    **Final configuration (2026-11-09 late evening):** `pump_freq_glow = 2.0` (level 8)
    in both the preset and the hoas YAML. Two findings sit behind the two knobs: the
    pre-glow floor (level 1) kills the original defect, and the glow-phase cap (level
    8) bounds the ECM's ramp while a cold chamber refuses to light. Level 10 was tried
    and produced **~1 min of thick white smoke** on a cold start (17:38: glow 12 A,
    dosing from +70 s, pump ramped 1.5 -> 2.6 Hz, flame only at +190 s — HA history in
    `@notes/ignition-gate-2026-11-09.md` §3b); the pre-glow window was clean in that
    run, so only the glow phase was over-fuelling. Level 3 or below starves it (pump
    0.0 Hz, abort ~50 s). The earlier "level 8 refuses to dose" conclusion came from
    lockout-contaminated runs and is withdrawn.
    **Open:** (a) the glow relights during cooldown on **every** stop observed at
    4-7 A for ~30-40 s — routine post-glow burn-off, not a fault (err=0 every time);
    (b) whether the ECM re-engages on its own after parking at the end of a burn, or
    needs a fresh start command — under 24 h soak from 2026-11-09; (c) whether level 8
    caps the cold-start smoke in practice — needs one cold start on the flashed 2.0
    build.
    **The lockout trap (cost most of the afternoon):** after repeated failed
    ignitions the ECM latches — it runs the glow, never drives the pump, and the
    abort window shrinks each attempt (63 s -> 37 s -> ~26 s), with `err=0` the whole
    time. It refuses fuel even at level 10 on the wire. **Only a full 12 V power
    cycle clears it**; rebooting the ESP32 does not. Verified: after a heater+ESP
    power cycle (RX showed V=0.0 / HX=0 at boot) the same firmware and the same
    frames dosed 1.5 Hz at 70 s into the glow and lit cleanly, no white smoke.
    Hypotheses this killed: hot-HX start guard (fails identically at 39-42 °C and
    59 °C), supply sag (rail held 13.0-13.3 V under full burn after the cycle), and
    "the gate breaks dosing" (TX bytes identical in passing and failing runs; the
    ECM ACKed power=10 and still refused). Our component cannot clear an ECM
    lockout — bench discipline is: few attempts, then cut 12 V.

V-22. **FIXED 2026-11-20 — HEAT mode could not restart a heater that faulted (E10)
    and parked in cooldown.** Report: "the heater was changed to heat mode and
    faulted with E10. The status shows cooldown. I can't restart the heater."
    Wire capture (SSE log, `uart_debug` on) shows exactly why:
    - The ECM aborts the start and parks in **raw state 4 (cooldown) with
      `err=1` (E10 - Startup Failure) and `on=1`**, and stays there (50+ min
      observed, fan 0 RPM, HX 24 °C) instead of returning to state 0.
    - Selecting Heat emitted **one** `AA 66 06 … 06` START frame
      (`heater_uart.vevor:137 "sending START"`) and then dropped to
      `AA 66 02 … 08` status polls for ever. `take_command()` only kept
      asserting START while `run_state == RUN_STATE_OFF`; in cooldown it fell
      into the "status poll" arm. This is the same one-shot mistake the STOP
      path already documents ("the Vevor ignores a one-shot 0x06/0x05").
    - Nothing in the control path could recover it either: `control()` pauses on
      any non-running state, the AUTO auto-restart required `!tlm.on` (true
      here), and **HEAT mode had no restart path at all** - mode ON only ever
      sent a start on the select event.
    Fixes (all in this item):
    - `vevor_driver.cpp:take_command()` — a start intent is now **persistent**
      through cooldown: `on_request_ && (OFF || COOLDOWN)` asserts
      `0x06/0x06` every frame until the heater is actually up, mirroring STOP.
      Host test `vevor_start_persists_through_cooldown`.
    - `Thermostat` — **HEAT self-resume** (`recheck_off_state`, mode ON): when
      the heater is not burning, re-request a start after
      `heat_restart_holdoff` (60 s, `Params::heat_restart_holdoff_ms`, YAML
      `heat_restart_holdoff`), for at most `heat_restart_max_attempts` (2,
      YAML `heat_restart_max_attempts`) start requests per failed series - the
      operator's own start counts as attempt 1 - then park with a loud log
      naming the 12 V power cycle, the ECM's own latch requirement. A mode
      re-select starts a fresh series; a confirmed burn (raw state 3 =
      `RUN_STATE_RUNNING`) resets it. Deliberately **not** reset on
      `RUN_STATE_IGNITED` (raw state 2), which the Vevor reports ~10 s into
      every start before the glow lights.
    - **`cooldown` is no longer treated as "running"** (`Thermostat::burning()`)
      in the off-state checks, so the AUTO auto-restart reaches a
      cooldown-parked heater too (host test
      `auto_restart_now_reaches_a_cooldown_parked_heater`).
    - The facade's 1 Hz frame-independent recheck now runs for **HEAT as well as
      AUTO**, not only when `!telemetry().on` in AUTO.
    - Guards on the resume, all host-tested: no resume while the room sensor is
      stale (no blind starts), while the HEAT ambient limit is exceeded (that
      limit is `pulse_stop()`ing every tick), or with `hx_temp >= hx_temp_high`
      (combustion safety outranks comfort).
    Verified: 106 host tests / 686 checks pass, `stub_check.sh` clean, `esphome
    config` + `esphome compile` on the hoas add-on, flashed and bench-tested
    the same evening (`compilation_time 2026-11-20 17:53:28 -0700`).
    **Bench result on a latched ECM: the power cycle is still required.** With
    the heater parked in cooldown/E10, selecting Heat produced 78 consecutive
    `AA 66 06 … 06` frames (persistent START, one per second - the one-shot bug
    is gone on the wire), `HEAT self-resume attempt 2/2` at the 60 s hold-off,
    and `HEAT self-resume gave up after 2 start attempt(s): … Cut the 12 V` at
    120 s. **The ECM never left state 8 / `err=1`, fan 0 RPM, pump 0.0 Hz** -
    it ignores START entirely while its E10 latch is set, so no firmware change
    can restart it. What the fix buys is the diagnosis: the mode parks with a
    log and a reason instead of sitting silently behind a "Cooldown" status.
    The START intent is deliberately left asserted after parking, so the
    operator's 12 V power cycle is met by a START request on the first frame the
    ECM answers - the heater can come back without re-selecting the mode.

---

## Reference

- The fan-probe entity type (`type: fan_probe`) stays in the component but is no
  longer declared in the 2kW YAML — the experiment closed negative (the ECM ignores
  undocumented TX bytes 10-11). Redeclare it in one line to re-test a future unit.
- Revision drift warning: this XMZ-D2 differs from the reference unit in bytes
  8-9, 13, 14 and 52-53. Never port byte meanings between units without checking.


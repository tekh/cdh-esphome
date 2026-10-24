# ISSUES

Open items for this repo, newest evidence-backed facts first. Each item states
what is known, what is missing, and what would close it. Protocol evidence and
bench data live in `@notes/vevor-protocol.md`; do not duplicate it here, link to it.

Status markers: **OPEN** (actionable) · **PARKED** (deliberately deferred) ·
**DECIDED** (settled, kept for context).

---

## 8kW / jeabong_8kw (BDAP) — PARKED 2026-10-12, revisit later

1. **PARKED — no BDAP device target exists, so the whole 8kW path is untested.**
   No YAML in this repo sets `heater_model: jeabong_8kw`; the only device config
   (`lab-heater-butler.yaml`) runs `vevor_2kw_v2` at 4800 baud. Any 8kW change
   ships unvalidated until a device config exists (25000 baud, non-inverted pins,
   `heater_model: jeabong_8kw`). Closes when: an 8kW device config is committed
   and flashed against real hardware.

2. **PARKED — 8kW fan-RPM dial: real protocol field, no entity, policy undecided.**
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

3. **DECIDED — one fuel dial per heater, enforced at runtime.** BDAP takes a real
   pump frequency (`has_pump_hz_control = true`); the Vevor takes a power level
   (`has_power_level_control = true`). Each dial refuses writes on hardware that
   lacks its control, so the two can never both be live. **The 8kW device YAML must
   declare `type: pump` — that entity is no longer in the 2kW YAML and may look
   "missing" to anyone reading only the current config.**

4. **DECIDED — Fuel Prime is BDAP-only.** The Vevor layer refuses it (no documented
   prime command; that heater self-primes during every start). Not a gap.

5. **DECIDED 2026-10-24 — LCD injection mode removed.** The 48-byte sniff/injection
   path (`parse_frame`, `send_command`, `loop()` injection branch, `standalone_mode:`
   YAML key) was deleted: it was BDAP-only, never validated against real hardware
   (no device target, item #1), and the multi-heater refactor (DESIGN.md, Phase 1b)
   restructures its surroundings so it would not drop back in cleanly. Standalone is
   now the only mode. Recovery: `git tag lcd-injection-final` (the commit just before
   removal). Closes: nothing to close — abandoned by owner decision, not fixed.

6. **DECIDED 2026-10-24 — Combustion oversight is owner-selectable per device.**
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

---

## 2kW / vevor_2kw + vevor_2kw_v2 (XMZ-D2) — OPEN

5. **FIXED 2026-10-12 — the parser no longer decodes byte 13.** It now decodes the
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

6. **PARTLY FIXED 2026-10-12 — `fan_voltage` now decodes (byte 15, verified reading
   11.0 V at 3692 RPM).** `sensor.lab_heater_glow_plug_voltage` still reads 0.0 and
   is deliberately left undecoded: pointing it at one of the glow-cluster bytes
   would be a guess dressed as a measurement. Closes with item 5's open half.

7. **DECIDED + IMPLEMENTED 2026-10-12 — HEAT starts at full power, then ramps down.**
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

8. **OPEN — `hx_temp_low = 145` sits above the level-1 equilibrium.**
   Measured: at level 1 this room equilibrates the HX around **154 °C**, i.e. the
   soot-override trigger zone is barely reachable at minimum power (it mattered for
   cold transients and for reductions while HX was low). Candidate: lower to ~110-120.
   Note the override is *disabled by design* in v2 HEAT (user owns the level), so
   this only affects v1 HEAT / AUTO. Closes when: a value is chosen from more
   low-power data.

9. **OPEN — long-hold and transition behaviour untested.**
   No soak beyond ~10 minutes. Unknown: does the ECM hold a level for hours, does it
   drift, and are level changes sent *during* a transition queued or dropped? These
   are firmware-determined and season-independent, so they are worth testing once.
   Closes with: a multi-hour hold at a fixed level, and a level change timed inside
   ignition/cooldown.

10. **OPEN — level → output is not calibrated, deliberately.**
   A frozen level→kW curve is worthless across seasons (intake air density, room
   ΔT). The durable path is a closed loop on HX temperature plus a live density
   correction using intake-air temperature and baro — the same σ formula already
   used for altitude. The BME280 automatic-altitude upgrade (this item's sensor
   half) is the standing follow-up.

11. **CLOSED 2026-10-24 — API key rotated and moved to secrets.**
    The live config references `!secret` for the API encryption key, the
    WiFi credentials and the use_address; values live in an untracked,
    gitignored `secrets.yaml`. The old key that was public in git history has
    been **rotated in HA**, so the historical exposure no longer grants access.
    (The device is LAN-only regardless.) Nothing further to close.

12. **CHANGED 2026-10-20 — Auto-mode band: shutdown at the setpoint, re-engage at target-1.0.**
    Field symptom "heater stays on": `auto_shutdown_overshoot: 0.5` required the
    room to exceed target+0.5 at min pump before shutting off; when the room's
    equilibrium at min power sat below that (observed 18.3-18.4C vs the 18.5C
    threshold on 2026-10-20), the heater idled at min level indefinitely. Now
    `overshoot: 0.0` (shut off as soon as the room crosses the setpoint at min
    pump) and `hysteresis: 1.0` (re-engage at target-1.0, user-chosen).
    Field-verified 2026-10-21: standby->start fired at a 18.4C reading against
    the 18.5C threshold (target 19.5 - 1.0) — that same firing also proved the
    internal target was the float 19.5 despite the display showing 19.0 (see
    item 13). Shutdown half of the cycle at 19.5 was still in progress.
    (Separately: the two "stays on" runs on the evening of 2026-10-20 were in
    HEAT mode, which by design never auto-shuts off on the v2 profile — v2 HEAT
    is user-owned level. The AUTO-mode margin was the second, real bug.)

13. **FIXED 2026-10-20 — half-degree setpoints truncated in the display.**
    `desired_temperature_value_` was an `int`, so a 19.5 request displayed as
    19.0 (number entity + desired_temperature sensor) while the thermostat
    already ran on the exact float `desired_temp_setting_`. Member is now
    `float`; the getter follows. Wire unchanged: Vevor TX byte 4 stays integer
    °C (rounded), per protocol convention.

14. **DEPLOYED 2026-10-21 — room-sensor fault cutoff.**
    Requirement (user): DHT22 in warning state > 10 min -> heater off. A dead
    sensor leaves the thermostat on a stale last value (the NaN guard holds the
    last good reading), and the 26C ambient limit reads that same stale value —
    i.e. unmonitored burn. Now: no *accepted* room reading for 10 min -> a
    running heater stops immediately (all modes), all auto-start paths are
    blocked, and Auto mode parks in standby so it resumes normally on recovery.
    Also blocks blind starts from a stale cold value. Deployed 2026-10-21
    (rebuild + reflash), live-verified via HA entity freshness (room temp
    last_updated 02:36:01 UTC at a 02:37:02 UTC check, all heater entities
    available). Scope note (superseded by item 15): staleness originally counted
    non-NaN readings only; the 2026-10-21 glitch guard extended it so rejected
    samples also fail to refresh the timer.

15. **FIXED 2026-10-21 — DHT22 glitch samples drove the thermostat (physics guard).**
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
    Consequence for item 14: a DHT22 that is dead (NaN) *or* stuck emitting
    garbage now both trip the 10-minute fault cutoff. Compile-verified
    (COMPILE_EXIT=0) and deployed 2026-10-21; live-verified via HA entity
    freshness (same check as item 14).

16. **DEPLOYED 2026-10-24 — boot baseline: one first sample could
    become the guard reference (3-sample consensus).**
    Gap in the item 15 design: the first valid sample after boot was accepted
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

17. **IMPLEMENTED 2026-10-24 — multi-heater architecture refactor complete
    (refactor Phases 0-5, all device-verified; DESIGN.md).** Landed on `main` (as the
    `arch_analysis` branch, renamed in 2026-10-24; the pre-refactor layout is
    preserved on the `vevor_2kw` branch and the old `main` tip is tagged
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
    verified unchanged at every phase's flash. Old layout recoverable at the
    `vevor_2kw` branch base and tag `lcd-injection-final` (LCD path).

---

## Reference

- The fan-probe entity type (`type: fan_probe`) stays in the component but is no
  longer declared in the 2kW YAML — the experiment closed negative (the ECM ignores
  undocumented TX bytes 10-11). Redeclare it in one line to re-test a future unit.
- Revision drift warning: this XMZ-D2 differs from the reference unit in bytes
  8-9, 13, 14 and 52-53. Never port byte meanings between units without checking.

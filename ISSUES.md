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
   The BDAP frame carries a genuine fan command (`build_tx_frame()` frame[7..10],
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

---

## 2kW / vevor_2kw + vevor_2kw_v2 (XMZ-D2) — OPEN

5. **OPEN — the parser's byte-13 mapping is wrong on this hardware.**
   `parse_vevor_rx_frame()` publishes byte 13 as glow-plug current (gated ≤ 40 A).
   Full-cycle capture (649 frames, `scratch/vevor-cycle-full.log`) disproves it:
   byte 13 is **0 through the entire glow-preheat phase** (glow definitively on),
   and **78-128 while running with the glow off**. Its only observed values are
   {0, 78, 88, 98, 108, 118, 128} — a 10-step ladder (0 to −50 in offset-128, or a
   ×10 fixed-point of an integer 8..13). It also drifts 108 ↔ 128 in the off state
   with fan 0 and HX flat, i.e. it moves with no load → internal/self-measured
   quantity, not combustion. The real values are elsewhere:
   - **byte 15 = fan voltage**, integer 0-16: 0 when the fan is stopped, 8-11 at
     running speeds, ceiling tracks the supply rail (ours 11 of 13.2 V). RPM lags it.
   - **bytes 24-27 = glow cluster**, non-zero ONLY while the glow is lit, ramping
     then zeroing (ignition glow ~103 s → peak 101/93/83/12; post-run glow ~40 s →
     peak 79/89/78/10). **b27 is the current candidate** (range 4-12, matching the
     reference unit's glow-current byte 0-11).
   Closes when: the parser decodes byte 15 + the b24-27 cluster and stops
   publishing byte 13 as current. Open sub-question: what to *name* the cluster
   members while b24/b25/b26 (voltage vs temperature) remain unseparated without an
   amp clamp.

6. **OPEN — two entities never decode on the Vevor path.**
   `sensor.lab_heater_fan_voltage` and `sensor.lab_heater_glow_plug_voltage` read
   0.0 permanently because `parse_vevor_rx_frame()` never assigns them. Fan voltage
   is now identified (item 5); glow voltage is not yet. Closes with item 5.

7. **OPEN — `hx_temp_low = 145` sits above the level-1 equilibrium.**
   Measured: at level 1 this room equilibrates the HX around **154 °C**, i.e. the
   soot-override trigger zone is barely reachable at minimum power (it mattered for
   cold transients and for reductions while HX was low). Candidate: lower to ~110-120.
   Note the override is *disabled by design* in v2 HEAT (user owns the level), so
   this only affects v1 HEAT / AUTO. Closes when: a value is chosen from more
   low-power data.

8. **OPEN — long-hold and transition behaviour untested.**
   No soak beyond ~10 minutes. Unknown: does the ECM hold a level for hours, does it
   drift, and are level changes sent *during* a transition queued or dropped? These
   are firmware-determined and season-independent, so they are worth testing once.
   Closes with: a multi-hour hold at a fixed level, and a level change timed inside
   ignition/cooldown.

9. **OPEN — level → output is not calibrated, deliberately.**
   A frozen level→kW curve is worthless across seasons (intake air density, room
   ΔT). The durable path is a closed loop on HX temperature plus a live density
   correction using intake-air temperature and baro — the same σ formula already
   used for altitude. Related: the standing `TODO` line (BME280 for automatic
   altitude) is the sensor half of this.

10. **OPEN — hardcoded API key in `lab-heater-butler.yaml`, public repo.**
    The ESPHome API encryption key is inline; the repo is public. Pre-existing
    (already published on `origin/full_control_w_autoshutoff` before 2026-10-12),
    so this push did not newly expose it. Device is LAN-only, so practical risk is
    low. Closes when: the key moves to `secrets.yaml` and/or is rotated on both
    device and HA.

---

## Reference

- The fan-probe entity type (`type: fan_probe`) stays in the component but is no
  longer declared in the 2kW YAML — the experiment closed negative (the ECM ignores
  undocumented TX bytes 10-11). Redeclare it in one line to re-test a future unit.
- Revision drift warning: this XMZ-D2 differs from the reference unit in bytes
  8-9, 13, 14 and 52-53. Never port byte meanings between units without checking.

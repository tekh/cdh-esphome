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

5. **FIXED 2026-10-12 — the parser no longer decodes byte 13.** It now decodes the
   slots the capture proved: **byte 15 → `fan_voltage`**, **byte 27 → glow current**
   (candidate), and **bytes 24-26 → `glow_a`/`glow_b`/`glow_c`** published raw and
   unit-less as "unverified". Verified live on the bench: during ignition glow
   a/b/c read 99/92/83 with current 11.0 A, all four returning to 0 when the glow
   extinguished; while running, fan voltage read 11.0 V at 3692 RPM.
   Evidence: 649-frame full cycle (`scratch/vevor-cycle-full.log`), incl. the
   disproof of byte 13 (0 through the whole glow-preheat phase; 78-128 while
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

7. **OPEN — level-commanded profiles boot to a mid-range level, not minimum.**
   `pump_freq_user_request_` initialises to 1.6 Hz (a v1-era default), which on the
   level facade is **level 6** — so the Power Level dial reads 6 after every reboot,
   and a HEAT start before anyone touches the dial burns at level 6. For a
   level-commanded profile a boot default of level 1 (minimum burn) is the honest
   default; it is a one-line change (initialise the user request to
   `pump_freq_min` when `has_power_level_control`). Closes when: the default is
   changed and verified, or the current default is declared intentional.

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
   used for altitude. Related: the standing `TODO` line (BME280 for automatic
   altitude) is the sensor half of this.

11. **OPEN — hardcoded API key in `lab-heater-butler.yaml`, public repo.**
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

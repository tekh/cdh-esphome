# Ignition fuel gate — bench session 2026-11-09 (complete)

Everything from the 2026-11-09 bench: the defect as the owner stated it, the code,
every run with timestamps, every hypothesis tested and how it died, the final
configuration, and what the 24 h soak still has to answer.

Device: `lab-heater-butler` (Vevor 2 kW / XMZ-D2, `vevor_2kw_v2`) on hoas.
Owner flashes; the agent commands and reads telemetry.
Related: `ISSUES.md` #19, `@notes/vevor-protocol.md` (protocol facts).

---

## 1. The defect (owner's words, and the only framing that matters)

> "the problem was that it was overly fueling while the glow plug wasnt even on at
> startup"

Fuel was being injected **before the glow plug cycle**. Field symptoms: the pump
audibly ramped, then stopped, then the glow ran; white smoke; hard or silent start
failures. Not an HX-temperature problem, not a cold-vs-warm problem — the driver is
the **requested power level**.

### Mechanism

- Vevor TX carries only a power level (byte 8) and a requested state (byte 9):
  `AA 66 02 0B 00 00 00 00 | LL SS 00 00 00 00 00 CS`
- `Thermostat::enter_mode()` (v2 HEAT) seats `pump_freq_max` (2.5 Hz = level 10) on
  mode entry, and `VevorDriver::take_command()` sends it in the START frame **and in
  every 1 Hz poll** while the ECM is pre-flame.
- The ECM scales its own pre-flame dosing from that request — it self-primes, there is
  no protocol prime command, so the requested level is the only host lever.

---

## 2. The fix

`apply_ignition_fuel_gate()` in `heater_policies.h`, called from `HeaterUart::loop()`
at the driver boundary:

```cpp
if (!profile.gate_fuel_before_ignition || !start_requested) return requested_hz;
if (run_state == RUN_STATE_RUNNING) return requested_hz;
if (run_state == RUN_STATE_IGNITED) {          // glow lit, pre-flame
  if (!glow_lit) return requested_hz;          // glow out = flame established
  return min(requested_hz, profile.pump_freq_glow);
}
return min(requested_hz, profile.pump_freq_pre_ignition);   // glow off: the defect window
```

- `start_requested = driver_->on_requested() || telemetry.on` — parked heaters' idle
  polls are untouched.
- `glow_lit` = glow current or any glow-cluster byte non-zero.
- Release: glow-out (cluster -> 0) or `RUNNING`, **not** the state label — the ECM
  enters state 4 ("Ignited – Heating to Full Temp") ~10 s into the start, ~2 s before
  the glow current appears, and holds it through the whole glow cycle. Releasing on
  the 2->4 transition (the first gate version) left the glow cycle at full power.
- Profile-gated: `jeabong_8kw` has `gate_fuel_before_ignition = false`; BDAP frames
  stay byte-identical (host test `ignition_gate_off_profile_passthrough`).
- Keys: `gate_fuel_before_ignition`, `pump_freq_pre_ignition`, `pump_freq_glow`
  (all YAML-overridable; out-of-range values revert to the preset).

### Final working configuration

| key | value | effect |
|---|---|---|
| `gate_fuel_before_ignition` | true | Vevor profiles only |
| `pump_freq_pre_ignition` | 0.5 Hz (level 1) | **the fix for the original defect** — ECM doses 0.0 Hz |
| `pump_freq_glow` | **1.5 Hz (level 6)** | caps the ECM's glow-phase ramp; level 10 produced thick white smoke on a cold start, level 8 was clean, 6 is the next rung down (under test) |

Two findings, two knobs. The pre-glow floor kills fuel before the glow plug. The
glow-phase cap limits how much the ECM ramps while it waits for a cold chamber to
light — at level 10 the ECM walked the pump 1.5 -> 2.6 Hz over ~2 min and smoked
badly; at level 8 it dosed 1.5 Hz and lit cleanly. Level 3 or below starves it
entirely (pump 0.0 Hz, abort ~50 s).

---

## 3. Every run, with timestamps

### b5 — request level 3 (1.0 Hz) — FAIL, and the reason stage 2 exists

`/tmp/vevor-coldstart-b5.log`

```
13:37:12 startup offered, START 01
13:37:14 pre-glow, level 01
13:37:24 glow, level 03, pump 0.0 Hz the entire glow cycle
13:38:03 ECM aborts to State 0 (err=0, silent), no flame, HX flat 41
```

The ECM doses **nothing** from a level-3 request. Flooring the glow phase starves it.

### b6 — request level 8 (2.0 Hz) — PASS (warm plug, after failed attempts)

`/tmp/vevor-coldstart-b6.log`

```
13:42:30.6 START, level 01
13:42:32-42 polls level 01, glow OFF, pump 0.0 Hz      <- pre-glow clean
13:42:43.9 glow ON 3.0 A
13:42:42.7 polls level 08
13:43:51.5 pump dosing 1.5 Hz (68 s into glow)
13:44:23.6 released to level 0A, glow out 13:44:23.9
13:44:24   power=10, fan 2232 -> 2658, HX 47 rising
13:44:58   voluntary Cooldown after ~35 s burn (err=0), glow relit 7.0 A during purge
13:47      standby, HX 57
```

### b8 — request level 8 — PASS, full run to 150 °C

`/tmp/vevor-coldstart-b8.log`

```
14:13:01.9 START level 01
14:13:03-14 polls level 01, glow OFF, pump 0.0 Hz
14:13:13.7 glow ON 3.0 A, wire -> 08 at 14:13:14.0
14:14:22.6 pump 1.5 Hz (69 s into glow)
14:14:56.6 released 08 -> 0A, glow out 14:14:58.7
14:15:26   power=10, HX 68, fan 3523
14:16:26   HX 123, thermostat ramping 0A -> 09 -> 08 -> 07 -> 06
14:17:26   request fell to level 01: room 19.2 °C > target 19.0 °C (satisfied)
14:19:40   user stop -> 14:20 cooldown (glow relit 4.0 A) -> 14:23 standby
```

### b9 — the lockout episode (all four attempts refused fuel)

`/tmp/vevor-coldstart-b9.log`

| attempt | wire | glow | pump | abort |
|---|---|---|---|---|
| 1 — 14:45:57 | level 08 | 4->6 A | **0.0 Hz** | 14:47:01 (63 s) |
| 2 — 14:48:44 | level 08 | 4 A | **0.0 Hz** | 14:49:21 (37 s) |
| 3 — 14:51:03 | level 08 | 4 A, V 12.6->11.2 | **0.0 Hz** | ~14:51:29 (API dropped, ECM off) |
| 4 — 14:57:4x | **level 0A (2.5)** | glow | **0.0 Hz** | off |

### b9 — the success after a full heater + ESP32 power cycle

```
15:00:39  ECM boot confirmed: RX shows V=0.0, HX=0, fan 0
15:00:39  state 2; 15:00:49 state 4, wire level 01, pump 0.0 Hz, fan 1509
15:00:54  glow ON 5.0 A
15:01:59  pump 1.5 Hz (70 s into glow), V 11.1 -> recovers to 13.1 during burn
15:02:29  glow OFF (flame); HX 49 -> 59 -> 83 -> 119
15:03:31  state 5 Running; thermostat ramps 10 -> 9 -> 8 -> 7 -> 6
15:04      HX 120, fan 3716, V 12.4-13.3, no white smoke (owner)
```

---

## 3b. 17:38 cold start at level 10 — thick white smoke (why the glow stage is capped at 8)

Owner restarted a genuinely cold unit; ~1 min of thick white cloud before it lit.
HA history for that start:

```
17:38:41  Glow Plug Pre-heat
17:38:51  state 4, glow 3 A -> 12 A            <- cold plug (warm starts: 3-6 A)
17:40:01  pump 0.0 -> 1.5 Hz                   <- 70 s into glow, normal cadence
17:41:26  pump ramps 1.5 -> 2.6 Hz             <- ECM ramping while no flame
17:42:01  glow OFF (flame lights)              <- ~2 min of fuel in the chamber
17:43:16  Running
```

The pre-glow window was clean (pump 0.0 Hz the whole time), so the original defect
is fixed. The smoke comes from the ECM's glow-phase ramp at full request: it starts
dosing 70 s in and keeps ramping while a cold chamber refuses to light, so ~2 minutes
of fuel accumulates and burns off as white smoke once it catches.

Level 8 caps that ramp (dosed 1.5 Hz, lit cleanly twice). Level 10 is too much on a
cold start. The cap is now at level 6 (1.5 Hz request) as the next rung down — under
test; the hard floor is just above level 3, where the ECM stops dosing altogether. Note the earlier "level 8 refuses to dose" conclusion came from runs
taken while the ECM was locked out — that data was invalid, and it had pushed the
tuning to level 10 in the first place.

---

## 4. Hypotheses tested, and how each died

| hypothesis | verdict | evidence |
|---|---|---|
| Level 3 under-primes (ECM dosing floor) | **CONFIRMED** | b5: pump 0.0 Hz, abort 50 s, HX flat |
| Level 8 is a reliable working point | **WEAK / RETIRED** | lit once (b6, b8) on a warm plug, then 0.0 Hz in three consecutive cold attempts |
| Hot-HX start guard (would not dose on a 59 °C exchanger) | **DISPROVEN** | attempts at ECM HX 39-42 °C failed identically to the 59 °C one |
| Supply sag / brownout is the blocker | **DISPROVEN** | after the power cycle the rail held 13.0-13.3 V under full burn; level 0A on the wire still refused to dose while locked out |
| A failed start is latched by the ECM (failed-attempt counter) | **CONFIRMED** | abort window shrank 63 s -> 37 s -> ~26 s; level 10 request still refused; **only a full 12 V power cycle cleared it** (ESP32-only reboot did not) |
| The gate itself breaks dosing | **DISPROVEN** | TX frames byte-identical in the passing and failing runs (`AA 66 06 … 01 06`, `… 0A 08 … 1F`); the ECM ACKed power=10 and still refused fuel |

The practical consequence: **our component cannot clear an ECM lockout.** After
repeated failed ignitions the unit must be power-cycled at the 12 V level. This
applies to any debugging session — fewer attempts, then cut power.

---

## 5. What the 24 h soak should answer

1. Does a cold start (unit at ambient for hours) light on the first attempt with
   pre-glow clamped to level 1 and the glow stage at full?
2. Any white smoke or fuel smell before the glow lights? (the original defect)
3. Ignition time consistent with the benched ~70 s glow-phase dosing onset?
4. Does the ECM park itself after a burn and re-engage on its own, or does it need a
   start command (the earlier one-cycle-then-standby observation)?
5. If any start fails: count the attempts before the ECM latches, and confirm a 12 V
   power cycle recovers it.
6. Watch the cooldown glow relight (4-7 A, 30-40 s) — observed on every stop so far,
   routinely, no E-code.

If the soak is clean, the last code-side decision is whether to set the repo preset
`pump_freq_glow` to 2.5 so the YAML override becomes redundant.

---

## 6. Operational gotchas hit during this session

- **API connection limit is 5.** HA holds one; stray `esphome logs` clients fill the
  rest -> `Max connections (5), rejecting …` and the log client flaps. Expect
  `Processing unexpected disconnect from ESPHome API` mid-run; it reconnects in ~30 s
  and is not a heater event (4 of them occurred during a *successful* run).
- Kill strays from **inside** the add-on container (host `kill` fails — nested LXC
  PID namespace; `pkill` from the hoas shell matches its own command line and kills
  your shell). The working helper: pipe a python script into the container that walks
  `/proc`, reads `cmdline`, replaces `\x00` with a space, matches `esphome logs`, and
  SIGTERMs. Skipping the NUL replace silently finds nothing.
- Capture command:
  `ssh hoas 'docker exec -e PYTHONUNBUFFERED=1 -w /data app_5c53de3b_esphome /usr/local/bin/esphome logs /config/esphome/lab-heater-butler.yaml'`
  wrapped in `timeout N stdbuf -oL … > file`. Exit 124 = cap reached, normal.
- HA `select.select_option` on the mode entity is flaky during an API flap: the call
  reports success while the device never changes. Always verify on the wire (a
  `AA 66 06 …` START frame), not in HA.
- `preferences: Writing 1 items` in the log is routine (every 1-2 min) — not a cause
  of anything.
- Secondary UART traffic `00 1C 00 00 00 1C … 23 12 21 17 …` shares the bus; the
  driver correctly ignores it.
- The DHT22 drops readings ~5 % of the time (NaN); unrelated.

## 7. Files

- `components/heater_uart/heater_policies.h` — `apply_ignition_fuel_gate()`
- `components/heater_uart/heater_profile.h` — `pump_freq_pre_ignition`,
  `pump_freq_glow`, `gate_fuel_before_ignition`; Vevor presets armed, jeabong not
- `components/heater_uart/heater_uart.cpp/h` — gate call site + setters
- `components/heater_uart/__init__.py` — three YAML keys
- `tests/test_policies.cpp`, `tests/test_profiles.cpp` — 95 tests / 633 checks
- hoas `/config/esphome/lab-heater-butler.yaml` — `pump_freq_glow: 2.5`
  (backups `.bak-20261109`, `…b`, `…c`)
- Captures: `b5` (level 3 fail), `b6`/`b8` (level 8 pass), `b9` (lockout + post-power-cycle pass)

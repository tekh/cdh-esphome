# Vevor XMZ-D2 2 kW (`vevor_2kw`, `vevor_2kw_v2`)

The lab unit. Everything below is measured on this specific heater unless marked
otherwise. Protocol bytes/timing: `@notes/vevor-protocol.md`. Tuning values:
`heater_profile.h` presets (`profile_vevor_2kw()` / `profile_vevor_2kw_v2()`).

## Identity

| | |
|---|---|
| YAML models | `vevor_2kw_v2` (**in production**, `lab-heater-butler.yaml`), `vevor_2kw` (same tuning, legacy HEAT policy — the one-line revert target) |
| Protocol | `VEVOR_UART`, 4800 baud **inverted**, 16-byte TX / 56-57-byte RX, additive checksum |
| Fuel command | power **level 1-10** in TX byte 8 (1 level = 0.25 Hz of encoded request) |
| Control authority | the **ECM** owns ignition sequencing, glow, fan and combustion safety (`ecm_owned_safety: true`) — the firmware asks, the ECM decides |
| Priming | **not supported** (no documented command; the ECM self-primes on every start) |

## The ECM doses from the requested level, not from the thermostat

This is the single most important fact about this unit. Two phases precede the
flame and the ECM's pre-flame fuel follows TX byte 8 directly, so the requested
level is the only host-side lever:

| Requested level | Encoded Hz | Glow OFF (pre-glow) | Glow LIT (pre-flame) | Verdict |
|---|---|---|---|---|
| 1 | 0.5 | doses **0.0 Hz** | 0.0 Hz | correct for pre-glow; **starves the glow phase** |
| 3 | 1.0 | 0.0 Hz | **0.0 Hz, start aborts ~50 s** | too low for the glow phase |
| 8 | 2.0 | 0.0 Hz | 1.5 Hz, ramps modestly | glow-phase cap — clean light, no smoke |
| 10 | 2.5 | 0.0 Hz (gate clamps to 1) | 1.5 Hz then ramps to ~2.6 Hz | too much cold: thick white cloud |

The firmware's ignition gate (`gate_fuel_before_ignition`, `pump_freq_pre_ignition`,
`pump_freq_glow`) exists because of the two middle columns. It releases on
**glow out**, not on the state label — see the next section for why.

## Sequencing, with timings

| Event | Warm start | Cold start |
|---|---|---|
| START sent → raw state 2 ("pre-heat"/ignited) | ~2 s | ~2 s |
| state 2 duration | ~10 s | ~10 s |
| state 4, glow current appears | right after state 2 | same |
| glow current | 3-6 A | **10-12 A** |
| first pump dosing | ~68-70 s after glow on | same ~70 s |
| glow out (flame established) | ~95-120 s after glow on | **~190 s** after glow on |
| HX rise after flame | 45 → 120 °C in ~60 s | similar, slower at first |

**Trap:** raw state 2 maps to `RUN_STATE_IGNITED` but the glow is *not* lit yet —
the ECM enters it ~2 s before the glow current appears and holds it through the
whole glow cycle. Releasing the fuel gate on that label leaves the glow phase at
full power. Anything that keys off "ignited" as proof of flame is wrong on this
unit (this is why the HEAT resume budget does not reset until
`RUN_STATE_RUNNING`, i.e. raw state 3).

## What a healthy start looks like

```
state 2 (10 s, glow off, pump 0.0) -> state 4, glow on -> pump 0.0 for ~70 s
-> pump 1.5 Hz -> glow out -> release to thermostat request -> HX climbing
-> Running -> thermostat ramps down
```

No white smoke, `err=0` throughout. Pre-glow pump is **0.0 Hz in every good run**;
if the pump moves before glow current is non-zero, the gate is not armed.

## Latches — two different ones, both need the 12 V cut

1. **Failed-ignition latch.** After a few failed starts the ECM runs the glow,
   never drives the pump, and shortens its abort window each attempt
   (~63 s → 37 s → ~26 s) with `err=0` throughout. It refuses fuel even at
   level 10 on the wire. Rebooting the ESP32 does nothing; only a 12 V power
   cycle clears it. **Consequence: never read a no-dose result while latched** —
   it is a lockout artefact, not a tuning result.
2. **E10 fault latch** (`err=1`, "Startup Failure"). The ECM parks in **raw state
   4 (cooldown) with `on` still true** and stays there indefinitely (50+ min
   observed, fan 0 RPM, pump 0.0 Hz, HX at ambient). It then **ignores START
   frames entirely** — proven 2026-11-20 with 120 s of one-per-second
   `AA 66 06 … 06` plus an automatic retry: state never left 4. Firmware keeps
   the start intent asserted so a power-cycled ECM is met by a START on the first
   frame it answers; it parks loudly instead of pretending to work
   (`ISSUES V-22`).

Operator procedure either way: **cut the 12 V for ~30 s, restore, re-select Heat
if it does not relight by itself.**

## Behaviour the ECM owns (not faults)

- **Cooldown is real and long.** After a burn it runs a cooldown with the fan,
  reporting raw state 4 and `on=true`. A normal cooldown ends by itself in a few
  minutes; the E10 latch is the one that never ends.
- **The glow relights during cooldown** (4-7 A, 30-40 s). That is residue
  burn-off, not a re-ignition attempt, and it needs no fuel. Seen on every stop.
- **Dead-man:** if controller frames stop, the ECM fails safe into cooldown 1-33 s
  later. An ESP32 reset or Wi-Fi drop therefore looks exactly like a heater
  shutdown — and since the mode is not persisted, the heater stays off after.
- **Silent when fully off.** Once the heater is really off it stops answering
  status polls, so `on_fresh_frame()`/`control()` starve. Restart decisions live
  on the facade's 1 Hz timer for that reason.
- **Altitude derate is the host's job** (the frame has no altitude field): fuel is
  scaled by ISA σ(h) at the TX boundary, applied only at stable combustion so
  ignition keeps its full rate.

## Sensor quirks on this bench unit

- The DHT22 is marginal: ~4-5 % of reads fail, publishing `NaN` for one 10 s
  cycle, roughly every 2-4 minutes. Log lines: `[W][dht:158] Communication
  failed`, `[W][dht:053] Invalid readings`. The physics guard (`RoomSensorGuard`)
  exists because of this — a glitch sample once tripped a false standby→start.
- `byte 13` is **not** glow current on this revision (reads 0 through the glow
  phase, 78-128 while running with the glow off). Bytes 24-26 ramp with the glow
  but voltage vs temperature is not separated — published raw (`ISSUES V-5`, `V-6`).

## Closed negative results

- **Fan-speed probe (TX bytes 10-11): dead.** The documented protocol has no fan
  command and this ECM ignores the bytes; keep the probe entity undeclared. One
  line redeclares it if a future unit is worth testing. Also: bytes 10-11 must be
  zeroed explicitly when the probe is off, or stack garbage goes on the bus.

## Open items on this unit

`ISSUES V-8` (`hx_temp_low = 145` sits above the level-1 equilibrium),
`V-9` (long-hold and transition behaviour untested), `V-10` (level → output not
calibrated, deliberately). Read the issue text before changing any preset value.

# Heater notes — per-model bench behaviour

One page per heater model. This is the **know-how** layer: what the unit actually
does on the bench, what its controller latches into, what "normal" looks like in
a capture. It is deliberately *not* the issue ledger — an issue is something to
fix or decide (`ISSUES.md`, ids `CF-` / `V-` / `BD-`), a note is something that
stays true after the issue closes.

| Model (YAML `heater_model:`) | Page | State |
|---|---|---|
| `vevor_2kw_v2` (production), `vevor_2kw` (legacy HEAT policy) | [vevor_xmz-d2_2kw.md](vevor_xmz-d2_2kw.md) | bench-validated, in production |
| `jeabong_8kw` | [bdap_jeabong_8kw.md](bdap_jeabong_8kw.md) | **PARKED** — no device target, compile-only |

## What belongs on a model page

- How the unit sequences a start and a stop, with timings measured on the bench
- What its controller does on its own versus what the firmware commands
- Latch/lockout behaviour, and what clears it
- What a healthy capture looks like, so an operator can tell normal from fault
- Numbers that are specific to that unit (dose per level, glow current, stall points)

## What does not

- Bit/byte layouts → `@notes/vevor-protocol.md` (link it, don't duplicate)
- Tuning values → `heater_profile.h` preset (the code is the source of truth)
- Open defects and decisions → `ISSUES.md`
- How to flash, capture, or deploy → the `cdh-esphome-heater` agent skill

## Adding a model

Copy the closest page, keep the section order (identity → sequencing → own
behaviour → latches → healthy trace → quirks → open items), and fill it only with
values you measured. A page with guessed numbers is worse than no page: it gets
trusted.

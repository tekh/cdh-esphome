# Home Assistant integration

UI pieces for the lab heater ESPHome device (`lab-heater-butler.yaml`, node
name `lab-heater`). `screenshot.png` is the dashboard as it currently looks.

## Install

**1. The scripts** (`homeassistant-scripts.yaml`) — first, the dashboard
buttons call them.
HA → *Settings → Devices & Services → Scripts → ⋮ → Edit scripts* → paste →
save. Gives you `script.heater_temp_up` / `script.heater_temp_down` (±0.5 °C
on the target, clamped 8–30).

**2. The dashboard card** (`homeassistant-dashboard.yaml`).
*Dashboard → ⋮ → Edit dashboard → add a card → Edit as YAML* → paste → save.

## If the ESPHome node is renamed

Entity ids are derived from the node name (`lab-heater` → prefix `lab_heater_`).
If you rename `esphome: name:` in `lab-heater-butler.yaml`, replace every
`lab_heater_` in both files.

## What the dashboard expects

Everything it shows is declared in `lab-heater-butler.yaml`: the mode select,
target-temperature and power-level dials, the DHT22 room sensors, all
`heater_uart` sensors (incl. `power_level_actual`), the four binary sensors,
the run-state / error-code text sensors, and the fuel-prime button. The red
error panel only appears when `text_sensor.lab_heater_error_code` ≠ "No
Error".

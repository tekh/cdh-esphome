import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import DEVICE_CLASS_POWER, DEVICE_CLASS_PROBLEM

from . import heater_uart_ns, HeaterUart

DEPENDENCIES = ["heater_uart"]

BINARY_SENSORS = {
    "on_off_state": ("On/Off State", DEVICE_CLASS_POWER),
    "auto_shutdown_active": ("Auto Shutdown Active", DEVICE_CLASS_POWER),
    "standby_active": ("Standby Active", DEVICE_CLASS_POWER),
    "priming_active": ("Priming Active", DEVICE_CLASS_POWER),
    # Room-probe health: ON when the DHT22 has produced no accepted reading for
    # >10 min (the guard is blind). That is the exact condition that suppresses
    # AUTO restarts, so it explains an off heater that will not come back.
    "temperature_probe_failed": ("Temperature Probe Failed", DEVICE_CLASS_PROBLEM),
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(HeaterUart),
        **{
            cv.Optional(binary_sensor_key): binary_sensor.binary_sensor_schema(
                device_class=device_class
            )
            for binary_sensor_key, (name, device_class) in BINARY_SENSORS.items()
        },
    }
)

async def to_code(config):
    parent = await cg.get_variable(config[cv.GenerateID()])
    for binary_sensor_key, (name, _) in BINARY_SENSORS.items():
        if binary_sensor_key in config:
            bin_sens = await binary_sensor.new_binary_sensor(config[binary_sensor_key])
            cg.add(parent.set_binary_sensor(binary_sensor_key, bin_sens))

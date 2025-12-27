import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor
from esphome.const import CONF_ID

heater_uart_ns = cg.esphome_ns.namespace("heater_uart")
HeaterUart = heater_uart_ns.class_("HeaterUart", cg.Component, uart.UARTDevice)

DEPENDENCIES = ["uart"]

# Configuration keys
CONF_STANDALONE_MODE = "standalone_mode"
CONF_OPERATING_VOLTAGE = "operating_voltage"
CONF_TEMPERATURE_SENSOR = "temperature_sensor"
CONF_TEMPERATURE_BACKOFF = "temperature_backoff"
CONF_ALTITUDE = "altitude"
CONF_AUTO_SHUTDOWN_OVERSHOOT = "auto_shutdown_overshoot"
CONF_AUTO_SHUTDOWN_HYSTERESIS = "auto_shutdown_hysteresis"

# Operating voltage constants (must match C++ values)
VOLTAGE_12V = 0x78  # 120 = 12.0V
VOLTAGE_24V = 0xF0  # 240 = 24.0V

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(HeaterUart),
        cv.Optional("update_interval", default="5s"): cv.update_interval,
        cv.Optional(CONF_STANDALONE_MODE, default=False): cv.boolean,
        cv.Optional(CONF_OPERATING_VOLTAGE, default="12V"): cv.enum(
            {"12V": VOLTAGE_12V, "24V": VOLTAGE_24V}, upper=True
        ),
        cv.Optional(CONF_TEMPERATURE_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_TEMPERATURE_BACKOFF, default=0.5): cv.float_range(min=0.0, max=5.0),
        cv.Optional(CONF_ALTITUDE, default=750): cv.int_range(min=0, max=5000),
        cv.Optional(CONF_AUTO_SHUTDOWN_OVERSHOOT, default=0.5): cv.float_range(min=0.0, max=3.0),
        cv.Optional(CONF_AUTO_SHUTDOWN_HYSTERESIS, default=1.5): cv.float_range(min=0.5, max=5.0),
    }
).extend(uart.UART_DEVICE_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    if "update_interval" in config:
        cg.add(var.set_update_interval(config["update_interval"]))

    # Configure standalone mode
    cg.add(var.set_standalone_mode(config[CONF_STANDALONE_MODE]))
    cg.add(var.set_operating_voltage(config[CONF_OPERATING_VOLTAGE]))
    cg.add(var.set_temperature_backoff(config[CONF_TEMPERATURE_BACKOFF]))
    cg.add(var.set_altitude(config[CONF_ALTITUDE]))

    # Configure auto-shutdown parameters
    cg.add(var.set_auto_shutdown_overshoot(config[CONF_AUTO_SHUTDOWN_OVERSHOOT]))
    cg.add(var.set_auto_shutdown_hysteresis(config[CONF_AUTO_SHUTDOWN_HYSTERESIS]))

    # Configure external temperature sensor (for standalone mode)
    if CONF_TEMPERATURE_SENSOR in config:
        temp_sensor = await cg.get_variable(config[CONF_TEMPERATURE_SENSOR])
        cg.add(var.set_temperature_sensor(temp_sensor))

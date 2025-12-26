import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    CONF_ID,
    CONF_MIN_VALUE,
    CONF_MAX_VALUE,
    CONF_STEP,
    UNIT_CELSIUS,
    ICON_THERMOMETER,
)

from . import heater_uart_ns, HeaterUart

DEPENDENCIES = ["heater_uart"]

# Define the HeaterTemperatureNumber class
HeaterTemperatureNumber = heater_uart_ns.class_(
    "HeaterTemperatureNumber", number.Number, cg.Component
)

CONF_HEATER_UART_ID = "heater_uart_id"

CONFIG_SCHEMA = (
    number.number_schema(HeaterTemperatureNumber)
    .extend(
        {
            cv.GenerateID(CONF_HEATER_UART_ID): cv.use_id(HeaterUart),
            cv.Optional(CONF_MIN_VALUE, default=8): cv.float_,
            cv.Optional(CONF_MAX_VALUE, default=35): cv.float_,
            cv.Optional(CONF_STEP, default=1): cv.float_,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await number.new_number(
        config,
        min_value=config[CONF_MIN_VALUE],
        max_value=config[CONF_MAX_VALUE],
        step=config[CONF_STEP],
    )
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_HEATER_UART_ID])
    cg.add(var.set_parent(parent))

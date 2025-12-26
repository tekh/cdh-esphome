import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    CONF_ID,
    CONF_MIN_VALUE,
    CONF_MAX_VALUE,
    CONF_STEP,
    CONF_TYPE,
    UNIT_CELSIUS,
    UNIT_HERTZ,
    ICON_THERMOMETER,
)

from . import heater_uart_ns, HeaterUart

DEPENDENCIES = ["heater_uart"]

# Define the number classes
HeaterTemperatureNumber = heater_uart_ns.class_(
    "HeaterTemperatureNumber", number.Number, cg.Component
)

HeaterPumpNumber = heater_uart_ns.class_(
    "HeaterPumpNumber", number.Number, cg.Component
)

CONF_HEATER_UART_ID = "heater_uart_id"

# Number types
TYPE_TEMPERATURE = "temperature"
TYPE_PUMP = "pump"

TEMPERATURE_SCHEMA = (
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

PUMP_SCHEMA = (
    number.number_schema(HeaterPumpNumber)
    .extend(
        {
            cv.GenerateID(CONF_HEATER_UART_ID): cv.use_id(HeaterUart),
            cv.Optional(CONF_MIN_VALUE, default=1.0): cv.float_,
            cv.Optional(CONF_MAX_VALUE, default=6.0): cv.float_,
            cv.Optional(CONF_STEP, default=0.1): cv.float_,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

CONFIG_SCHEMA = cv.typed_schema(
    {
        TYPE_TEMPERATURE: TEMPERATURE_SCHEMA,
        TYPE_PUMP: PUMP_SCHEMA,
    },
    default_type=TYPE_TEMPERATURE,
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

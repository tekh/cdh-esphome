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

HeaterFanProbeNumber = heater_uart_ns.class_(
    "HeaterFanProbeNumber", number.Number, cg.Component
)

HeaterPowerLevelNumber = heater_uart_ns.class_(
    "HeaterPowerLevelNumber", number.Number, cg.Component
)

CONF_HEATER_UART_ID = "heater_uart_id"

# Number types
TYPE_TEMPERATURE = "temperature"
TYPE_PUMP = "pump"
TYPE_POWER_LEVEL = "power_level"
TYPE_FAN_PROBE = "fan_probe"

TEMPERATURE_SCHEMA = (
    number.number_schema(HeaterTemperatureNumber)
    .extend(
        {
            cv.GenerateID(CONF_HEATER_UART_ID): cv.use_id(HeaterUart),
            cv.Optional(CONF_MIN_VALUE, default=8): cv.float_,
            cv.Optional(CONF_MAX_VALUE, default=35): cv.float_,
            cv.Optional(CONF_STEP, default=0.5): cv.float_,
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

# Power-level dial (1-10). The fuel control on level-commanded hardware
# (Vevor 2kW). Writes are refused at runtime unless the active profile's
# has_power_level_control flag is set, so it can never coexist with the pump
# frequency dial on the same heater.
POWER_LEVEL_SCHEMA = (
    number.number_schema(HeaterPowerLevelNumber)
    .extend(
        {
            cv.GenerateID(CONF_HEATER_UART_ID): cv.use_id(HeaterUart),
            cv.Optional(CONF_MIN_VALUE, default=1): cv.float_,
            cv.Optional(CONF_MAX_VALUE, default=10): cv.float_,
            cv.Optional(CONF_STEP, default=1): cv.float_,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

# Experimental fan-speed probe (Vevor only). 0 = disabled (undocumented TX
# bytes 10-11 stay 0x00); >0 = BE uint16 RPM written into TX bytes 10-11.
FAN_PROBE_SCHEMA = (
    number.number_schema(HeaterFanProbeNumber)
    .extend(
        {
            cv.GenerateID(CONF_HEATER_UART_ID): cv.use_id(HeaterUart),
            cv.Optional(CONF_MIN_VALUE, default=0): cv.float_,
            cv.Optional(CONF_MAX_VALUE, default=4000): cv.float_,
            cv.Optional(CONF_STEP, default=50): cv.float_,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

CONFIG_SCHEMA = cv.typed_schema(
    {
        TYPE_TEMPERATURE: TEMPERATURE_SCHEMA,
        TYPE_PUMP: PUMP_SCHEMA,
        TYPE_POWER_LEVEL: POWER_LEVEL_SCHEMA,
        TYPE_FAN_PROBE: FAN_PROBE_SCHEMA,
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

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID, CONF_TYPE

from . import heater_uart_ns, HeaterUart

DEPENDENCIES = ["heater_uart"]

# Define the switch classes
HeaterSwitch = heater_uart_ns.class_("HeaterSwitch", switch.Switch, cg.Component)
HeaterAutoShutdownSwitch = heater_uart_ns.class_(
    "HeaterAutoShutdownSwitch", switch.Switch, cg.Component
)

CONF_HEATER_UART_ID = "heater_uart_id"

# Switch types
TYPE_POWER = "power"
TYPE_AUTO_SHUTDOWN = "auto_shutdown"

POWER_SCHEMA = (
    switch.switch_schema(HeaterSwitch)
    .extend(
        {
            cv.GenerateID(CONF_HEATER_UART_ID): cv.use_id(HeaterUart),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

AUTO_SHUTDOWN_SCHEMA = (
    switch.switch_schema(HeaterAutoShutdownSwitch)
    .extend(
        {
            cv.GenerateID(CONF_HEATER_UART_ID): cv.use_id(HeaterUart),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

CONFIG_SCHEMA = cv.typed_schema(
    {
        TYPE_POWER: POWER_SCHEMA,
        TYPE_AUTO_SHUTDOWN: AUTO_SHUTDOWN_SCHEMA,
    },
    default_type=TYPE_POWER,
)


async def to_code(config):
    var = await switch.new_switch(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_HEATER_UART_ID])
    cg.add(var.set_parent(parent))

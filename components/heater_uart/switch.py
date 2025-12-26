import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID

from . import heater_uart_ns, HeaterUart

DEPENDENCIES = ["heater_uart"]

# Define the HeaterSwitch class
HeaterSwitch = heater_uart_ns.class_("HeaterSwitch", switch.Switch, cg.Component)

CONF_HEATER_UART_ID = "heater_uart_id"

CONFIG_SCHEMA = switch.switch_schema(HeaterSwitch).extend(
    {
        cv.GenerateID(CONF_HEATER_UART_ID): cv.use_id(HeaterUart),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = await switch.new_switch(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_HEATER_UART_ID])
    cg.add(var.set_parent(parent))

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import CONF_ID

from . import heater_uart_ns, HeaterUart

DEPENDENCIES = ["heater_uart"]

# Define the select class
HeaterModeSelect = heater_uart_ns.class_(
    "HeaterModeSelect", select.Select, cg.Component
)

CONF_HEATER_UART_ID = "heater_uart_id"

CONFIG_SCHEMA = (
    select.select_schema(HeaterModeSelect)
    .extend(
        {
            cv.GenerateID(CONF_HEATER_UART_ID): cv.use_id(HeaterUart),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await select.new_select(config, options=["Off", "Auto", "On"])
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_HEATER_UART_ID])
    cg.add(var.set_parent(parent))

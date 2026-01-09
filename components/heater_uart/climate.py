import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate
from esphome.const import CONF_ID

from . import heater_uart_ns, HeaterUart

DEPENDENCIES = ["heater_uart"]

# Define the climate class
HeaterClimate = heater_uart_ns.class_(
    "HeaterClimate", climate.Climate, cg.Component
)

CONF_HEATER_UART_ID = "heater_uart_id"

CONFIG_SCHEMA = climate.climate_schema(HeaterClimate).extend(
    {
        cv.GenerateID(CONF_HEATER_UART_ID): cv.use_id(HeaterUart),
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)

    parent = await cg.get_variable(config[CONF_HEATER_UART_ID])
    cg.add(var.set_parent(parent))

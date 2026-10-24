import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor
from esphome.const import CONF_ID

heater_uart_ns = cg.esphome_ns.namespace("heater_uart")
HeaterUart = heater_uart_ns.class_("HeaterUart", cg.Component, uart.UARTDevice)

DEPENDENCIES = ["uart"]

# Configuration keys
CONF_HEATER_MODEL = "heater_model"
CONF_OPERATING_VOLTAGE = "operating_voltage"
CONF_TEMPERATURE_SENSOR = "temperature_sensor"
CONF_TEMPERATURE_BACKOFF = "temperature_backoff"
CONF_ALTITUDE = "altitude"
CONF_AUTO_SHUTDOWN_OVERSHOOT = "auto_shutdown_overshoot"
CONF_AUTO_SHUTDOWN_HYSTERESIS = "auto_shutdown_hysteresis"
CONF_APPROACH_THRESHOLD = "approach_threshold"
CONF_AMBIENT_HEAT_LIMIT = "ambient_heat_limit"
CONF_FIRMWARE_SAFETY_OVERSIGHT = "firmware_safety_oversight"

# --- Tuning overrides (ROADMAP Phase 4): every key optional; overrides sit on
# top of the selected model's compiled preset. Invalid combos are reverted to
# the preset at setup() with a warning (permissive, single-operator audience).
CONF_PUMP_FREQ_MIN = "pump_freq_min"
CONF_PUMP_FREQ_MAX = "pump_freq_max"
CONF_PUMP_FREQ_STEP = "pump_freq_step"
CONF_PUMP_FREQ_INITIAL = "pump_freq_initial"
CONF_PUMP_PRIME_FREQ = "pump_prime_freq"
CONF_FAN_RPM_MIN = "fan_rpm_min"
CONF_FAN_RPM_MAX = "fan_rpm_max"
CONF_IGNITION_FAN_RPM = "ignition_fan_rpm"
CONF_COOLDOWN_FAN_RPM = "cooldown_fan_rpm"
CONF_IGNITION_HX_THRESHOLD = "ignition_hx_threshold"
CONF_HX_TEMP_LOW = "hx_temp_low"
CONF_HX_TEMP_TARGET = "hx_temp_target"
CONF_HX_TEMP_HIGH = "hx_temp_high"
CONF_HX_TEMP_CRITICAL = "hx_temp_critical"
CONF_COOLDOWN_TARGET_TEMP = "cooldown_target_temp"
CONF_FAN_SENSOR = "fan_sensor"
CONF_GLOW_POWER = "glow_power"
CONF_TEMP_MIN = "temp_min"
CONF_TEMP_MAX = "temp_max"
CONF_PUMP_ADJUST_INTERVAL = "pump_adjust_interval"
CONF_PRIMING_DURATION = "priming_duration"
CONF_THERMO_GAP_FAR = "thermostat_gap_far"
CONF_THERMO_GAP_MID = "thermostat_gap_mid"
CONF_THERMO_STEP_MULT_FAR = "thermostat_step_mult_far"
CONF_THERMO_STEP_MULT_MID = "thermostat_step_mult_mid"
CONF_THERMO_OS_BIG = "thermostat_os_big"
CONF_THERMO_OS_MID = "thermostat_os_mid"
CONF_THERMO_OS_MULT_BIG = "thermostat_os_mult_big"
CONF_THERMO_OS_MULT_MID = "thermostat_os_mult_mid"

# Operating voltage constants (must match C++ values)
VOLTAGE_12V = 0x78  # 120 = 12.0V
VOLTAGE_24V = 0xF0  # 240 = 24.0V

# Heater model enum (must match HeaterModel in heater_profile.h).
# Scoped enum (`enum class`) -> is_class=True renders `heater_uart::HeaterModel::X`.
# Members are referenced by attribute access; MockObj builds them lazily.
HeaterModel = heater_uart_ns.enum("HeaterModel", is_class=True)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(HeaterUart),
        cv.Optional("update_interval", default="5s"): cv.update_interval,
        # Required: which physical heater is connected. The selected profile
        # drives pump/fan/HX limits -> never leave this implicit.
        cv.Required(CONF_HEATER_MODEL): cv.enum(            {"jeabong_8kw": HeaterModel.JEABONG_8KW, "vevor_2kw": HeaterModel.VEVOR_2KW,
             "vevor_2kw_v2": HeaterModel.VEVOR_2KW_V2},
            lower=True,
        ),
        cv.Optional(CONF_OPERATING_VOLTAGE, default="12V"): cv.enum(
            {"12V": VOLTAGE_12V, "24V": VOLTAGE_24V}, upper=True
        ),
        cv.Optional(CONF_TEMPERATURE_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_TEMPERATURE_BACKOFF, default=0.5): cv.float_range(min=0.0, max=5.0),
        cv.Optional(CONF_ALTITUDE, default=750): cv.int_range(min=0, max=5000),
        cv.Optional(CONF_AUTO_SHUTDOWN_OVERSHOOT, default=0.5): cv.float_range(min=0.0, max=3.0),
        cv.Optional(CONF_AUTO_SHUTDOWN_HYSTERESIS, default=1.5): cv.float_range(min=0.5, max=5.0),
        cv.Optional(CONF_APPROACH_THRESHOLD, default=1.0): cv.float_range(min=0.5, max=3.0),
        cv.Optional(CONF_AMBIENT_HEAT_LIMIT, default=26.0): cv.float_range(min=-40.0, max=80.0),
        cv.Optional(CONF_FIRMWARE_SAFETY_OVERSIGHT, default=True): cv.boolean,
        # --- Tuning overrides (all optional; validated ranges; see ROADMAP) ---
        cv.Optional(CONF_PUMP_FREQ_MIN): cv.float_range(min=0.0, max=10.0),
        cv.Optional(CONF_PUMP_FREQ_MAX): cv.float_range(min=0.0, max=10.0),
        cv.Optional(CONF_PUMP_FREQ_STEP): cv.float_range(min=0.01, max=1.0),
        cv.Optional(CONF_PUMP_FREQ_INITIAL): cv.float_range(min=0.0, max=10.0),
        cv.Optional(CONF_PUMP_PRIME_FREQ): cv.float_range(min=0.0, max=10.0),
        cv.Optional(CONF_FAN_RPM_MIN): cv.int_range(min=0, max=65000),
        cv.Optional(CONF_FAN_RPM_MAX): cv.int_range(min=0, max=65000),
        cv.Optional(CONF_IGNITION_FAN_RPM): cv.int_range(min=0, max=65000),
        cv.Optional(CONF_COOLDOWN_FAN_RPM): cv.int_range(min=0, max=65000),
        cv.Optional(CONF_IGNITION_HX_THRESHOLD): cv.float_range(min=20.0, max=250.0),
        cv.Optional(CONF_HX_TEMP_LOW): cv.float_range(min=50.0, max=350.0),
        cv.Optional(CONF_HX_TEMP_TARGET): cv.float_range(min=50.0, max=350.0),
        cv.Optional(CONF_HX_TEMP_HIGH): cv.float_range(min=50.0, max=350.0),
        cv.Optional(CONF_HX_TEMP_CRITICAL): cv.float_range(min=50.0, max=350.0),
        cv.Optional(CONF_COOLDOWN_TARGET_TEMP): cv.float_range(min=0.0, max=150.0),
        cv.Optional(CONF_FAN_SENSOR): cv.int_range(min=0, max=255),
        cv.Optional(CONF_GLOW_POWER): cv.int_range(min=0, max=255),
        cv.Optional(CONF_TEMP_MIN): cv.int_range(min=0, max=15),
        cv.Optional(CONF_TEMP_MAX): cv.int_range(min=15, max=40),
        cv.Optional(CONF_PUMP_ADJUST_INTERVAL): cv.float_range(min=1.0, max=30.0),   # seconds
        cv.Optional(CONF_PRIMING_DURATION): cv.float_range(min=10.0, max=300.0),     # seconds
        cv.Optional(CONF_THERMO_GAP_FAR): cv.float_range(min=2.0, max=30.0),         # °C
        cv.Optional(CONF_THERMO_GAP_MID): cv.float_range(min=0.5, max=15.0),         # °C
        cv.Optional(CONF_THERMO_STEP_MULT_FAR): cv.float_range(min=1.0, max=5.0),
        cv.Optional(CONF_THERMO_STEP_MULT_MID): cv.float_range(min=1.0, max=5.0),
        cv.Optional(CONF_THERMO_OS_BIG): cv.float_range(min=0.5, max=10.0),          # °C
        cv.Optional(CONF_THERMO_OS_MID): cv.float_range(min=0.2, max=5.0),           # °C
        cv.Optional(CONF_THERMO_OS_MULT_BIG): cv.float_range(min=1.0, max=5.0),
        cv.Optional(CONF_THERMO_OS_MULT_MID): cv.float_range(min=1.0, max=5.0),
    }
).extend(uart.UART_DEVICE_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    if "update_interval" in config:
        cg.add(var.set_update_interval(config["update_interval"]))

    # Configure the heater model (standalone is the only mode since the LCD
    # injection path was removed 2026-10-24; recover via tag lcd-injection-final)
    cg.add(var.set_heater_model(config[CONF_HEATER_MODEL]))
    cg.add(var.set_operating_voltage(config[CONF_OPERATING_VOLTAGE]))
    cg.add(var.set_temperature_backoff(config[CONF_TEMPERATURE_BACKOFF]))
    cg.add(var.set_altitude(config[CONF_ALTITUDE]))

    # Configure auto-shutdown parameters
    cg.add(var.set_auto_shutdown_overshoot(config[CONF_AUTO_SHUTDOWN_OVERSHOOT]))
    cg.add(var.set_auto_shutdown_hysteresis(config[CONF_AUTO_SHUTDOWN_HYSTERESIS]))
    cg.add(var.set_approach_threshold(config[CONF_APPROACH_THRESHOLD]))

    # Configure ambient heat limit (HEAT mode safety)
    cg.add(var.set_ambient_heat_limit(config[CONF_AMBIENT_HEAT_LIMIT]))

    # Combustion oversight (gated to ECM-owned-safety profiles in C++ setup())
    cg.add(var.set_firmware_safety_oversight(config[CONF_FIRMWARE_SAFETY_OVERSIGHT]))

    # --- Tuning overrides: only keys PRESENT in the YAML are set; the rest
    # keep the selected model's compiled preset values.
    TUNING_KEYS = [
        (CONF_PUMP_FREQ_MIN, "set_pump_freq_min"),
        (CONF_PUMP_FREQ_MAX, "set_pump_freq_max"),
        (CONF_PUMP_FREQ_STEP, "set_pump_freq_step"),
        (CONF_PUMP_FREQ_INITIAL, "set_pump_freq_initial"),
        (CONF_PUMP_PRIME_FREQ, "set_pump_prime_freq"),
        (CONF_FAN_RPM_MIN, "set_fan_rpm_min"),
        (CONF_FAN_RPM_MAX, "set_fan_rpm_max"),
        (CONF_IGNITION_FAN_RPM, "set_ignition_fan_rpm"),
        (CONF_COOLDOWN_FAN_RPM, "set_cooldown_fan_rpm"),
        (CONF_IGNITION_HX_THRESHOLD, "set_ignition_hx_threshold"),
        (CONF_HX_TEMP_LOW, "set_hx_temp_low"),
        (CONF_HX_TEMP_TARGET, "set_hx_temp_target"),
        (CONF_HX_TEMP_HIGH, "set_hx_temp_high"),
        (CONF_HX_TEMP_CRITICAL, "set_hx_temp_critical"),
        (CONF_COOLDOWN_TARGET_TEMP, "set_cooldown_target_temp"),
        (CONF_FAN_SENSOR, "set_fan_sensor"),
        (CONF_GLOW_POWER, "set_glow_power"),
        (CONF_TEMP_MIN, "set_temp_min"),
        (CONF_TEMP_MAX, "set_temp_max"),
        (CONF_PUMP_ADJUST_INTERVAL, "set_pump_adjust_interval"),
        (CONF_PRIMING_DURATION, "set_priming_duration"),
        (CONF_THERMO_GAP_FAR, "set_thermostat_gap_far"),
        (CONF_THERMO_GAP_MID, "set_thermostat_gap_mid"),
        (CONF_THERMO_STEP_MULT_FAR, "set_thermostat_step_mult_far"),
        (CONF_THERMO_STEP_MULT_MID, "set_thermostat_step_mult_mid"),
        (CONF_THERMO_OS_BIG, "set_thermostat_os_big"),
        (CONF_THERMO_OS_MID, "set_thermostat_os_mid"),
        (CONF_THERMO_OS_MULT_BIG, "set_thermostat_os_mult_big"),
        (CONF_THERMO_OS_MULT_MID, "set_thermostat_os_mult_mid"),
    ]
    for key, setter in TUNING_KEYS:
        if key in config:
            cg.add(getattr(var, setter)(config[key]))

    # Configure external temperature sensor (for standalone mode)
    if CONF_TEMPERATURE_SENSOR in config:
        temp_sensor = await cg.get_variable(config[CONF_TEMPERATURE_SENSOR])
        cg.add(var.set_temperature_sensor(temp_sensor))

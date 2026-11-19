import esphome.codegen as cg
from esphome.components import binary_sensor, climate, sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_HUMIDITY_SENSOR,
    CONF_SENSOR,
    CONF_TARGET_TEMPERATURE,
)

from . import valve_actuator_ns
from .valve import ValveActuator

AUTO_LOAD = ["binary_sensor", "sensor"]

CONF_VALVE_ID = "valve_id"
CONF_DEFAULT_TARGET_TEMPERATURE = "default_target_temperature"
CONF_PRESETS = "presets"
CONF_VALVE_POSITION = "valve_position"
CONF_CONTROL = "control"
CONF_KP = "kp"
CONF_KI = "ki"
CONF_DEADBAND = "deadband"
CONF_MIN_STEP = "min_step"
CONF_MAX_POSITION = "max_position"
CONF_VALVE_HELD = "valve_held"

RadiatorClimate = valve_actuator_ns.class_(
    "RadiatorClimate", climate.Climate, cg.PollingComponent
)

PRESETS = {
    "home": climate.ClimatePreset.CLIMATE_PRESET_HOME,
    "away": climate.ClimatePreset.CLIMATE_PRESET_AWAY,
    "sleep": climate.ClimatePreset.CLIMATE_PRESET_SLEEP,
    "eco": climate.ClimatePreset.CLIMATE_PRESET_ECO,
    "comfort": climate.ClimatePreset.CLIMATE_PRESET_COMFORT,
    "boost": climate.ClimatePreset.CLIMATE_PRESET_BOOST,
    "activity": climate.ClimatePreset.CLIMATE_PRESET_ACTIVITY,
}

PRESET_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_TARGET_TEMPERATURE): cv.temperature,
        # Where the valve usually sits at this temperature: the starting point
        # when the preset is chosen.
        cv.Optional(CONF_VALVE_POSITION): cv.percentage,
    }
)

CONTROL_SCHEMA = cv.Schema(
    {
        # Valve opening per degree below the target.
        cv.Optional(CONF_KP, default="20%"): cv.percentage,
        # Valve opening added per degree-hour below the target.
        cv.Optional(CONF_KI, default="5%"): cv.percentage,
        cv.Optional(CONF_DEADBAND, default=0.2): cv.temperature_delta,
        cv.Optional(CONF_MIN_STEP, default="5%"): cv.percentage,
        cv.Optional(CONF_MAX_POSITION, default="100%"): cv.All(
            cv.percentage, cv.Range(min=0.05)
        ),
    }
)

CONFIG_SCHEMA = (
    climate.climate_schema(RadiatorClimate)
    .extend(
        {
            cv.GenerateID(CONF_VALVE_ID): cv.use_id(ValveActuator),
            cv.Required(CONF_SENSOR): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_HUMIDITY_SENSOR): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_DEFAULT_TARGET_TEMPERATURE, default=21.0): cv.temperature,
            cv.Optional(CONF_PRESETS, default={}): cv.Schema(
                {cv.Optional(name): PRESET_SCHEMA for name in PRESETS}
            ),
            cv.Optional(CONF_CONTROL, default={}): CONTROL_SCHEMA,
            cv.Optional(CONF_VALVE_HELD): binary_sensor.binary_sensor_schema(
                icon="mdi:hand-back-right",
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
)


async def to_code(config):
    var = await climate.new_climate(config)
    await cg.register_component(var, config)

    valve = await cg.get_variable(config[CONF_VALVE_ID])
    cg.add(var.set_valve(valve))
    cg.add(var.set_sensor(await cg.get_variable(config[CONF_SENSOR])))
    if CONF_HUMIDITY_SENSOR in config:
        cg.add(
            var.set_humidity_sensor(await cg.get_variable(config[CONF_HUMIDITY_SENSOR]))
        )
    cg.add(var.set_default_target_temperature(config[CONF_DEFAULT_TARGET_TEMPERATURE]))

    for name, preset in config[CONF_PRESETS].items():
        cg.add(
            var.add_preset(
                PRESETS[name],
                preset[CONF_TARGET_TEMPERATURE],
                preset.get(CONF_VALVE_POSITION, cg.RawExpression("NAN")),
            )
        )

    control = config[CONF_CONTROL]
    cg.add(var.set_kp(control[CONF_KP]))
    cg.add(var.set_ki(control[CONF_KI]))
    cg.add(var.set_deadband(control[CONF_DEADBAND]))
    cg.add(var.set_min_step(control[CONF_MIN_STEP]))
    cg.add(var.set_max_position(control[CONF_MAX_POSITION]))

    if CONF_VALVE_HELD in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_VALVE_HELD])
        cg.add(var.set_held_sensor(sens))

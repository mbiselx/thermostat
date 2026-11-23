import re

import esphome.codegen as cg
from esphome.components import binary_sensor, climate, sensor, text
import esphome.config_validation as cv
from esphome.const import (
    CONF_HUMIDITY_SENSOR,
    CONF_INITIAL_VALUE,
    CONF_SENSOR,
    CONF_TARGET_TEMPERATURE,
)

from . import valve_actuator_ns
from .valve import ValveActuator

AUTO_LOAD = ["binary_sensor", "sensor", "text"]

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
CONF_TEMPERATURE_ENTITY = "temperature_entity"

ENTITY_ID_PATTERN = r"^$|^sensor\.[a-z0-9_]+$"
TEXT_MAX_LENGTH = 100

RadiatorClimate = valve_actuator_ns.class_(
    "RadiatorClimate", climate.Climate, cg.PollingComponent
)
TemperatureEntityText = valve_actuator_ns.class_(
    "TemperatureEntityText", text.Text, cg.Component
)


def sensor_entity_id(value):
    value = cv.string_strict(value).strip()
    if not re.fullmatch(ENTITY_ID_PATTERN, value):
        raise cv.Invalid(
            f"'{value}' isn't a sensor entity ID like sensor.room_temperature"
        )
    return value


# A text entity in Home Assistant holding the entity ID of the sensor to take
# the temperature from. initial_value is used until someone changes it.
TEMPERATURE_ENTITY_SCHEMA = cv.All(
    text.text_schema(
        TemperatureEntityText,
        entity_category="config",
        icon="mdi:thermometer",
        mode="TEXT",
    )
    .extend(
        {
            cv.Optional(CONF_INITIAL_VALUE, default=""): sensor_entity_id,
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    cv.requires_component("api"),
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
        # A different sensor for this preset; empty = the main one.
        cv.Optional(CONF_TEMPERATURE_ENTITY): TEMPERATURE_ENTITY_SCHEMA,
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

CONFIG_SCHEMA = cv.All(
    climate.climate_schema(RadiatorClimate)
    .extend(
        {
            cv.GenerateID(CONF_VALVE_ID): cv.use_id(ValveActuator),
            # Home Assistant sensor to follow, chosen in Home Assistant.
            cv.Optional(CONF_TEMPERATURE_ENTITY): TEMPERATURE_ENTITY_SCHEMA,
            # A sensor on the device, used when no Home Assistant one is chosen.
            cv.Optional(CONF_SENSOR): cv.use_id(sensor.Sensor),
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
    .extend(cv.polling_component_schema("60s")),
    cv.has_at_least_one_key(CONF_TEMPERATURE_ENTITY, CONF_SENSOR),
)


async def temperature_entity_text(config, thermostat, *preset):
    var = await text.new_text(
        config, min_length=0, max_length=TEXT_MAX_LENGTH, pattern=ENTITY_ID_PATTERN
    )
    await cg.register_component(var, config)
    cg.add(var.set_initial_value(config[CONF_INITIAL_VALUE]))
    cg.add(var.set_thermostat(thermostat, *preset))
    cg.add_define("USE_API_HOMEASSISTANT_STATES")


async def to_code(config):
    var = await climate.new_climate(config)
    await cg.register_component(var, config)

    valve = await cg.get_variable(config[CONF_VALVE_ID])
    cg.add(var.set_valve(valve))
    if CONF_SENSOR in config:
        cg.add(var.set_sensor(await cg.get_variable(config[CONF_SENSOR])))
    if CONF_TEMPERATURE_ENTITY in config:
        await temperature_entity_text(config[CONF_TEMPERATURE_ENTITY], var)
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
        if CONF_TEMPERATURE_ENTITY in preset:
            await temperature_entity_text(
                preset[CONF_TEMPERATURE_ENTITY], var, PRESETS[name]
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

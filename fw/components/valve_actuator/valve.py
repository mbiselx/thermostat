import math

from esphome import pins
import esphome.codegen as cg
from esphome.components import binary_sensor, sensor, text_sensor, uart, valve
import esphome.config_validation as cv
from esphome.const import (
    CONF_ACCELERATION,
    CONF_ADDRESS,
    CONF_CURRENT,
    CONF_DIR_PIN,
    CONF_ENABLE_PIN,
    CONF_MAX_SPEED,
    CONF_ON_BOOT,
    CONF_SPEED,
    CONF_STATUS,
    CONF_STEP_PIN,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_DEGREES,
)

from . import valve_actuator_ns

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["binary_sensor", "sensor", "text_sensor"]

ValveActuator = valve_actuator_ns.class_(
    "ValveActuator", valve.Valve, cg.Component, uart.UARTDevice
)

CONF_CLK_PIN = "clk_pin"
CONF_SPREAD_PIN = "spread_pin"
CONF_SENSE_RESISTOR = "sense_resistor"
CONF_RUN_CURRENT = "run_current"
CONF_HOLD_CURRENT = "hold_current"
CONF_MICROSTEPS = "microsteps"
CONF_STEPS_PER_REVOLUTION = "steps_per_revolution"
CONF_GEAR_RATIO = "gear_ratio"
CONF_TRAVEL = "travel"
CONF_SPREAD_CYCLE = "spread_cycle"
CONF_INVERT_DIRECTION = "invert_direction"
CONF_HOLD_WHEN_IDLE = "hold_when_idle"
CONF_OPEN_LIMIT = "open_limit"
CONF_CLOSED_LIMIT = "closed_limit"
CONF_HOMING = "homing"
CONF_STALL_THRESHOLD = "stall_threshold"
CONF_MAX_TRAVEL = "max_travel"
CONF_BACKOFF = "backoff"
CONF_STALLGUARD = "stallguard"
CONF_MEASURED_TRAVEL = "measured_travel"


def _max_current(sense_resistor):
    # CS = 31 with vsense = 0 (TMC2209 datasheet).
    return 0.325 / (sense_resistor + 0.02) / math.sqrt(2)


def _validate(config):
    limit = _max_current(config[CONF_SENSE_RESISTOR])
    currents = [(CONF_RUN_CURRENT, config[CONF_RUN_CURRENT])]
    if CONF_CURRENT in config.get(CONF_HOMING, {}):
        currents.append(("homing current", config[CONF_HOMING][CONF_CURRENT]))
    for name, value in currents:
        if value > limit:
            raise cv.Invalid(
                f"{name} {value:.2f} A is above the {limit:.2f} A this sense "
                "resistor allows"
            )
    if CONF_TRAVEL not in config and CONF_HOMING not in config:
        raise cv.Invalid(
            f"Set {CONF_TRAVEL}, or add a {CONF_HOMING} block so calibration "
            "can measure it"
        )
    return config


# Finding the end stops with StallGuard. Speeds and angles are valve degrees.
HOMING_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_ON_BOOT, default=True): cv.boolean,
        cv.Optional(CONF_SPEED, default=45.0): cv.positive_float,
        # Defaults to run_current.
        cv.Optional(CONF_CURRENT): cv.All(cv.current, cv.Range(min=0.05)),
        # A stall is StallGuard below this share of its free-running value.
        cv.Optional(CONF_STALL_THRESHOLD, default="50%"): cv.All(
            cv.percentage, cv.Range(min=0.05, max=0.95)
        ),
        # How far to search for an end stop before giving up.
        cv.Optional(CONF_MAX_TRAVEL, default="1080°"): cv.All(
            cv.angle, cv.Range(min=1.0)
        ),
        # Move this far off the closed end stop after finding it.
        cv.Optional(CONF_BACKOFF, default="5°"): cv.All(cv.angle, cv.Range(min=0.0)),
    }
)


CONFIG_SCHEMA = cv.All(
    valve.valve_schema(ValveActuator)
    .extend(
        {
            cv.Required(CONF_STEP_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_DIR_PIN): pins.gpio_output_pin_schema,
            # Logical "on" = motor powered; use `inverted: true` for the active-low ENN.
            cv.Required(CONF_ENABLE_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_CLK_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_SPREAD_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_ADDRESS): cv.int_range(min=0, max=3),
            cv.Optional(CONF_SENSE_RESISTOR, default="0.12 ohm"): cv.All(
                cv.resistance, cv.Range(min=0.05, max=1.0)
            ),
            cv.Optional(CONF_RUN_CURRENT, default="1.0 A"): cv.All(
                cv.current, cv.Range(min=0.05)
            ),
            cv.Optional(CONF_HOLD_CURRENT, default="30%"): cv.percentage,
            cv.Optional(CONF_MICROSTEPS, default=16): cv.one_of(
                1, 2, 4, 8, 16, 32, 64, 128, 256, int=True
            ),
            cv.Optional(CONF_STEPS_PER_REVOLUTION, default=200): cv.int_range(
                min=1, max=1000
            ),
            # Motor revolutions per valve revolution.
            cv.Optional(CONF_GEAR_RATIO, default=1.0): cv.positive_float,
            # Valve rotation from fully closed to fully open. Measured by
            # calibration when left out.
            cv.Optional(CONF_TRAVEL): cv.All(cv.angle, cv.Range(min=1.0)),
            # Valve degrees per second, and per second squared.
            cv.Optional(CONF_MAX_SPEED, default=90.0): cv.positive_float,
            cv.Optional(CONF_ACCELERATION, default=180.0): cv.positive_float,
            cv.Optional(CONF_SPREAD_CYCLE, default=True): cv.boolean,
            cv.Optional(CONF_INVERT_DIRECTION, default=False): cv.boolean,
            cv.Optional(CONF_HOLD_WHEN_IDLE, default=False): cv.boolean,
            cv.Optional(CONF_OPEN_LIMIT): cv.use_id(binary_sensor.BinarySensor),
            cv.Optional(CONF_CLOSED_LIMIT): cv.use_id(binary_sensor.BinarySensor),
            cv.Optional(CONF_HOMING): HOMING_SCHEMA,
            cv.Optional(CONF_STALLGUARD): sensor.sensor_schema(
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:gauge",
            ),
            cv.Optional(CONF_MEASURED_TRAVEL): sensor.sensor_schema(
                unit_of_measurement=UNIT_DEGREES,
                accuracy_decimals=0,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:rotate-right",
            ),
            cv.Optional(CONF_STATUS): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:information-outline",
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA),
    _validate,
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "valve_actuator", require_tx=True, require_rx=True
)


async def to_code(config):
    var = await valve.new_valve(config)
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    for key, setter in (
        (CONF_STEP_PIN, var.set_step_pin),
        (CONF_DIR_PIN, var.set_dir_pin),
        (CONF_ENABLE_PIN, var.set_enable_pin),
        (CONF_CLK_PIN, var.set_clk_pin),
        (CONF_SPREAD_PIN, var.set_spread_pin),
    ):
        if key in config:
            cg.add(setter(await cg.gpio_pin_expression(config[key])))

    if CONF_ADDRESS in config:
        cg.add(var.set_address(config[CONF_ADDRESS]))
    cg.add(var.set_sense_resistor(config[CONF_SENSE_RESISTOR]))
    cg.add(var.set_run_current(config[CONF_RUN_CURRENT]))
    cg.add(var.set_hold_current(config[CONF_HOLD_CURRENT]))
    cg.add(var.set_microsteps(config[CONF_MICROSTEPS]))
    cg.add(var.set_steps_per_revolution(config[CONF_STEPS_PER_REVOLUTION]))
    cg.add(var.set_gear_ratio(config[CONF_GEAR_RATIO]))
    if CONF_TRAVEL in config:
        cg.add(var.set_travel(config[CONF_TRAVEL]))
    cg.add(var.set_max_speed(config[CONF_MAX_SPEED]))
    cg.add(var.set_acceleration(config[CONF_ACCELERATION]))
    cg.add(var.set_spread_cycle(config[CONF_SPREAD_CYCLE]))
    cg.add(var.set_invert_direction(config[CONF_INVERT_DIRECTION]))
    cg.add(var.set_hold_when_idle(config[CONF_HOLD_WHEN_IDLE]))

    if CONF_OPEN_LIMIT in config:
        cg.add(var.set_open_limit(await cg.get_variable(config[CONF_OPEN_LIMIT])))
    if CONF_CLOSED_LIMIT in config:
        cg.add(var.set_closed_limit(await cg.get_variable(config[CONF_CLOSED_LIMIT])))

    if homing := config.get(CONF_HOMING):
        cg.add(var.set_homing_enabled(True))
        cg.add(var.set_home_on_boot(homing[CONF_ON_BOOT]))
        cg.add(var.set_homing_speed(homing[CONF_SPEED]))
        if CONF_CURRENT in homing:
            cg.add(var.set_homing_current(homing[CONF_CURRENT]))
        cg.add(var.set_stall_threshold(homing[CONF_STALL_THRESHOLD]))
        cg.add(var.set_max_travel(homing[CONF_MAX_TRAVEL]))
        cg.add(var.set_backoff(homing[CONF_BACKOFF]))

    if CONF_STALLGUARD in config:
        sens = await sensor.new_sensor(config[CONF_STALLGUARD])
        cg.add(var.set_stallguard_sensor(sens))
    if CONF_MEASURED_TRAVEL in config:
        sens = await sensor.new_sensor(config[CONF_MEASURED_TRAVEL])
        cg.add(var.set_travel_sensor(sens))
    if CONF_STATUS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_STATUS])
        cg.add(var.set_status_sensor(sens))

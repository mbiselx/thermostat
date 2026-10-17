import math

from esphome import pins
import esphome.codegen as cg
from esphome.components import binary_sensor, uart, valve
import esphome.config_validation as cv
from esphome.const import (
    CONF_ACCELERATION,
    CONF_ADDRESS,
    CONF_DIR_PIN,
    CONF_ENABLE_PIN,
    CONF_MAX_SPEED,
    CONF_STEP_PIN,
)

from . import valve_actuator_ns

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["binary_sensor"]

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


def _max_current(sense_resistor):
    # CS = 31 with vsense = 0 (TMC2209 datasheet).
    return 0.325 / (sense_resistor + 0.02) / math.sqrt(2)


def _validate_current(config):
    limit = _max_current(config[CONF_SENSE_RESISTOR])
    if config[CONF_RUN_CURRENT] > limit:
        raise cv.Invalid(
            f"run_current {config[CONF_RUN_CURRENT]:.2f} A is above the "
            f"{limit:.2f} A this sense resistor allows"
        )
    return config


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
            # Valve rotation from fully closed to fully open.
            cv.Required(CONF_TRAVEL): cv.All(cv.angle, cv.Range(min=1.0)),
            # Valve degrees per second, and per second squared.
            cv.Optional(CONF_MAX_SPEED, default=90.0): cv.positive_float,
            cv.Optional(CONF_ACCELERATION, default=180.0): cv.positive_float,
            cv.Optional(CONF_SPREAD_CYCLE, default=True): cv.boolean,
            cv.Optional(CONF_INVERT_DIRECTION, default=False): cv.boolean,
            cv.Optional(CONF_HOLD_WHEN_IDLE, default=False): cv.boolean,
            cv.Optional(CONF_OPEN_LIMIT): cv.use_id(binary_sensor.BinarySensor),
            cv.Optional(CONF_CLOSED_LIMIT): cv.use_id(binary_sensor.BinarySensor),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA),
    _validate_current,
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

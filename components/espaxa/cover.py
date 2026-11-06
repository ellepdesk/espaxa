import esphome.codegen as cg
from esphome.components import binary_sensor, cover, uart
import esphome.config_validation as cv
from esphome.const import DEVICE_CLASS_LOCK

from . import espaxa_ns

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["binary_sensor"]

CONF_UNLOCK_DURATION = "unlock_duration"
CONF_OPEN_DURATION = "open_duration"
CONF_LOCK_DURATION = "lock_duration"
CONF_POLL_INTERVAL = "poll_interval"
CONF_MOVING_POLL_INTERVAL = "moving_poll_interval"
CONF_RELEASE_BEFORE_CLOSE = "release_before_close"
CONF_LOCK = "lock"

EspAxaCover = espaxa_ns.class_(
    "EspAxaCover", cover.Cover, cg.Component, uart.UARTDevice
)

CONFIG_SCHEMA = (
    cover.cover_schema(EspAxaCover, device_class="window")
    .extend(
        {
            # Time between OPEN and the chain starting to move (unlocking)
            cv.Optional(
                CONF_UNLOCK_DURATION, default="10s"
            ): cv.positive_time_period_milliseconds,
            # Time for the chain to travel from closed to fully open
            cv.Optional(
                CONF_OPEN_DURATION, default="50s"
            ): cv.positive_time_period_milliseconds,
            # Time after reaching closed before the window is locked
            cv.Optional(
                CONF_LOCK_DURATION, default="45s"
            ): cv.positive_time_period_milliseconds,
            # STATUS poll interval while idle, detects use of the AXA buttons
            cv.Optional(
                CONF_POLL_INTERVAL, default="30s"
            ): cv.positive_time_period_milliseconds,
            # STATUS poll interval while moving, detects locking early
            cv.Optional(
                CONF_MOVING_POLL_INTERVAL, default="2s"
            ): cv.positive_time_period_milliseconds,
            # Send a short OPEN before each CLOSE to release the AXA
            cv.Optional(CONF_RELEASE_BEFORE_CLOSE, default=True): cv.boolean,
            # On only while the AXA does not report 211 Strong Locked
            cv.Optional(CONF_LOCK): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_LOCK
            ),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "espaxa",
    baud_rate=19200,
    require_tx=True,
    require_rx=True,
    stop_bits=2,
)


async def to_code(config):
    var = await cover.new_cover(config)
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_unlock_duration(config[CONF_UNLOCK_DURATION]))
    cg.add(var.set_open_duration(config[CONF_OPEN_DURATION]))
    cg.add(var.set_lock_duration(config[CONF_LOCK_DURATION]))
    cg.add(var.set_poll_interval(config[CONF_POLL_INTERVAL]))
    cg.add(var.set_moving_poll_interval(config[CONF_MOVING_POLL_INTERVAL]))
    cg.add(var.set_release_before_close(config[CONF_RELEASE_BEFORE_CLOSE]))
    if lock_config := config.get(CONF_LOCK):
        sens = await binary_sensor.new_binary_sensor(lock_config)
        cg.add(var.set_lock_sensor(sens))

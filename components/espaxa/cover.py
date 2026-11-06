import esphome.codegen as cg
from esphome.components import cover, uart
import esphome.config_validation as cv

from . import espaxa_ns

DEPENDENCIES = ["uart"]

CONF_UNLOCK_DURATION = "unlock_duration"
CONF_OPEN_DURATION = "open_duration"
CONF_LOCK_DURATION = "lock_duration"
CONF_POLL_INTERVAL = "poll_interval"
CONF_MOVING_POLL_INTERVAL = "moving_poll_interval"

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
                CONF_LOCK_DURATION, default="30s"
            ): cv.positive_time_period_milliseconds,
            # STATUS poll interval while idle, detects use of the AXA buttons
            cv.Optional(
                CONF_POLL_INTERVAL, default="30s"
            ): cv.positive_time_period_milliseconds,
            # STATUS poll interval while moving, detects locking early
            cv.Optional(
                CONF_MOVING_POLL_INTERVAL, default="2s"
            ): cv.positive_time_period_milliseconds,
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

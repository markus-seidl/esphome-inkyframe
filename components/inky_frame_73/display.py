from esphome import automation, core, pins
import esphome.codegen as cg
from esphome.components import display, spi
from esphome.components.display import CONF_SHOW_TEST_CARD, validate_rotation
import esphome.config_validation as cv
from esphome.config_validation import update_interval
from esphome.const import (
    CONF_AUTO_CLEAR_ENABLED,
    CONF_CS_PIN,
    CONF_DC_PIN,
    CONF_ID,
    CONF_LAMBDA,
    CONF_PAGES,
    CONF_RESET_PIN,
    CONF_ROTATION,
    CONF_UPDATE_INTERVAL,
)
from esphome.final_validate import full_config

AUTO_LOAD = ["split_buffer"]
DEPENDENCIES = ["spi"]

CONF_SR_CLOCK_PIN = "sr_clock_pin"
CONF_SR_LATCH_PIN = "sr_latch_pin"
CONF_SR_DATA_PIN = "sr_data_pin"
CONF_BUSY_TIMEOUT = "busy_timeout"

WIDTH = 800
HEIGHT = 480

inky_frame_73_ns = cg.esphome_ns.namespace("inky_frame_73")
InkyFrame73 = inky_frame_73_ns.class_(
    "InkyFrame73", cg.PollingComponent, spi.SPIDevice, display.Display
)

automation.register_apply_condition(
    "inky_frame_73.is_updating",
    automation.maybe_simple_id({cv.Required(CONF_ID): cv.use_id(InkyFrame73)}),
    "is_updating()",
)


def _add_metadata(config):
    display.add_metadata(
        config[CONF_ID],
        WIDTH,
        HEIGHT,
        has_hardware_rotation=True,
        byte_order=cv.UNDEFINED,
        has_writer=config.get(CONF_AUTO_CLEAR_ENABLED) is True
        or config.get(CONF_PAGES) is not None
        or config.get(CONF_LAMBDA) is not None
        or config.get(CONF_SHOW_TEST_CARD) is True,
        rotation=config.get(CONF_ROTATION, 0),
        draw_rounding=0,
    )
    return config


# The defaults are the pin assignments of the Inky Frame board.
CONFIG_SCHEMA = cv.All(
    display.FULL_DISPLAY_SCHEMA.extend(
        spi.spi_device_schema(
            cs_pin_required=False,
            default_mode="MODE0",
            default_data_rate="20MHz",
        )
    ).extend(
        {
            cv.GenerateID(): cv.declare_id(InkyFrame73),
            cv.Optional(CONF_ROTATION, default=0): validate_rotation,
            cv.Optional(CONF_UPDATE_INTERVAL, default=cv.UNDEFINED): cv.All(
                update_interval, cv.Range(min=core.TimePeriod(seconds=30))
            ),
            cv.Optional(CONF_CS_PIN, default="GPIO17"): pins.gpio_output_pin_schema,
            cv.Optional(CONF_DC_PIN, default="GPIO28"): pins.gpio_output_pin_schema,
            cv.Optional(
                CONF_RESET_PIN, default="GPIO27"
            ): pins.gpio_output_pin_schema,
            cv.Optional(
                CONF_SR_CLOCK_PIN, default="GPIO8"
            ): pins.gpio_output_pin_schema,
            cv.Optional(
                CONF_SR_LATCH_PIN, default="GPIO9"
            ): pins.gpio_output_pin_schema,
            cv.Optional(
                CONF_SR_DATA_PIN, default="GPIO10"
            ): pins.gpio_input_pin_schema,
            cv.Optional(
                CONF_BUSY_TIMEOUT, default="45s"
            ): cv.positive_time_period_milliseconds,
        }
    ),
    cv.only_on_rp2,
    _add_metadata,
)


def _final_validate(config):
    spi.final_validate_device_schema(
        "inky_frame_73", require_miso=False, require_mosi=True
    )(config)

    if CONF_LAMBDA not in config and CONF_PAGES not in config:
        if "lvgl" in full_config.get():
            if CONF_UPDATE_INTERVAL not in config:
                config[CONF_UPDATE_INTERVAL] = update_interval("never")
        else:
            # Nothing to draw with: show the test card so the wiring can be verified
            config[CONF_SHOW_TEST_CARD] = True
            if CONF_UPDATE_INTERVAL not in config:
                config[CONF_UPDATE_INTERVAL] = update_interval("1min")
    elif CONF_UPDATE_INTERVAL not in config:
        config[CONF_UPDATE_INTERVAL] = update_interval("5min")
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await display.register_display(var, config)
    await spi.register_spi_device(var, config, write_only=True)

    for key, setter in (
        (CONF_DC_PIN, var.set_dc_pin),
        (CONF_RESET_PIN, var.set_reset_pin),
        (CONF_SR_CLOCK_PIN, var.set_sr_clock_pin),
        (CONF_SR_LATCH_PIN, var.set_sr_latch_pin),
        (CONF_SR_DATA_PIN, var.set_sr_data_pin),
    ):
        cg.add(setter(await cg.gpio_pin_expression(config[key])))

    cg.add(var.set_busy_timeout(config[CONF_BUSY_TIMEOUT]))

    if CONF_LAMBDA in config:
        lambda_ = await cg.process_lambda(
            config[CONF_LAMBDA], [(display.DisplayRef, "it")], return_type=cg.void
        )
        cg.add(var.set_writer(lambda_))

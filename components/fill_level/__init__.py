"""Measure the colored fill column in ESP32 camera frames."""

from pathlib import Path

from PIL import Image, UnidentifiedImageError

import esphome.codegen as cg
from esphome.components import esp32_camera, light, sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.core import CORE, HexInt, ID

DEPENDENCIES = ["esp32_camera", "psram"]
AUTO_LOAD = ["sensor"]

CONF_CAMERA_ID = "camera_id"
CONF_TEMPLATES = "templates"
CONF_FLASHLIGHT_ID = "flashlight_id"
CONF_STABILIZATION_DELAY = "stabilization_delay"
CONF_FILL_PERCENTAGE = "fill_percentage"
CONF_MATCH_CONFIDENCE = "match_confidence"
CONF_MIN_CONFIDENCE = "min_confidence"
CONF_MIN_BLOB_AREA = "min_blob_area"
CONF_INTERVAL = "interval"

fill_level_ns = cg.esphome_ns.namespace("fill_level")
FillLevel = fill_level_ns.class_("FillLevel", cg.Component)


def _template_path(value):
    path = CORE.relative_config_path(cv.string_strict(value))
    if not path.is_file():
        raise cv.Invalid(f"Template image does not exist: {path}")
    try:
        with Image.open(path) as image:
            width, height = image.size
            if not (16 <= width <= 640 and 121 <= height <= 480):
                raise cv.Invalid(
                    f"Template {path} must be 16..640 pixels wide and 121..480 pixels high"
                )
    except (OSError, UnidentifiedImageError) as exc:
        raise cv.Invalid(f"Cannot read template {path}: {exc}") from exc
    return str(path)


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(FillLevel),
        cv.Required(CONF_CAMERA_ID): cv.use_id(esp32_camera.ESP32Camera),
        cv.Required(CONF_FLASHLIGHT_ID): cv.use_id(light.LightState),
        cv.Required(CONF_TEMPLATES): cv.All(
            cv.ensure_list(_template_path), cv.Length(min=1, max=8)
        ),
        cv.Required(CONF_FILL_PERCENTAGE): sensor.sensor_schema(
            unit_of_measurement="%", accuracy_decimals=2, icon="mdi:water-percent",
            state_class="measurement",
        ),
        cv.Optional(CONF_MATCH_CONFIDENCE): sensor.sensor_schema(
            accuracy_decimals=2, icon="mdi:image-search", state_class="measurement"
        ),
        cv.Optional(CONF_MIN_CONFIDENCE, default=0.4): cv.float_range(min=0.0, max=1.0),
        cv.Optional(CONF_MIN_BLOB_AREA, default=150): cv.int_range(min=1, max=65535),
        cv.Optional(CONF_INTERVAL, default="12h"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_STABILIZATION_DELAY, default="3s"): cv.positive_time_period_milliseconds,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    camera = await cg.get_variable(config[CONF_CAMERA_ID])
    cg.add(var.set_camera(camera))
    flashlight = await cg.get_variable(config[CONF_FLASHLIGHT_ID])
    cg.add(var.set_flashlight(flashlight))
    cg.add(var.set_fill_sensor(await sensor.new_sensor(config[CONF_FILL_PERCENTAGE])))
    if CONF_MATCH_CONFIDENCE in config:
        cg.add(var.set_confidence_sensor(await sensor.new_sensor(config[CONF_MATCH_CONFIDENCE])))
    cg.add(var.set_min_confidence(config[CONF_MIN_CONFIDENCE]))
    cg.add(var.set_min_blob_area(config[CONF_MIN_BLOB_AREA]))
    cg.add(var.set_interval(config[CONF_INTERVAL].total_milliseconds))
    cg.add(var.set_stabilization_delay(config[CONF_STABILIZATION_DELAY].total_milliseconds))

    for index, filename in enumerate(config[CONF_TEMPLATES]):
        with Image.open(Path(filename)) as image:
            grayscale = image.convert("L")
            width, height = grayscale.size
            pixels = grayscale.tobytes()
        data_id = ID(f"fill_template_{config[CONF_ID].id}_{index}", type=cg.uint8)
        data = cg.progmem_array(data_id, [HexInt(pixel) for pixel in pixels])
        cg.add(var.add_template(data, width, height))

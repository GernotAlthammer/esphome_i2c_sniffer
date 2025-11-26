import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.const import CONF_ID, CONF_TRIGGER_ID
from esphome.components import sensor, text_sensor

# =================================================================
# 1. Code Generation Setup
# =================================================================

# Define the C++ namespace and class for easy reference in the Python code
esphome_i2c_sniffer_ns = cg.esphome_ns.namespace("esphome_i2c_sniffer")
# Define the C++ class derived from the ESPHome base Component class
EsphomeI2cSniffer = esphome_i2c_sniffer_ns.class_("EsphomeI2cSniffer", cg.Component)

# =================================================================
# 2. Configuration Constants and Schema Definition
# =================================================================

# Configuration key constants used in the YAML schema
CONF_SDA_PIN = "sda_pin"
CONF_SCL_PIN = "scl_pin"
CONF_MSG_SENSOR = "msg_sensor"
CONF_LAST_ADDR_SENSOR = "last_address_sensor"
CONF_LAST_DATA_SENSOR = "last_data_sensor"
CONF_LAST_BYTE_SENSOR = "last_byte_sensor"
CONF_ON_ADDRESS = "on_address" # Key for the automation trigger

# Define the overall configuration schema (YAML structure)
CONFIG_SCHEMA = cv.Schema({
    # Mandatory: Generate C++ ID for the main component instance
    cv.GenerateID(): cv.declare_id(EsphomeI2cSniffer),
    
    # Required pins for the sniffer hardware
    cv.Required(CONF_SDA_PIN): cv.uint8_t,
    cv.Required(CONF_SCL_PIN): cv.uint8_t,
    
    # Optional output sensors definition
    cv.Optional(CONF_MSG_SENSOR): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_LAST_ADDR_SENSOR): sensor.sensor_schema(),
    cv.Optional(CONF_LAST_DATA_SENSOR): sensor.sensor_schema(),
    cv.Optional(CONF_LAST_BYTE_SENSOR): sensor.sensor_schema(),

    # Automation Trigger Definition: on_address
    cv.Optional(CONF_ON_ADDRESS): automation.validate_automation({
        # Generate C++ ID for the Trigger object, templated with the arguments
        # that will be passed to it: (uint8_t address, bool read_write)
        cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(automation.Trigger.template(cg.uint8, cg.bool_)),
    }),
}).extend(cv.COMPONENT_SCHEMA)

# =================================================================
# 3. Code Generation Function
# =================================================================

# The main function that runs during compilation to generate the C++ code
async def to_code(config):
    # Create the C++ component object instance (e.g., `auto i2c_sniffer = new EsphomeI2cSniffer();`)
    var = cg.new_Pvariable(config[CONF_ID])
    # Register the component lifecycle methods (setup, loop, etc.)
    await cg.register_component(var, config)

    # Set mandatory pin configuration
    cg.add(var.set_sda_pin(config[CONF_SDA_PIN]))
    cg.add(var.set_scl_pin(config[CONF_SCL_PIN]))

    # Set up optional sensors if configured in YAML
    if CONF_MSG_SENSOR in config:
        sens = await text_sensor.new_text_sensor(config[CONF_MSG_SENSOR])
        cg.add(var.set_msg_sensor(sens))
        
    if CONF_LAST_ADDR_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_LAST_ADDR_SENSOR])
        cg.add(var.set_last_address_sensor(sens))

    if CONF_LAST_DATA_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_LAST_DATA_SENSOR])
        cg.add(var.set_last_data_sensor(sens))

    if CONF_LAST_BYTE_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_LAST_BYTE_SENSOR])
        cg.add(var.set_last_byte_sensor(sens))

    # Handle all defined automation actions for on_address trigger
    for conf in config.get(CONF_ON_ADDRESS, []):
        # 1. Create the C++ Trigger object (e.g., `auto t = new Trigger<uint8_t, bool>();`)
        # We do NOT pass 'var' here because the base Trigger class has an empty constructor.
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        
        # 2. Build the automation code and actions (e.g., 'then: lambda: ...')
        # This generates a C++ lambda function that calls the trigger's internal actions.
        # The arguments passed to this lambda are defined here: (cg.uint8, "address") and (cg.bool_, "rw").
        await automation.build_automation(
            trigger, 
            [(cg.uint8, "address"), (cg.bool_, "rw")], 
            conf
        )
        
        # 3. Register the trigger with the main component instance
        # This calls EsphomeI2cSniffer::register_address_trigger(t) in C++.
        cg.add(var.register_address_trigger(trigger))

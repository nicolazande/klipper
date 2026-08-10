# Support fans that are controlled by gcode
#
# Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import fan, output_pin

# Module-level dictionary to hold fan objects
fan_objects = {}

class PrinterFanGeneric:
    cmd_SET_FAN_SPEED_help = "Sets the speed of a fan"
    def __init__(self, config):
        self.printer = config.get_printer()
        self.fan_name = config.get_name().split()[-1]
        
        # Check for master_fan parameter
        master_fan_short_name = config.get('master_fan', None)
        if master_fan_short_name:
            master_fan_full_name = f"fan_generic {master_fan_short_name}"
            master_fan = fan_objects.get(master_fan_full_name)
            if master_fan is not None \
               and master_fan.printer is not self.printer:
                # Stale entry from a previous session (the module dict
                # survives a soft RESTART); never bind across sessions
                master_fan = None
            if master_fan is None:
                raise config.error(f"Master fan '{master_fan_short_name}' not found")
            if not isinstance(master_fan, fan.Fan):
                raise config.error(f"Master fan '{master_fan_short_name}' is not a valid fan object")
            self.fan = fan.Fan(config, master_fan=master_fan, default_shutdown_speed=0.)
        else:
            self.fan = fan.Fan(config, default_shutdown_speed=0.)
        
        # Store the fan object in the module-level dictionary
        fan_objects[f"fan_generic {self.fan_name}"] = self.fan
        
        # Template handling
        self.template_eval = output_pin.lookup_template_eval(config)
        
        # Register G-code command
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("SET_FAN_SPEED", "FAN",
                                   self.fan_name,
                                   self.cmd_SET_FAN_SPEED,
                                   desc=self.cmd_SET_FAN_SPEED_help)

    def get_status(self, eventtime):
        return self.fan.get_status(eventtime)
    
    def _template_update(self, text):
        try:
            value = float(text)
        except ValueError as e:
            logging.exception("fan_generic template render error")
        self.fan.set_speed(value)
    
    def cmd_SET_FAN_SPEED(self, gcmd):
        speed = gcmd.get_float('SPEED', None, minval=0.)
        template = gcmd.get('TEMPLATE', None)
        if (speed is None) == (template is None):
            raise gcmd.error("SET_FAN_SPEED must specify SPEED or TEMPLATE")
        if template is not None:
            self.template_eval.set_template(gcmd, self._template_update)
            return
        self.fan.set_speed_from_command(speed)

def load_config_prefix(config):
    return PrinterFanGeneric(config)

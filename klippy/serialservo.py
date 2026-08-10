# Serialservo (TMC4671) axis support - setpoint streaming over serial
#
# Copyright (C) 2024  Nicola Zandegiacomo <nicola.zandegiacomo@flyingbasket.com>
# Copyright (C) 2026  Warmbird
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import collections, logging
import chelper

class error(Exception):
    pass

# Fixed SPI transfer lead compensated by the mcu when evaluating
# setpoints (seconds)
EVAL_LEAD_TIME = 0.000100
# Effective "step distance" reported to the homing code (sets the
# endstop poll rate; the servo has no steps of its own)
HOMING_SAMPLE_DIST = 0.010


class SerialServo:
    # Interface to the low-level mcu and chelper code for a TMC4671
    # servo axis.  Mirrors the MCU_stepper API surface used by the
    # kinematics, homing and stepper_enable code.
    def __init__(self, config, name, mcu, units_in_radians=False):
        self._name = name
        self._mcu = mcu
        self._units_in_radians = units_in_radians
        self._rotation_distance = config.getfloat('rotation_distance',
                                                  above=0.)
        self._pole_pairs = config.getint('pole_pairs', minval=1, maxval=120)
        self._sampling_time = config.getfloat('sampling_time', 0.010,
                                              minval=0.002, maxval=0.100)
        self._interpolation_time = config.getfloat(
            'interpolation_time', 0.001, minval=0.0005,
            maxval=self._sampling_time)
        self._following_error = config.getfloat('following_error', 2.0,
                                                minval=0.)
        self._ferror_check_time = config.getfloat(
            'following_error_check_time', 0.050, minval=0.010)
        self._oid = oid = self._mcu.create_oid()
        self._mcu.register_config_callback(self._build_config)
        self._mcu_position_offset = 0.
        self._reset_cmd_tag = self._get_position_cmd = None
        self._set_spi_cmd = self._query_state_cmd = None
        self._active_callbacks = []
        ffi_main, ffi_lib = chelper.get_ffi()
        self._ffi_lib = ffi_lib
        self._ffi_main = ffi_main
        self._stepqueue = ffi_main.gc(ffi_lib.serialservo_compress_alloc(oid),
                                      ffi_lib.serialservo_compress_free)
        self._mcu.register_stepqueue(self._stepqueue)
        self._stepper_kinematics = None
        self._known_kinematics = []
        self._itersolve_generate_steps = ffi_lib.serialservo_solve_generate_steps
        self._itersolve_check_active = ffi_lib.serialservo_solve_check_active
        self._trapq = ffi_main.NULL
        self._mcu.get_printer().register_event_handler(
            'klippy:connect', self._query_mcu_position)
    def get_mcu(self):
        return self._mcu
    def get_name(self, short=False):
        if short and self._name.startswith('serialservo_'):
            return self._name[len('serialservo_'):]
        return self._name
    def get_oid(self):
        return self._oid
    def units_in_radians(self):
        return self._units_in_radians
    def get_pole_pairs(self):
        return self._pole_pairs
    def get_rotation_distance(self):
        return self._rotation_distance
    def _position_to_units(self, pos):
        # mm to TMC4671 position register units (1/65536 electrical rev)
        return pos * self._pole_pairs * 65536. / self._rotation_distance
    def _build_config(self):
        mcu = self._mcu
        interp_ticks = max(1, mcu.seconds_to_clock(self._interpolation_time))
        mcu_freq = mcu.seconds_to_clock(1.)
        # Wire velocity is electrical rpm; scale converts rpm to
        # position lsb per clock tick (<<32) for mcu interpolation
        vel_scale = int(65536. * float(1 << 32) / (60. * mcu_freq) + .5)
        eval_lead = mcu.seconds_to_clock(EVAL_LEAD_TIME)
        ferror_window = 0
        if self._following_error:
            ferror_window = int(self._position_to_units(
                self._following_error) + .5)
        ferror_ticks = mcu.seconds_to_clock(self._ferror_check_time)
        mcu.add_config_cmd(
            "config_serialservo oid=%d interp_ticks=%u vel_scale=%u"
            " eval_lead=%u ferror_window=%u ferror_ticks=%u"
            % (self._oid, interp_ticks, vel_scale, eval_lead,
               ferror_window, ferror_ticks))
        mcu.add_config_cmd("serialservo_reset_step_clock oid=%d clock=0"
                           % (self._oid,), on_restart=True)
        step_cmd_tag = mcu.lookup_command(
            "serialservo_queue_step oid=%c target_position=%i"
            " target_velocity=%i clock=%u").get_command_tag()
        self._reset_cmd_tag = mcu.lookup_command(
            "serialservo_reset_step_clock oid=%c clock=%u").get_command_tag()
        self._get_position_cmd = mcu.lookup_query_command(
            "serialservo_get_position oid=%c",
            "serialservo_position oid=%c pos=%i", oid=self._oid)
        self._query_state_cmd = mcu.lookup_query_command(
            "serialservo_query_state oid=%c",
            "serialservo_state oid=%c clock=%u target=%i actual=%i"
            " velocity=%i", oid=self._oid)
        self._set_spi_cmd = mcu.lookup_command(
            "config_serialservo_spi oid=%c spi_oid=%c")
        self._ffi_lib.serialservo_compress_fill(
            self._stepqueue, step_cmd_tag, self._pole_pairs,
            self._rotation_distance)
    def setup_spi(self, spi_oid):
        # Bind the runtime SPI device (called by the tmc4671 driver
        # once chip configuration is complete)
        self._set_spi_cmd.send([self._oid, spi_oid])
    def query_state(self):
        # Return live drive state (used for telemetry/monitoring)
        if self._query_state_cmd is None or self._mcu.is_fileoutput():
            return None
        params = self._query_state_cmd.send([self._oid])
        scale = self._pole_pairs * 65536. / self._rotation_distance
        vel_scale = self._pole_pairs * 60. / self._rotation_distance
        return {
            'target': params['target'] / scale,
            'actual': params['actual'] / scale,
            'velocity': params['velocity'] / vel_scale,
            'mcu_clock': params['clock'],
        }
    def get_step_dist(self):
        return HOMING_SAMPLE_DIST
    def is_active_axis(self, axis):
        ffi_main, ffi_lib = chelper.get_ffi()
        return ffi_lib.serialservo_solve_is_active_axis(
            self._stepper_kinematics, axis.encode())
    def setup_itersolve(self, alloc_func, *params):
        ffi_main, ffi_lib = chelper.get_ffi()
        sk = ffi_main.gc(getattr(ffi_lib, alloc_func)(*params), ffi_lib.free)
        self._known_kinematics.append(sk)
        self.set_stepper_kinematics(sk)
    def get_stepper_kinematics(self):
        return self._stepper_kinematics
    def set_stepper_kinematics(self, sk):
        # Only kinematics allocated via setup_itersolve share the
        # serialservo solver ABI; reject foreign objects (FORCE_MOVE
        # style swaps) with a gcode error instead of corrupting memory
        if sk is not None and sk not in self._known_kinematics:
            raise self._mcu.get_printer().command_error(
                "serialservo %s does not support FORCE_MOVE/STEPPER_BUZZ"
                % (self._name,))
        old_sk = self._stepper_kinematics
        mcu_pos = 0.
        if old_sk is not None:
            mcu_pos = self._get_mcu_position_mm()
        self._stepper_kinematics = sk
        self._ffi_lib.serialservo_solve_set_stepcompress(
            sk, self._stepqueue, self._sampling_time)
        self.set_trapq(self._trapq)
        self._set_mcu_position(mcu_pos)
        return old_sk
    def calc_position_from_coord(self, coord):
        return self._ffi_lib.serialservo_solve_calc_position_from_coord(
            self._stepper_kinematics, coord[0], coord[1], coord[2])
    def set_position(self, coord):
        mcu_pos = self._get_mcu_position_mm()
        self._ffi_lib.serialservo_solve_set_position(
            self._stepper_kinematics, coord[0], coord[1], coord[2])
        self._set_mcu_position(mcu_pos)
    def get_commanded_position(self):
        return self._ffi_lib.serialservo_solve_get_commanded_pos(
            self._stepper_kinematics)
    def _get_mcu_position_mm(self):
        # Internal drive-frame position in mm
        return self.get_commanded_position() + self._mcu_position_offset
    def get_mcu_position(self, cmd_pos=None):
        # Positions reported to the homing code ("mcu positions") are
        # expressed in units of HOMING_SAMPLE_DIST: homing.py
        # multiplies mcu position offsets by get_step_dist(), so this
        # keeps its algebra exact in mm while the endstop poll rate
        # stays fine grained.
        if cmd_pos is None:
            cmd_pos = self.get_commanded_position()
        return (cmd_pos + self._mcu_position_offset) / HOMING_SAMPLE_DIST
    def _set_mcu_position(self, mcu_pos):
        self._mcu_position_offset = mcu_pos - self.get_commanded_position()
        # Outgoing wire positions are generated in the mcu frame
        self._ffi_lib.serialservo_compress_set_position_offset(
            self._stepqueue, self._mcu_position_offset)
    def mcu_to_commanded_position(self, mcu_pos):
        return mcu_pos * HOMING_SAMPLE_DIST - self._mcu_position_offset
    def get_past_mcu_position(self, print_time):
        clock = self._mcu.print_time_to_clock(print_time)
        pos = self._ffi_lib.serialservo_compress_find_past_position(
            self._stepqueue, clock)
        return pos / HOMING_SAMPLE_DIST
    def dump_steps(self, count, start_clock, end_clock):
        data = self._ffi_main.new('struct pull_history_serialservo_steps[]',
                                  count)
        count = self._ffi_lib.serialservo_compress_extract_old(
            self._stepqueue, data, count, start_clock, end_clock)
        return (data, count)
    def get_trapq(self):
        return self._trapq
    def set_trapq(self, tq):
        if tq is None:
            tq = self._ffi_main.NULL
        self._ffi_lib.serialservo_solve_set_trapq(
            self._stepper_kinematics, tq)
        old_tq = self._trapq
        self._trapq = tq
        return old_tq
    def add_active_callback(self, cb):
        self._active_callbacks.append(cb)
    def generate_steps(self, flush_time):
        if self._active_callbacks:
            sk = self._stepper_kinematics
            ret = self._itersolve_check_active(sk, flush_time)
            if ret:
                cbs = self._active_callbacks
                self._active_callbacks = []
                for cb in cbs:
                    cb(ret)
        ret = self._itersolve_generate_steps(self._stepper_kinematics,
                                             flush_time)
        if ret:
            raise error("Internal error in serialservo_compress")
    def get_stop_on_trigger_command(self):
        return "serialservo_stop_on_trigger oid=%c trsync_oid=%c"
    def note_homing_end(self):
        ret = self._ffi_lib.serialservo_compress_reset(self._stepqueue, 0)
        if ret:
            raise error("Internal error in serialservo_compress")
        data = (self._reset_cmd_tag, self._oid, 0)
        ret = self._ffi_lib.serialservo_compress_queue_msg(
            self._stepqueue, data, len(data))
        if ret:
            raise error("Internal error in serialservo_compress")
        self._query_mcu_position()
    def _query_mcu_position(self):
        if self._mcu.is_fileoutput():
            return
        params = self._get_position_cmd.send([self._oid])
        last_pos = params['pos']
        print_time = self._mcu.estimated_print_time(params['#receive_time'])
        clock = self._mcu.print_time_to_clock(print_time)
        last_pos_mm = self._ffi_lib.serialservo_compress_set_last_position(
            self._stepqueue, clock, last_pos)
        self._set_mcu_position(last_pos_mm)
        self._mcu.get_printer().send_event("stepper:sync_mcu_position", self)


def PrinterStepper(config, units_in_radians=False):
    # Build a serialservo object from a config section.  The mcu
    # hosting the servo is selected with the 'mcu' config option (the
    # TMC4671 board is typically a secondary mcu).
    printer = config.get_printer()
    name = config.get_name()
    mcu_name = config.get('mcu', 'mcu')
    if mcu_name == 'mcu':
        mcu = printer.lookup_object('mcu')
    else:
        mcu = printer.lookup_object('mcu ' + mcu_name)
    mcu_servo = SerialServo(config, name, mcu, units_in_radians)
    # Register with modules that expect a stepper-like object.  Note
    # motion_report is deliberately not registered: its dump format is
    # step based and does not apply to setpoint streams.
    for mname in ['stepper_enable', 'force_move']:
        m = printer.load_object(config, mname)
        m.register_stepper(config, mcu_servo)
    return mcu_servo


class PrinterRail:
    # A motor control rail for serialservo axes: one (or more) servos
    # and one (or more) endstops
    def __init__(self, config, need_position_minmax=True,
                 default_position_endstop=None, units_in_radians=False):
        self.stepper_units_in_radians = units_in_radians
        self.steppers = []
        self.endstops = []
        self.endstop_map = {}
        self.add_extra_stepper(config)
        mcu_stepper = self.steppers[0]
        self.get_name = mcu_stepper.get_name
        self.get_commanded_position = mcu_stepper.get_commanded_position
        self.calc_position_from_coord = mcu_stepper.calc_position_from_coord
        # Primary endstop position
        mcu_endstop = self.endstops[0][0]
        if hasattr(mcu_endstop, "get_position_endstop"):
            self.position_endstop = mcu_endstop.get_position_endstop()
        elif default_position_endstop is None:
            self.position_endstop = config.getfloat('position_endstop')
        else:
            self.position_endstop = config.getfloat(
                'position_endstop', default_position_endstop)
        # Axis range
        if need_position_minmax:
            self.position_min = config.getfloat('position_min', 0.)
            self.position_max = config.getfloat('position_max',
                                                above=self.position_min)
        else:
            self.position_min = 0.
            self.position_max = self.position_endstop
        if (self.position_endstop < self.position_min
                or self.position_endstop > self.position_max):
            raise config.error(
                "position_endstop in section '%s' must be between"
                " position_min and position_max" % config.get_name())
        # Homing mechanics
        self.homing_speed = config.getfloat('homing_speed', 5.0, above=0.)
        self.second_homing_speed = config.getfloat(
            'second_homing_speed', self.homing_speed / 2., above=0.)
        self.homing_retract_speed = config.getfloat(
            'homing_retract_speed', self.homing_speed, above=0.)
        self.homing_retract_dist = config.getfloat(
            'homing_retract_dist', 5., minval=0.)
        self.homing_positive_dir = config.getboolean(
            'homing_positive_dir', None)
        if self.homing_positive_dir is None:
            axis_len = self.position_max - self.position_min
            if self.position_endstop <= self.position_min + axis_len / 4.:
                self.homing_positive_dir = False
            elif self.position_endstop >= self.position_max - axis_len / 4.:
                self.homing_positive_dir = True
            else:
                raise config.error(
                    "Unable to infer homing_positive_dir in section '%s'"
                    % (config.get_name(),))
            config.getboolean('homing_positive_dir', self.homing_positive_dir)
        elif ((self.homing_positive_dir
               and self.position_endstop == self.position_min)
              or (not self.homing_positive_dir
                  and self.position_endstop == self.position_max)):
            raise config.error(
                "Invalid homing_positive_dir / position_endstop in '%s'"
                % (config.get_name(),))
    def get_range(self):
        return self.position_min, self.position_max
    def get_homing_info(self):
        homing_info = collections.namedtuple('homing_info', [
            'speed', 'position_endstop', 'retract_speed', 'retract_dist',
            'positive_dir', 'second_homing_speed'])(
                self.homing_speed, self.position_endstop,
                self.homing_retract_speed, self.homing_retract_dist,
                self.homing_positive_dir, self.second_homing_speed)
        return homing_info
    def get_steppers(self):
        return list(self.steppers)
    def get_endstops(self):
        return list(self.endstops)
    def add_extra_stepper(self, config):
        stepper = PrinterStepper(config, self.stepper_units_in_radians)
        self.steppers.append(stepper)
        if self.endstops and config.get('endstop_pin', None) is None:
            # No endstop defined - use primary endstop
            self.endstops[0][0].add_stepper(stepper)
            return
        endstop_pin = config.get('endstop_pin')
        printer = config.get_printer()
        ppins = printer.lookup_object('pins')
        pin_params = ppins.parse_pin(endstop_pin, True, True)
        pin_name = "%s:%s" % (pin_params['chip_name'], pin_params['pin'])
        endstop = self.endstop_map.get(pin_name, None)
        if endstop is None:
            mcu_endstop = ppins.setup_pin('endstop', endstop_pin)
            self.endstop_map[pin_name] = {'endstop': mcu_endstop,
                                          'invert': pin_params['invert'],
                                          'pullup': pin_params['pullup']}
            name = stepper.get_name(short=True)
            self.endstops.append((mcu_endstop, name))
            query_endstops = printer.load_object(config, 'query_endstops')
            query_endstops.register_endstop(mcu_endstop, name)
        else:
            mcu_endstop = endstop['endstop']
            changed_invert = pin_params['invert'] != endstop['invert']
            changed_pullup = pin_params['pullup'] != endstop['pullup']
            if changed_invert or changed_pullup:
                raise error("Printer rail %s shared endstop pin %s "
                            "must specify the same pullup/invert settings" % (
                                self.get_name(), pin_name))
        mcu_endstop.add_stepper(stepper)
    def setup_itersolve(self, alloc_func, *params):
        for stepper in self.steppers:
            stepper.setup_itersolve(alloc_func, *params)
    def generate_steps(self, flush_time):
        for stepper in self.steppers:
            stepper.generate_steps(flush_time)
    def set_trapq(self, trapq):
        for stepper in self.steppers:
            stepper.set_trapq(trapq)
    def set_position(self, coord):
        for stepper in self.steppers:
            stepper.set_position(coord)


def LookupMultiRail(config, need_position_minmax=True,
                    default_position_endstop=None, units_in_radians=False):
    # Wrapper for dual servo support
    rail = PrinterRail(config, need_position_minmax,
                       default_position_endstop, units_in_radians)
    for i in range(1, 99):
        if not config.has_section(config.get_name() + str(i)):
            break
        rail.add_extra_stepper(config.getsection(config.get_name() + str(i)))
    return rail

#
# Printer pvt servo support.
#

# imports
import os, sys, math, logging, collections
import chelper

class error(Exception):
    pass


class SerialServo:
    '''
    Interface to low-level mcu and chelper code for pvt drive.
    '''
    def __init__(self, name, mcu, units_in_radians, simtime, pole_pairs, scaler):
        # parameters
        self._name = name
        self._units_in_radians = units_in_radians
        self._step_dist = 1.0
        self._simtime = simtime
        self._pole_pairs = pole_pairs
        self._scaler = scaler
        self._mcu = mcu
        self._oid = oid = self._mcu.create_oid()
        self._mcu.register_config_callback(self._build_config) #MCU configuration callback (executed during connection)
        self._mcu_position_offset = 0.
        self._reset_cmd_tag = self._get_position_cmd = self._set_spi_cmd = None
        self._active_callbacks = []
        # get C helpers
        ffi_main, ffi_lib = chelper.get_ffi()
        # allocate and register (in the MCU module) the private compressor object
        self._stepqueue = ffi_main.gc(ffi_lib.serialservo_compress_alloc(oid),
                                      ffi_lib.serialservo_compress_free)
        self._mcu.register_stepqueue(self._stepqueue)
        self._stepper_kinematics = None
        self._itersolve_generate_steps = ffi_lib.serialservo_solve_generate_steps #function to sample and generate move steps
        self._itersolve_check_active = ffi_lib.serialservo_solve_check_active #function to check for real displacement
        self._trapq = ffi_main.NULL
        # register connection callback (query position during printer initialization)
        self._mcu.get_printer().register_event_handler('klippy:connect', self._query_mcu_position)

    def get_mcu(self):
        '''
        Get associated mcu.
        '''
        return self._mcu
    
    def get_name(self, short=False):
        '''
        Get serialservo name.
        NOTE: z axis drive will be identified in the configuration file
              using the "serialservo_" prefix.
        '''
        prefix = 'serialservo_'
        if short and self._name.startswith(prefix):
            return self._name[len(prefix):]
        return self._name
    
    def units_in_radians(self):
        '''
        Returns true if distances are in radians instead of millimeters.
        '''
        return self._units_in_radians
        
    def setup_itersolve(self, alloc_func, *params):
        '''
        Setup custom solver algorithm combining trapezoidal queue sampling
        and forward kinematics simulation, this is done after initialization
        and according to the specific printer kinematics.
        '''
        ffi_main, ffi_lib = chelper.get_ffi()
        # use kinematics specific allocation function
        sk = ffi_main.gc(getattr(ffi_lib, alloc_func)(*params), ffi_lib.free)
        # set servo kinematics
        self.set_stepper_kinematics(sk)
    
    def _build_config(self):
        '''
        Function used to inform MCU of serialservo.
        '''
        logging.info("===> MCU frequency = %s" % self._mcu._mcu_freq)
        self._mcu.add_config_cmd("config_serialservo oid=%d sampling_time=%u" % 
            (self._oid, self._simtime*self._mcu._mcu_freq))
        self._mcu.add_config_cmd("serialservo_reset_step_clock oid=%d clock=0"
                                 % (self._oid,), on_restart=True)
        step_cmd_tag = self._mcu.lookup_command(
            "serialservo_queue_step oid=%c target_position=%i target_velocity=%i time=%u").get_command_tag()
        self._reset_cmd_tag = self._mcu.lookup_command(
            "serialservo_reset_step_clock oid=%c clock=%u").get_command_tag()
        self._get_position_cmd = self._mcu.lookup_query_command(
            "serialservo_get_position oid=%c",
            "serialservo_position oid=%c pos=%i", oid=self._oid)
        self._set_spi_cmd = self._mcu.lookup_command(
            "config_serialservo_spi oid=%c spi_oid=%c chain_len=%c chain_pos=%c")
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.serialservo_compress_fill(self._stepqueue, step_cmd_tag, self._pole_pairs, self._scaler)
        
    def get_step_dist(self):
        '''
        Get step distance. NOTE: this is a dummy function needed for
        compatibility reasons (homing, ...).
        '''
        return self._step_dist
    
    def get_oid(self):
        '''
        Get drive id, it is used to ientify the correspondent compressor
        object in the simulation and command scheduling procedures.
        '''
        return self._oid
        
    def calc_position_from_coord(self, coord):
        '''
        Calculate toolhead position from coordinates.
        '''
        ffi_main, ffi_lib = chelper.get_ffi()
        return ffi_lib.serialservo_solve_calc_position_from_coord(self._stepper_kinematics, coord[0], coord[1], coord[2])
        
    def set_position(self, coord):
        '''
        Set toolhead position.
        '''
        mcu_pos = self.get_mcu_position()
        sk = self._stepper_kinematics
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.serialservo_solve_set_position(sk, coord[0], coord[1], coord[2])
        self._set_mcu_position(mcu_pos)
        
    def get_commanded_position(self):
        '''
        Get drive commanded position (expected position at flush time).
        '''
        ffi_main, ffi_lib = chelper.get_ffi()
        return ffi_lib.serialservo_solve_get_commanded_pos(self._stepper_kinematics)
    
    def get_mcu_position(self):
        '''
        Get drive position in [mm].
        '''
        return self.get_commanded_position() + self._mcu_position_offset
    
    def _set_mcu_position(self, mcu_pos):
        '''
        Set drive position in [mm].
        '''
        self._mcu_position_offset = mcu_pos - self.get_commanded_position()
        
    def get_past_mcu_position(self, print_time):
        '''
        Get a past position given a time.
        '''
        clock = self._mcu.print_time_to_clock(print_time)
        ffi_main, ffi_lib = chelper.get_ffi()
        pos = ffi_lib.serialservo_compress_find_past_position(self._stepqueue, clock)
        return pos
    
    def mcu_to_commanded_position(self, mcu_pos):
        '''
        Return commanded position.
        '''
        return mcu_pos - self._mcu_position_offset
    
    def dump_steps(self, count, start_clock, end_clock):
        '''
        Rebuild history from compressed data. 
        '''
        ffi_main, ffi_lib = chelper.get_ffi()
        data = ffi_main.new('struct pull_history_serialservo_steps[]', count)
        count = ffi_lib.serialservo_compress_extract_old(self._stepqueue, data, count, start_clock, end_clock)
        return (data, count)
    
    def get_stepper_kinematics(self):
        '''
        Get drive kinematics.
        '''
        return self._stepper_kinematics
    
    def set_stepper_kinematics(self, sk):
        '''
        Set drive kinematics and helper functions.
        '''
        # get old kinematics
        old_sk = self._stepper_kinematics
        # initial position (consistent with kinematics change)
        mcu_pos = 0
        if old_sk is not None:
            mcu_pos = self.get_mcu_position()
        self._stepper_kinematics = sk
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.serialservo_solve_set_stepcompress(sk, self._stepqueue, self._simtime)
        self.set_trapq(self._trapq)
        self._set_mcu_position(mcu_pos) #move back to previous position
        return old_sk
    
    def note_homing_end(self):
        '''
        Note homing end.
        '''
        # reset the internal state of the ethercatservo_compress object
        ffi_main, ffi_lib = chelper.get_ffi()
        ret = ffi_lib.serialservo_compress_reset(self._stepqueue, 0)
        if ret:
            raise error("Internal error in stepcompress")
        data = (self._reset_cmd_tag, self._oid, 0)
        ret = ffi_lib.serialservo_compress_queue_msg(self._stepqueue, data, len(data))
        if ret:
            raise error("Internal error in stepcompress")
        self._query_mcu_position()
        
    def _query_mcu_position(self):
        '''
        Query mcu position.
        '''
        if self._mcu.is_fileoutput():
            return
        # send request
        params = self._get_position_cmd.send([self._oid])
        last_pos = params['pos']
        # get time of when the command was received by the drive
        print_time = self._mcu.estimated_print_time(params['#receive_time'])
        clock = self._mcu.print_time_to_clock(print_time)
        ffi_main, ffi_lib = chelper.get_ffi()
        # update last drive position
        last_pos = ffi_lib.serialservo_compress_set_last_position(self._stepqueue, clock, last_pos)
        self._set_mcu_position(last_pos)
        '''
        Send drive synch event for angle and extruder module.
        '''
        self._mcu.get_printer().send_event("stepper:sync_mcu_position", self)
        
    def get_trapq(self):
        '''
        Get trapezoidal queue.
        '''
        return self._trapq
    
    def set_trapq(self, tq):
        '''
        Set trapezoidal queue.
        '''
        ffi_main, ffi_lib = chelper.get_ffi()
        if tq is None:
            tq = ffi_main.NULL
        ffi_lib.serialservo_solve_set_trapq(self._stepper_kinematics, tq)
        old_tq = self._trapq
        self._trapq = tq
        return old_tq
    
    def add_active_callback(self, cb):
        '''
        Add active callback, used only for enable pins (unused for drives).
        '''
        self._active_callbacks.append(cb)
        
    def generate_steps(self, flush_time):
        '''
        Generate steps out of the trapezoidal profile up to flush_time.
        If the sampling time used for step generation is too big, the
        sampled queue and the trapezoidal queue are the same.
        '''
        # check for activity if necessary
        if self._active_callbacks:
            '''
            Execute all pending/active callbacks
            '''
            sk = self._stepper_kinematics
            '''
            Check if drive is likely to be active between the current time and flush_time,
            i.e. there is at least a move that produces a real displacement.
            '''
            ret = self._itersolve_check_active(sk, flush_time)
            if ret:
                # execute activity callbacks
                cbs = self._active_callbacks
                self._active_callbacks = []
                for cb in cbs:
                    cb(ret)
        # generate pvt steps
        sk = self._stepper_kinematics
        ret = self._itersolve_generate_steps(sk, flush_time)
        if ret:
            raise error("Internal error in ethercatservo_compress")
        
    def is_active_axis(self, axis):
        '''
        Check if the specified axis is active.
        '''
        ffi_main, ffi_lib = chelper.get_ffi()
        a = axis.encode()
        return ffi_lib.serialservo_solve_is_active_axis(self._stepper_kinematics, a)

    def _handle_debug_response(self, params):
        oid = params['oid']
        position = params['position']
        velocity = params['velocity']
        time = params['time']
        logging.info("-> serialservo step: p=%s, v=%s, t=%s" % 
                     (position, velocity, time))

def PrinterStepper(config, units_in_radians=False):
    '''
    Helper code to build a stepper object from a config section.
    '''
    printer = config.get_printer()
    name = config.get_name()
    simtime = config.getfloat('sampling_time', 0.01, minval=0.001, maxval=1)
    pole_pairs = config.getint('pole_pairs', 1, minval=1, maxval=100)
    scaler = config.getint('scaler', 1, minval=1, maxval=1e6)
    '''
    NOTE: the proper way to get it is through the pin module which is
    guaranteed to be instantiated, however the mcu should be always
    instantiated before the stepper modules are loaded (no sense to
    add a virtual pin to mask this).
    '''
    mcu = printer.lookup_object('mcu')
    # create drive object
    mcu_servo = SerialServo(name=name, mcu=mcu, units_in_radians=False,
                            simtime=simtime, pole_pairs=pole_pairs, scaler=scaler)
    '''
    Register helper modules.
    TODO check which are really needed for servos and adapt them
         to new serialservo module.
    '''
    for mname in ['stepper_enable', 'force_move', 'motion_report']:
        m = printer.load_object(config, mname)
        m.register_stepper(config, mcu_servo)

    return mcu_servo


class PrinterRail:
    '''
    A motor control rail with one (or more) steppers and one (or more)
    endstops with support for pvt drive.
    '''
    def __init__(self, config, need_position_minmax=True,
                 default_position_endstop=None, units_in_radians=False):
        # primary stepper and endstop
        self.stepper_units_in_radians = units_in_radians
        self.steppers = []
        self.endstops = []
        self.endstop_map = {}
        self.add_extra_stepper(config)
        mcu_stepper = self.steppers[0]
        self.get_name = mcu_stepper.get_name
        self.get_commanded_position = mcu_stepper.get_commanded_position
        self.calc_position_from_coord = mcu_stepper.calc_position_from_coord
        # primary endstop position
        mcu_endstop = self.endstops[0][0]
        if hasattr(mcu_endstop, "get_position_endstop"):
            self.position_endstop = mcu_endstop.get_position_endstop()
        elif default_position_endstop is None:
            self.position_endstop = config.getfloat('position_endstop')
        else:
            self.position_endstop = config.getfloat('position_endstop', default_position_endstop)
        # axis range
        if need_position_minmax:
            self.position_min = config.getfloat('position_min', 0.)
            self.position_max = config.getfloat('position_max', above=self.position_min)
        else:
            self.position_min = 0.
            self.position_max = self.position_endstop
        if (self.position_endstop < self.position_min or self.position_endstop > self.position_max):
            raise config.error(
                "position_endstop in section '%s' must be between"
                " position_min and position_max" % config.get_name())
        # homing mechanics
        self.homing_speed = config.getfloat('homing_speed', 500.0, above=0.)
        self.second_homing_speed = config.getfloat('second_homing_speed', self.homing_speed/2., above=0.)
        self.homing_retract_speed = config.getfloat('homing_retract_speed', self.homing_speed, above=0.)
        self.homing_retract_dist = config.getfloat('homing_retract_dist', 5., minval=0.)
        self.homing_positive_dir = config.getboolean( 'homing_positive_dir', None)
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
        '''
        Get rail position range.
        '''
        return self.position_min, self.position_max
    
    def get_homing_info(self):
        '''
        Get homing information.
        '''
        homing_info = collections.namedtuple('homing_info', [
            'speed', 'position_endstop', 'retract_speed', 'retract_dist',
            'positive_dir', 'second_homing_speed'])(
                self.homing_speed, self.position_endstop,
                self.homing_retract_speed, self.homing_retract_dist,
                self.homing_positive_dir, self.second_homing_speed)
        return homing_info
    
    def get_steppers(self):
        '''
        Get rail steppers (drives).
        '''
        return list(self.steppers)
    
    def get_endstops(self):
        '''
        Get rail endstops.
        '''
        return list(self.endstops)
    
    def add_extra_stepper(self, config):
        '''
        Add stepper (drive) to the current rail.
        '''
        stepper = PrinterStepper(config, self.stepper_units_in_radians)
        self.steppers.append(stepper)
        if self.endstops and config.get('endstop_pin', None) is None:
            # no endstop defined, use primary endstop
            self.endstops[0][0].add_stepper(stepper)
            return
        endstop_pin = config.get('endstop_pin')
        printer = config.get_printer()
        ppins = printer.lookup_object('pins')
        pin_params = ppins.parse_pin(endstop_pin, True, True)
        # normalize pin name
        pin_name = "%s:%s" % (pin_params['chip_name'], pin_params['pin'])
        # look for already registered endstop
        endstop = self.endstop_map.get(pin_name, None)
        if endstop is None:
            # new endstop, register it
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
                raise error("Pinter rail %s shared endstop pin %s "
                            "must specify the same pullup/invert settings" % (
                                self.get_name(), pin_name))
        mcu_endstop.add_stepper(stepper)
        
    def setup_itersolve(self, alloc_func, *params):
        '''
        Setup solver for each associated stepper.
        '''
        for stepper in self.steppers:
            stepper.setup_itersolve(alloc_func, *params)
            
    def generate_steps(self, flush_time):
        '''
        Generate steps for each associated stepper.
        '''
        for stepper in self.steppers:
            stepper.generate_steps(flush_time)
            
    def set_trapq(self, trapq):
        '''
        Setup trapezoidal queue for each associated stepper.
        '''
        for stepper in self.steppers:
            stepper.set_trapq(trapq)
            
    def set_position(self, coord):
        '''
        Set position for each associated stepper.
        '''
        for stepper in self.steppers:
            stepper.set_position(coord)

def LookupMultiRail(config, need_position_minmax=True,
                    default_position_endstop=None, units_in_radians=False):
    '''
    Wrapper for dual stepper motor support.
    '''
    rail = PrinterRail(config, need_position_minmax, default_position_endstop, units_in_radians)
    for i in range(1, 99):
        if not config.has_section(config.get_name() + str(i)):
            break
        rail.add_extra_stepper(config.getsection(config.get_name() + str(i)))
    return rail
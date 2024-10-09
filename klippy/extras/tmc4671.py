#
# TMC4671 configuration with Position Control and Velocity Feed-Forward
#
import math, logging
from . import bus, tmc

# frequency specific for TMC4671
TMC_FREQUENCY = 25000000.

# TMC4671 registers
Registers = {
    "CHIPINFO_DATA": 0x00,
    "CHIPINFO_ADDR": 0x01,
    "MOTOR_TYPE_N_POLE_PAIRS": 0x1B,
    "PID_TORQUE_P_TORQUE_I": 0x56,
    "PID_VELOCITY_P_VELOCITY_I": 0x58,
    "PID_POSITION_P_POSITION_I": 0x5A,
    "MODE_RAMP_MODE_MOTION": 0x63,
    "PID_TORQUE_FLUX_TARGET": 0x64,
    "PID_VELOCITY_TARGET": 0x66,
    "PID_POSITION_TARGET": 0x68,
    "PID_VELOCITY_ACTUAL": 0x6A,
    "PID_POSITION_ACTUAL": 0x6B,
    "STATUS_FLAGS": 0x7C,
    "STATUS_MASK": 0x7D,
}

# read subset registers
ReadRegisters = [
    "PID_POSITION_ACTUAL", "PID_VELOCITY_ACTUAL",
    "PID_VELOCITY_TARGET", "PID_POSITION_TARGET"
]

# register fields
Fields = {}
Fields["PID_POSITION_P_POSITION_I"] = {
    "kp_position": 0xffff << 16,
    "ki_position": 0xffff << 0,
}
Fields["PID_VELOCITY_P_VELOCITY_I"] = {
    "kp_velocity": 0xffff << 16,
    "ki_velocity": 0xffff << 0,
}
Fields["PID_TORQUE_P_TORQUE_I"] = {
    "kp_torque": 0xffff << 16,
    "ki_torque": 0xffff << 0,
}
Fields["MOTOR_TYPE_N_POLE_PAIRS"] = {
    "motor_type": 0xffff << 16,
    "pole_pairs": 0xffff << 0,
}
Fields["MODE_RAMP_MODE_MOTION"] = {
    "mode_pid_type": 0x01 << 31,
    "mode_pid_smpl": 0x7F << 24,
    "mode_motion": 0xFF << 0,
}

Fields["STATUS_FLAGS"] = {
    "pid_x_target_limit": 0x01 << 0,
    "pid_x_errsum_limit": 0x01 << 2,
    "pid_x_output_limit": 0x01 << 3,
    "pid_v_target_limit": 0x01 << 4,
    "pid_v_errsum_limit": 0x01 << 6,
    "pid_v_output_limit": 0x01 << 7,
    "pid_id_target_limit": 0x01 << 8,
    "pid_id_errsum_limit": 0x01 << 10,
    "pid_id_output_limit": 0x01 << 11,
    "pid_iq_target_limit": 0x01 << 12,
    "pid_iq_errsum_limit": 0x01 << 14,
    "pid_iq_output_limit": 0x01 << 15,
    "ipark_cirlim_limit_u_d": 0x01 << 16,
    "ipark_cirlim_limit_u_q": 0x01 << 17,
    "ipark_cirlim_limit_u_r": 0x01 << 18,
    "ref_sw_r": 0x01 << 20,
    "ref_sw_h": 0x01 << 21,
    "ref_sw_l": 0x01 << 22,
    "pwm_min": 0x01 << 24,
    "pwm_max": 0x01 << 25,
    "adc_i_clipped": 0x01 << 26,
    "aenc_clipped": 0x01 << 27,
    "enc_n": 0x01 << 28,
    "enc_2_n": 0x01 << 29,
    "aenc_n": 0x01 << 30,   
}

Fields["STATUS_MASK"] = {
    "status_mask": 0xffffffff
}

SignedFields = ["position_target", "velocity_target", "torque_target"]

FieldFormatters = {
    "position_target": (lambda v: "Position Target: %d" % v),
    "velocity_target": (lambda v: "Velocity Target: %d" % v),
    "torque_target": (lambda v: "Torque Target: %d" % v),
}


class TMCCurrentHelper:
    '''
    TMC current helper. NOTE: it is a separate class just in case
    more tmc servo models will be added in the future, otherwise
    it can be merged into TMC4671 class.
    '''
    def __init__(self, config, mcu_tmc):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()
        run_current = config.getfloat('run_current', above=0.)
        hold_current = config.getfloat('hold_current', run_current, above=0.)

    def set_current(self, run_current, hold_current, print_time=None):
        torque_target = self._calc_torque(run_current)
        self.mcu_tmc.set_register("PID_TORQUE_FLUX_TARGET", torque_target, print_time)

    def _calc_torque(self, current):
        torque = int(current * 1000)
        return max(0, min(0xffff, torque))

    def get_current(self):
        torque_target = self.mcu_tmc.get_register("PID_TORQUE_FLUX_TARGET")
        return self._calc_current_from_torque(torque_target)

    def _calc_current_from_torque(self, torque):
        current = torque / 1000
        return current
    

class TMCCommandHelper:
    '''
    TMC command helper. NOTE: it is a separate class just in case
    more tmc servo models will be added in the future, otherwise
    it can be merged into TMC4671 class.
    '''
    def __init__(self, config, mcu_tmc, current_helper):
        self.printer = config.get_printer()
        self.stepper_name = ' '.join(config.get_name().split()[1:])
        self.name = config.get_name().split()[-1]
        self.mcu_tmc = mcu_tmc
        self.current_helper = current_helper
        #self.echeck_helper = TMCErrorCheck(config, mcu_tmc)
        self.fields = mcu_tmc.get_fields()
        self.read_registers = None
        self.read_translate = None
        self.toff = None
        self.mcu_phase_offset = None
        self.stepper = None
        self.spi = mcu_tmc.tmc_spi.spi
        self.mcu = self.spi.get_mcu()
        self.stepper_enable = self.printer.load_object(config, "stepper_enable")
        self.printer.register_event_handler("stepper:sync_mcu_position", self._handle_sync_mcu_pos)
        self.printer.register_event_handler("klippy:mcu_identify", self._handle_mcu_identify)
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        # register commands (NOTE: keep stepper notation for compatibility)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("SET_TMC4671_FIELD", "STEPPER", self.name, self.cmd_SET_TMC_FIELD, desc=self.cmd_SET_TMC_FIELD_help)
        gcode.register_mux_command("INIT_TMC4671", "STEPPER", self.name, self.cmd_INIT_TMC, desc=self.cmd_INIT_TMC_help)
        gcode.register_mux_command("SET_TMC4671_CURRENT", "STEPPER", self.name, self.cmd_SET_TMC_CURRENT, desc=self.cmd_SET_TMC_CURRENT_help)

    def _init_registers(self, print_time=None):
        '''
        Setup registers.
        '''
        for reg_name in list(self.fields.registers.keys()):
            val = self.fields.registers[reg_name] # Val may change during loop
            pre_val = self.mcu_tmc.get_register(reg_name)
            self.mcu_tmc.set_register(reg_name, val, print_time)
            post_val = self.mcu_tmc.get_register(reg_name)
            logging.info("%s: (pre = %s, post = %s)" % (reg_name, pre_val, post_val))
            
    cmd_INIT_TMC_help = "Initialize TMC4671 stepper driver registers"
    def cmd_INIT_TMC(self, gcmd):
        '''
        Initialize TMC.
        '''
        logging.info("INIT_TMC for TMC4671 %s", self.name)
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        self._init_registers(print_time)

    cmd_SET_TMC_FIELD_help = "Set a register field of the TMC4671 driver"
    def cmd_SET_TMC_FIELD(self, gcmd):
        '''
        Set TMC field.
        '''
        field_name = gcmd.get('FIELD').lower()
        reg_name = self.fields.lookup_register(field_name, None)
        if reg_name is None:
            raise gcmd.error("Unknown field name '%s'" % (field_name,))
        value = gcmd.get_int('VALUE', None)
        if value is None:
            raise gcmd.error("Null value")
        reg_val = self.fields.set_field(field_name, value)
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        self.mcu_tmc.set_register(reg_name, reg_val, print_time)

    cmd_SET_TMC_CURRENT_help = "Set the current of a TMC4671 driver"
    def cmd_SET_TMC_CURRENT(self, gcmd):
        pass

    def _handle_sync_mcu_pos(self, stepper):
        '''
        Synchronize mcu position. TODO: add logic to handle
        position instead of phase.
        '''
        if stepper.get_name() != self.name:
            return
        pass

    def _handle_mcu_identify(self):
        '''
        Identify mcu and perform post initialization tasks.
        '''
        force_move = self.printer.lookup_object("force_move")
        self.stepper = force_move.lookup_stepper(self.stepper_name)
        
    def _handle_stepper_enable(self, print_time, is_enable):
        '''
        Enable or disable stepper.
        '''
        if is_enable:
            cb = (lambda ev: self._do_enable(print_time))
        else:
            cb = (lambda ev: self._do_disable(print_time))
        self.printer.get_reactor().register_callback(cb)

    def _do_enable(self, print_time):
        '''
        Enable stepper.
        '''
        try:
            self._init_registers(print_time)
            logging.info(f"TMC4671 {self.name}: Enabled")
        except self.printer.command_error as e:
            logging.error(f"Error enabling {self.name}: {str(e)}")
            self.printer.invoke_shutdown(str(e))

    def _do_disable(self, print_time):
        '''
        Disable stepper.
        '''
        try:
            logging.info(f"TMC4671 {self.name}: Disabled")
        except self.printer.command_error as e:
            logging.error(f"Error disabling {self.name}: {str(e)}")
            self.printer.invoke_shutdown(str(e))

    def _handle_connect(self):
        '''
        Connect to mcu and perform postinitialization tasks.
        '''
        try:
            self._init_registers()
            # enable serialservo spi in firmware directly
            self.stepper._set_spi_cmd.send(
                [self.stepper.get_oid(), self.spi.get_oid(),
                 self.mcu_tmc.tmc_spi.chain_len,
                 self.mcu_tmc.chain_pos])
            logging.info(f"TMC4671 {self.name}: connected and initialized")
            status_flags = self.mcu_tmc.get_register("STATUS_FLAGS")
            logging.info("STATUS REGISTER = %s" % status_flags)
        except self.printer.command_error as e:
            logging.error(f"TMC4671 {self.name} failed to initialize: {str(e)}")

    def setup_register_dump(self, read_registers, read_translate=None):
        '''
        Dump register.
        '''
        self.read_registers = read_registers
        self.read_translate = read_translate
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("DUMP_TMCS", "STEPPER", self.name, self.cmd_DUMP_TMC, desc=self.cmd_DUMP_TMC_help)

    cmd_DUMP_TMC_help = "Read and display TMC4671 servo driver registers"
    def cmd_DUMP_TMC(self, gcmd):
        '''
        Dump tmc command.
        '''
        logging.info("DUMP_TMC %s", self.name)
        reg_name = gcmd.get('REGISTER', None)
        if reg_name is not None:
            reg_name = reg_name.upper()
            val = self.fields.registers.get(reg_name)
            if (val is not None) and (reg_name not in self.read_registers):
                # write-only register
                gcmd.respond_info(self.fields.pretty_format(reg_name, val))
            elif reg_name in self.read_registers:
                # readable register
                val = self.mcu_tmc.get_register(reg_name)
                if self.read_translate is not None:
                    reg_name, val = self.read_translate(reg_name, val)
                gcmd.respond_info(self.fields.pretty_format(reg_name, val))
            else:
                raise gcmd.error("Unknown register name '%s'" % (reg_name))
        else:
            gcmd.respond_info("========== Write-only registers ==========")
            for reg_name, val in self.fields.registers.items():
                if reg_name not in self.read_registers:
                    gcmd.respond_info(self.fields.pretty_format(reg_name, val))
            gcmd.respond_info("========== Queried registers ==========")
            for reg_name in self.read_registers:
                val = self.mcu_tmc.get_register(reg_name)
                if self.read_translate is not None:
                    reg_name, val = self.read_translate(reg_name, val)
                gcmd.respond_info(self.fields.pretty_format(reg_name, val))


class MCU_TMC_SPI_chain:
    '''
    Spi chain helper.
    '''
    def __init__(self, config, chain_len=1):
        self.printer = config.get_printer()
        self.chain_len = chain_len
        self.mutex = self.printer.get_reactor().mutex()
        share = None
        if chain_len > 1:
            share = "tmc_spi_cs"
        self.spi = bus.MCU_SPI_from_config(config, 3, default_speed=4000000,
                                           share_type=share)
        self.taken_chain_positions = []

    def _build_cmd(self, data, chain_pos):
        '''
        Build command.
        '''
        return ([0x00] * ((self.chain_len - chain_pos) * 5) +
                data + [0x00] * ((chain_pos - 1) * 5))
    
    def reg_read(self, reg, chain_pos):
        '''
        Read register.
        '''
        cmd = self._build_cmd([reg, 0x00, 0x00, 0x00, 0x00], chain_pos)
        self.spi.spi_send(cmd)
        if self.printer.get_start_args().get('debugoutput') is not None:
            return 0
        params = self.spi.spi_transfer(cmd)
        pr = bytearray(params['response'])
        pr = pr[(self.chain_len - chain_pos) * 5 :
                (self.chain_len - chain_pos + 1) * 5]
        return (pr[1] << 24) | (pr[2] << 16) | (pr[3] << 8) | pr[4]
    
    def reg_write(self, reg, val, chain_pos, print_time=None):
        '''
        Write register.
        '''
        minclock = 0
        if print_time is not None:
            minclock = self.spi.get_mcu().print_time_to_clock(print_time)
        data = [(reg | 0x80) & 0xff, (val >> 24) & 0xff, (val >> 16) & 0xff,
                (val >> 8) & 0xff, val & 0xff]
        if self.printer.get_start_args().get('debugoutput') is not None:
            self.spi.spi_send(self._build_cmd(data, chain_pos), minclock)
            return val
        write_cmd = self._build_cmd(data, chain_pos)
        dummy_read = self._build_cmd([0x00, 0x00, 0x00, 0x00, 0x00], chain_pos)
        params = self.spi.spi_transfer_with_preface(write_cmd, dummy_read,
                                                    minclock=minclock)
        pr = bytearray(params['response'])
        pr = pr[(self.chain_len - chain_pos) * 5 :
                (self.chain_len - chain_pos + 1) * 5]
        return (pr[1] << 24) | (pr[2] << 16) | (pr[3] << 8) | pr[4]

def lookup_tmc_spi_chain(config):
    '''
    Helper to setup an spi daisy chain bus from settings in a config section
    '''
    chain_len = config.getint('chain_length', None, minval=2)
    if chain_len is None:
        # simple, non daisy chained SPI connection
        return MCU_TMC_SPI_chain(config, 1), 1

    # shared SPI bus - lookup existing MCU_TMC_SPI_chain
    ppins = config.get_printer().lookup_object("pins")
    cs_pin_params = ppins.lookup_pin(config.get('cs_pin'),
                                     share_type="tmc_spi_cs")
    tmc_spi = cs_pin_params.get('class')
    if tmc_spi is None:
        tmc_spi = cs_pin_params['class'] = MCU_TMC_SPI_chain(config, chain_len)
    if chain_len != tmc_spi.chain_len:
        raise config.error("TMC SPI chain must have same length")
    chain_pos = config.getint('chain_position', minval=1, maxval=chain_len)
    if chain_pos in tmc_spi.taken_chain_positions:
        raise config.error("TMC SPI chain can not have duplicate position")
    tmc_spi.taken_chain_positions.append(chain_pos)
    return tmc_spi, chain_pos


class MCU_TMC_SPI:
    '''
    Helper code for working with TMC devices via SPI.
    '''
    def __init__(self, config, name_to_reg, fields, tmc_frequency):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.tmc_spi, self.chain_pos = lookup_tmc_spi_chain(config)
        self.mutex = self.tmc_spi.mutex
        self.name_to_reg = name_to_reg
        self.fields = fields
        self.tmc_frequency = tmc_frequency

    def get_fields(self):
        '''
        Field getter.
        '''
        return self.fields
    
    def get_register(self, reg_name):
        '''
        Get register from name.
        '''
        reg = self.name_to_reg[reg_name]
        with self.mutex:
            read = self.tmc_spi.reg_read(reg, self.chain_pos)
        return read
    
    def set_register(self, reg_name, val, print_time=None):
        '''
        Set register from name.
        '''
        reg = self.name_to_reg[reg_name]
        with self.mutex:
            self.tmc_spi.reg_write(reg, val, self.chain_pos, print_time)

    def get_tmc_frequency(self):
        '''
        Frequency getter.
        '''
        return self.tmc_frequency


class TMC4671:
    '''
    TMC4671 driver. NOTE: keep this class as simple as possible for generalization,
    so that it will be easier to add different tmc servo models.
    '''
    def __init__(self, config):
        # field parser
        self.fields = tmc.FieldHelper(Fields, SignedFields, FieldFormatters)
        # create spi bus
        self.mcu_tmc = MCU_TMC_SPI(config, Registers, self.fields, TMC_FREQUENCY)
        # virtual pin for sensorless homing
        tmc.TMCVirtualPinHelper(config, self.mcu_tmc)
        # generic current helper
        current_helper = TMCCurrentHelper(config, self.mcu_tmc)
        # generic command helper
        cmdhelper = TMCCommandHelper(config, self.mcu_tmc, current_helper)
        # setup registers
        self.fields.set_config_field(config, "motor_type", config.getint('motor_type', 3, minval=1))
        self.fields.set_config_field(config, "pole_pairs", config.getint('pole_pairs', 1, minval=1)) #7
        self.fields.set_config_field(config, "kp_position", config.getint('kp_position', 4000)) #3000
        self.fields.set_config_field(config, "ki_position", config.getint('ki_position', 200)) #100
        self.fields.set_config_field(config, "kp_velocity", config.getint('kp_velocity', 5000)) #4000
        self.fields.set_config_field(config, "ki_velocity", config.getint('ki_velocity', 200))
        self.fields.set_config_field(config, "kp_torque", config.getint('kp_torque', 4000))
        self.fields.set_config_field(config, "ki_torque", config.getint('ki_torque', 300))
        self.fields.set_config_field(config, "mode_motion", config.getint('mode_motion', 3)) #position mode
        self.fields.set_config_field(config, "status_mask", 0xffffffff)

def load_config_prefix(config):
    '''
    Load TMC4671 from config file.
    '''
    return TMC4671(config)
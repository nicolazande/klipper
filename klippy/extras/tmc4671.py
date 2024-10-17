#
# TMC4671 configuration with Position Control and Velocity Feed-Forward
#
import math, logging
from . import bus, tmc

# frequency specific for TMC4671
TMC_FREQUENCY = 25000000.

# TMC4671 registers
Registers = {
    "CHIPINFO_DATA" : 0x00,
    "CHIPINFO_ADDR" : 0x01,
    "ADC_RAW_DATA" : 0x02,
    "ADC_RAW_ADDR" : 0x03,
    "dsADC_MCFG_B_MCFG_A" : 0x04,
    "dsADC_MCLK_A" : 0x05,
    "dsADC_MCLK_B" : 0x06,
    "dsADC_MDEC_B_MDEC_A" : 0x07,
    "ADC_I1_SCALE_OFFSET" : 0x08,
    "ADC_I0_SCALE_OFFSET" : 0x09,
    "ADC_I_SELECT" : 0x0A,
    "ADC_I1_I0_EXT" : 0x0B,
    "DS_ANALOG_INPUT_STAGE_CFG" : 0x0C,
    "AENC_0_SCALE_OFFSET" : 0x0D,
    "AENC_1_SCALE_OFFSET" : 0x0E,
    "AENC_2_SCALE_OFFSET" : 0x0F,
    "AENC_SELECT" : 0x11,
    "ADC_IWY_IUX" : 0x12,
    "ADC_IV" : 0x13,
    "AENC_WY_UX" : 0x15,
    "AENC_VN" : 0x16,
    "PWM_POLARITIES" : 0x17,
    "PWM_MAXCNT" : 0x18,
    "PWM_BBM_H_BBM_L" : 0x19,
    "PWM_SV_CHOP" : 0x1A,
    "MOTOR_TYPE_N_POLE_PAIRS" : 0x1B,
    "PHI_E_EXT" : 0x1C,
    "OPENLOOP_MODE" : 0x1F,
    "OPENLOOP_ACCELERATION" : 0x20,
    "OPENLOOP_VELOCITY_TARGET" : 0x21,
    "OPENLOOP_VELOCITY_ACTUAL" : 0x22,
    "UQ_UD_EXT" : 0x24,
    "ABN_DECODER_MODE" : 0x25,
    "ABN_DECODER_PPR" : 0x26,
    "ABN_DECODER_COUNT" : 0x27,
    "ABN_DECODER_COUNT_N" : 0x28,
    "ABN_DECODER_PHI_E_PHI_M_OFFSET" : 0x29,
    "ABN_DECODER_PHI_E_PHI_M" : 0x2A,
    "ABN_2_DECODER_MODE" : 0x2C,
    "ABN_2_DECODER_PPR" : 0x2D,
    "ABN_2_DECODER_COUNT" : 0x2E,
    "ABN_2_DECODER_COUNT_N" : 0x2F,
    "ABN_2_DECODER_PHI_M_OFFSET" : 0x30,
    "ABN_2_DECODER_PHI_M" : 0x31,
    "HALL_MODE" : 0x33,
    "HALL_POSITION_060_000" : 0x34,
    "HALL_POSITION_180_120" : 0x35,
    "HALL_POSITION_300_240" : 0x36,
    "HALL_PHI_E_PHI_M_OFFSET" : 0x37,
    "HALL_DPHI_MAX" : 0x38,
    "HALL_PHI_E_INTERPOLATED_PHI_E" : 0x39,
    "HALL_PHI_M" : 0x3A,
    "AENC_DECODER_MODE" : 0x3B,
    "AENC_DECODER_N_THRESHOLD" : 0x3C,
    "AENC_DECODER_PHI_A_RAW" : 0x3D,
    "AENC_DECODER_PHI_A_OFFSET" : 0x3E,
    "AENC_DECODER_PHI_A" : 0x3F,
    "AENC_DECODER_PPR" : 0x40,
    "AENC_DECODER_COUNT" : 0x41,
    "AENC_DECODER_COUNT_N" : 0x42,
    "AENC_DECODER_PHI_E_PHI_M_OFFSET" : 0x45,
    "AENC_DECODER_PHI_E_PHI_M" : 0x46,
    "CONFIG_DATA" : 0x4D,
    "CONFIG_ADDR" : 0x4E,
    "VELOCITY_SELECTION" : 0x50,
    "POSITION_SELECTION" : 0x51,
    "PHI_E_SELECTION" : 0x52,
    "PHI_E" : 0x53,
    "PID_FLUX_P_FLUX_I" : 0x54,
    "PID_TORQUE_P_TORQUE_I" : 0x56,
    "PID_VELOCITY_P_VELOCITY_I" : 0x58,
    "PID_POSITION_P_POSITION_I" : 0x5A,
    "PIDOUT_UQ_UD_LIMITS" : 0x5D,
    "PID_TORQUE_FLUX_LIMITS" : 0x5E,
    "PID_VELOCITY_LIMIT" : 0x60,
    "PID_POSITION_LIMIT_LOW" : 0x61,
    "PID_POSITION_LIMIT_HIGH" : 0x62,
    "MODE_RAMP_MODE_MOTION" : 0x63,
    "PID_TORQUE_FLUX_TARGET" : 0x64,
    "PID_TORQUE_FLUX_OFFSET" : 0x65,
    "PID_VELOCITY_TARGET" : 0x66,
    "PID_VELOCITY_OFFSET" : 0x67,
    "PID_POSITION_TARGET" : 0x68,
    "PID_TORQUE_FLUX_ACTUAL" : 0x69,
    "PID_VELOCITY_ACTUAL" : 0x6A,
    "PID_POSITION_ACTUAL" : 0x6B,
    "PID_ERROR_DATA" : 0x6C,
    "PID_ERROR_ADDR" : 0x6D,
    "INTERIM_DATA" : 0x6E,
    "INTERIM_ADDR" : 0x6F,
    "ADC_VM_LIMITS" : 0x75,
    "TMC4671_INPUTS_RAW" : 0x76,
    "TMC4671_OUTPUTS_RAW" : 0x77,
    "STEP_WIDTH" : 0x78,
    "UART_BPS" : 0x79,
    "GPIO_dsADCI_CONFIG" : 0x7B,
    "STATUS_FLAGS" : 0x7C,
    "STATUS_MASK" : 0x7D
}

# register fields
Fields = {}

# status flags
Fields["STATUS_FLAGS"] = {
    "status_flags" : 0xffffffff
}
Fields["TMC4671_INPUTS_RAW"] = {
    "tmc4671_inputs_raw" : 0xffffffff << 0
}
Fields["TMC4671_OUTPUTS_RAW"] = {
    "tmc4671_outputs_raw" : 0xffffffff << 0
}

# motor type
Fields["MOTOR_TYPE_N_POLE_PAIRS"] = {
    "pole_pairs": 0xffff << 0,
    "motor_type": 0xffff << 16,
}

#motion mode
Fields["MODE_RAMP_MODE_MOTION"] = {
    "mode_motion": 0xFF << 0,
    "mode_pid_smpl": 0x7F << 24,
    "mode_pid_type": 0x01 << 31,
}

# pwm
Fields["PWM_POLARITIES"] = {
    "low_side_gate" : 0x01 << 0,
    "high_side_gate" : 0x01 << 1,
}
Fields["PWM_MAXCNT"] = {
    "pwm_maxcnt" : 0xfff << 0,
}
Fields["PWM_SV_CHOP"] = {
    "pwm_chomp" : 0xff << 0,
    "pwm_sv" : 0x01 << 8
}
Fields["PWM_BBM_H_BBM_L"] = {
    "pwm_bbm_l" : 0xff << 0,
    "pwm_bbm_h" : 0xff << 8,
}

# adc
Fields["ADC_I_SELECT"] = {
    "adc_i0_select": 0xff << 0,
    "adc_i1_select": 0xff << 8,
    "adc_i_ux_select" : 0x03 << 24,
    "adc_i_v_select" : 0x03 << 26,
    "adc_i_wy_select" : 0x03 << 28,
}
Fields["ADC_I0_SCALE_OFFSET"] = {
    "adc_i0_offset" : 0xffff << 0,
    "adc_i0_scale" : 0xffff << 16,
}
Fields["ADC_I1_SCALE_OFFSET"] = {
    "adc_i1_offset" : 0xffff << 0,
    "adc_i1_scale" : 0xffff << 16,
}
Fields["dsADC_MDEC_B_MDEC_A"] = {
    "dsadc_mdec_a" : 0xffff << 0,
    "dsadc_mdec_b" : 0xffff << 16,
}
Fields["dsADC_MCFG_B_MCFG_A"] = {
    "cfg_dsmodulator_a" : 0x03 << 0,
    "mclk_polarity_a" : 0x01 << 2,
    "mdat_polarity_a" : 0x01 << 3,
    "sel_nclk_mclk_i_a" : 0x01 << 4,
    "cfg_dsmodulator_b" : 0x03 << 16,
    "mclk_polarity_b" : 0x01 << 18,
    "mdat_polarity_b" : 0x01 << 19,
    "sel_nclk_mclk_i_b" : 0x01 << 20,
}
Fields["dsADC_MCLK_A"] = {
    "dsadc_mclk_a" : 0xffffffff << 0,
}
Fields["dsADC_MCLK_B"] = {
    "dsadc_mclk_b": 0xffffffff << 0,
}

#encoder
Fields["ABN_DECODER_MODE"] = {
    "apol" : 0x01 << 0,
    "bpol" : 0x01 << 1,
    "npol" : 0x01 << 2,
    "use_abn_as_n" : 0x01 << 3,
    "cln" : 0x01 << 8,
    "direction" : 0x01 << 12
}
Fields["ABN_DECODER_PPR"] = {
    "abn_decoder_ppr" : 0xffffff << 0,
}
Fields["ABN_DECODER_COUNT"] = {
    "abn_decoder_count" : 0xffffff << 0,
}
Fields["ABN_DECODER_PHI_E_PHI_M_OFFSET"] = {
    "abn_decoder_phi_m_offset" : 0xffff << 0,
    "abn_decoder_phi_e_offset" : 0xffff << 16,
}
Fields["VELOCITY_SELECTION"] = {
    "velocity_selection" : 0xff << 0,
    "velocity_meter_selection" : 0xff << 8
}
Fields["POSITION_SELECTION"] = {
    "position_selection" : 0xff << 0,
}
Fields["PHI_E_SELECTION"] = {
    "phi_e_selection" : 0xff << 0
}
Fields["PHI_E_EXT"] = {
    "phi_e_ext" : 0xffff << 0
}
Fields["UQ_UD_EXT"] = {
    "ud_ext" : 0xffff << 0,
    "uq_ext" : 0xffff << 16
}

#limits
Fields["PID_TORQUE_FLUX_LIMITS"] = {
    "pid_torque_flux_limits" : 0xffff
}
Fields["PID_VELOCITY_LIMIT"] = {
    "pid_velocity_limit" : 0xffff
}
Fields["PID_POSITION_LIMIT_LOW"] = {
    "pid_position_limit_low" : 0xffffffff
}
Fields["PID_POSITION_LIMIT_HIGH"] = {
    "pid_position_limit_high" : 0xffffffff
}

#controller
Fields["PID_POSITION_P_POSITION_I"] = {
    "ki_position": 0xffff << 0,
    "kp_position": 0xffff << 16,
}
Fields["PID_VELOCITY_P_VELOCITY_I"] = {
    "ki_velocity": 0xffff << 0,
    "kp_velocity": 0xffff << 16,
}
Fields["PID_TORQUE_P_TORQUE_I"] = {
    "ki_torque": 0xffff << 0,
    "kp_torque": 0xffff << 16,
}
Fields["PID_FLUX_P_FLUX_I"] = {
    "ki_flux" : 0xffff << 0,
    "kp_flux" : 0xffff << 16
}

Fields["OPENLOOP_MODE"] = {
    "open_loop_mode" : 0xffffffff << 0
}
Fields["OPENLOOP_ACCELERATION"] = {
    "openloop_acceleration" : 0xffffffff << 0
}
Fields["PID_POSITION_ACTUAL"] = {
    "pid_position_angle" : 0xffff << 0,
    "pid_position_revolutions" : 0xffff << 16
}

SignedFields = [
    "position_target",
    "velocity_target",
    "torque_target"]

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
        #STATUS_FLAGS
        self.fields.set_config_field(config, "status_flags", 0)
        #MOTOR_TYPE_N_POLE_PAIRS
        self.fields.set_config_field(config, "pole_pairs", 4) #four poles
        self.fields.set_config_field(config, "motor_type", 3) #three phase motor
        #PWM_POLARITIES
        self.fields.set_config_field(config, "low_side_gate", 0) #standard polarity
        self.fields.set_config_field(config, "high_side_gate", 0) #standard polarity
        #PWM_MAXCNT
        self.fields.set_config_field(config, "pwm_maxcnt", 0xF9F) #pwm frequency (25kHz)
        #PWM_BBM_H_BBM_L
        self.fields.set_config_field(config, "pwm_bbm_l", 0x0A) #low side mosfet dead time (10 ms)
        self.fields.set_config_field(config, "pwm_bbm_h", 0x0A) #high side mosfet dead time (10 ms)
        #PWM_SV_CHOP
        self.fields.set_config_field(config, "pwm_chomp", 0x07) #centered pwm for foc
        self.fields.set_config_field(config, "pwm_sv", 0x00) #enable space vector modulation
        #ADC_I_SELECT
        self.fields.set_config_field(config, "adc_i0_select", 0x00) #adc channel ADCSD_I0_RAW
        self.fields.set_config_field(config, "adc_i1_select", 0x01) #adc channel ADCSD_I1_RAW
        self.fields.set_config_field(config, "adc_i_ux_select", 0x00) #UX = ADC_I0 (default = 0)
        self.fields.set_config_field(config, "adc_i_v_select", 0x01) #UX = ADC_I2 (default = 0)
        self.fields.set_config_field(config, "adc_i_wy_select", 0x02) #WY = ADC_I1 (default = 2)
        #dsADC_MCFG_B_MCFG_A
        self.fields.set_config_field(config, "cfg_dsmodulator_a", 0x00)
        self.fields.set_config_field(config, "mclk_polarity_a", 0x00)
        self.fields.set_config_field(config, "mdat_polarity_a", 0x00)
        self.fields.set_config_field(config, "sel_nclk_mclk_i_a", 0x01)
        self.fields.set_config_field(config, "cfg_dsmodulator_b", 0x00)
        self.fields.set_config_field(config, "mclk_polarity_b", 0x00)
        self.fields.set_config_field(config, "mdat_polarity_b", 0x00)
        self.fields.set_config_field(config, "sel_nclk_mclk_i_b", 0x01)
        #dsADC_MCLK_A
        self.fields.set_config_field(config, "dsadc_mclk_a", 0x20000000)
        #dsADC_MCLK_B
        self.fields.set_config_field(config, "dsadc_mclk_b", 0x00)
        #dsADC_MDEC_B_MDEC_A
        self.fields.set_config_field(config, "dsadc_mdec_a", 0x014E)
        self.fields.set_config_field(config, "dsadc_mdec_b", 0x014E)
        #ADC_I0_SCALE_OFFSET
        self.fields.set_config_field(config, "adc_i0_offset", 0x815B)
        self.fields.set_config_field(config, "adc_i0_scale", 0x0100)
        #ADC_I1_SCALE_OFFSET
        self.fields.set_config_field(config, "adc_i1_offset", 0x81CE)
        self.fields.set_config_field(config, "adc_i1_scale", 0x0100)
        #ABN_DECODER_MODE
        self.fields.set_config_field(config, "apol", 0x00)
        self.fields.set_config_field(config, "bpol", 0x00)
        self.fields.set_config_field(config, "npol", 0x00)
        self.fields.set_config_field(config, "use_abn_as_n", 0x00)
        self.fields.set_config_field(config, "cln", 0x00)
        self.fields.set_config_field(config, "direction", 0x00)
        #ABN_DECODER_PPR
        self.fields.set_config_field(config, "abn_decoder_ppr", 0x00001000)
        #ABN_DECODER_COUNT
        self.fields.set_config_field(config, "abn_decoder_count", 0x00) #encoder count
        #ABN_DECODER_PHI_E_PHI_M_OFFSET
        self.fields.set_config_field(config, "abn_decoder_phi_m_offset", 0x00)
        self.fields.set_config_field(config, "abn_decoder_phi_e_offset", 0x00)
        #PID_TORQUE_FLUX_LIMITS
        self.fields.set_config_field(config, "pid_torque_flux_limits", 0x7fff)
        #PID_VELOCITY_LIMIT
        self.fields.set_config_field(config, "pid_velocity_limit", 0x7fffffff)
        #PID_TORQUE_P_TORQUE_I
        self.fields.set_config_field(config, "ki_torque", 0x012C)
        self.fields.set_config_field(config, "kp_torque", 0x0FA0)
        #PID_FLUX_P_FLUX_I
        self.fields.set_config_field(config, "ki_flux", 0x0100)
        self.fields.set_config_field(config, "kp_flux", 0x0100)
        #PID_VELOCITY_P_VELOCITY_I
        self.fields.set_config_field(config, "ki_velocity", 0x00C8)
        self.fields.set_config_field(config, "kp_velocity", 0x1388)
        #PID_POSITION_P_POSITION_I
        self.fields.set_config_field(config, "ki_position", 0x00C8)
        self.fields.set_config_field(config, "kp_position", 0x0FA0)
        #MODE_RAMP_MODE_MOTION
        self.fields.set_config_field(config, "mode_motion", 0x08 + 0x80) #Initialize in open-loop mode
        self.fields.set_config_field(config, "mode_pid_smpl", 0x00)
        self.fields.set_config_field(config, "mode_pid_type", 0x00)
        #VELOCITY_SELECTION
        self.fields.set_config_field(config, "velocity_selection", 0x09) #0x09
        #POSITION_SELECTION
        self.fields.set_config_field(config, "position_selection", 0x09) #0x09
        #PHI_E_SELECTION
        self.fields.set_config_field(config, "phi_e_selection", 0x02) #0x05
        
        self.fields.set_config_field(config,"open_loop_mode", 0)
        self.fields.set_config_field(config,"openloop_acceleration", 100)
        #UQ_UD_EXT
        self.fields.set_config_field(config,"ud_ext", 2000)
        self.fields.set_config_field(config,"uq_ext", 0)        


def load_config_prefix(config):
    '''
    Load TMC4671 from config file.
    '''
    return TMC4671(config)
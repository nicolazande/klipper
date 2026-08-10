# TMC4671 FOC servo controller - configuration and supervision
#
# Copyright (C) 2024  Nicola Zandegiacomo <nicola.zandegiacomo@flyingbasket.com>
# Copyright (C) 2026  Warmbird
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# The TMC4671 closes the torque/velocity/position loops in hardware;
# runtime setpoint streaming is done by the serialservo mcu code (see
# src/serialservo.c).  This module owns chip bring-up and supervision:
# safe connect-time configuration (motor never energized), per-boot ADC
# offset calibration, encoder alignment, enable/disable sequencing,
# fault polling with static register scrubbing, and shutdown-safe
# de-energize messages replayed by the mcu on any shutdown.
#
# Register values follow the TMC4671-LA datasheet (rev 2.x) and the
# tuned TMCL-IDE reference for the DMM DST640 (docs/serialservo/).

import logging
from . import bus, tmc

TMC_FREQUENCY = 25000000.

Registers = {
    "CHIPINFO_DATA":            0x00,
    "CHIPINFO_ADDR":            0x01,
    "ADC_RAW_DATA":             0x02,
    "ADC_RAW_ADDR":             0x03,
    "dsADC_MCFG_B_MCFG_A":      0x04,
    "dsADC_MCLK_A":             0x05,
    "dsADC_MCLK_B":             0x06,
    "dsADC_MDEC_B_MDEC_A":      0x07,
    "ADC_I1_SCALE_OFFSET":      0x08,
    "ADC_I0_SCALE_OFFSET":      0x09,
    "ADC_I_SELECT":             0x0A,
    "ADC_I1_I0_EXT":            0x0B,
    "DS_ANALOG_INPUT_STAGE_CFG": 0x0C,
    "AENC_0_SCALE_OFFSET":      0x0D,
    "AENC_1_SCALE_OFFSET":      0x0E,
    "AENC_2_SCALE_OFFSET":      0x0F,
    "AENC_SELECT":              0x11,
    "ADC_IWY_IUX":              0x12,
    "ADC_IV":                   0x13,
    "AENC_WY_UX":               0x15,
    "AENC_VN":                  0x16,
    "PWM_POLARITIES":           0x17,
    "PWM_MAXCNT":               0x18,
    "PWM_BBM_H_BBM_L":          0x19,
    "PWM_SV_CHOP":              0x1A,
    "MOTOR_TYPE_N_POLE_PAIRS":  0x1B,
    "PHI_E_EXT":                0x1C,
    "OPENLOOP_MODE":            0x1F,
    "OPENLOOP_ACCELERATION":    0x20,
    "OPENLOOP_VELOCITY_TARGET": 0x21,
    "OPENLOOP_VELOCITY_ACTUAL": 0x22,
    "OPENLOOP_PHI":             0x23,
    "UQ_UD_EXT":                0x24,
    "ABN_DECODER_MODE":         0x25,
    "ABN_DECODER_PPR":          0x26,
    "ABN_DECODER_COUNT":        0x27,
    "ABN_DECODER_COUNT_N":      0x28,
    "ABN_DECODER_PHI_E_PHI_M_OFFSET": 0x29,
    "ABN_DECODER_PHI_E_PHI_M":  0x2A,
    "HALL_MODE":                0x33,
    "HALL_POSITION_060_000":    0x34,
    "HALL_POSITION_180_120":    0x35,
    "HALL_POSITION_300_240":    0x36,
    "HALL_PHI_E_PHI_M_OFFSET":  0x37,
    "HALL_DPHI_MAX":            0x38,
    "HALL_PHI_E_INTERPOLATED_PHI_E": 0x39,
    "HALL_PHI_M":               0x3A,
    "CONFIG_DATA":              0x4D,
    "CONFIG_ADDR":              0x4E,
    "VELOCITY_SELECTION":       0x50,
    "POSITION_SELECTION":       0x51,
    "PHI_E_SELECTION":          0x52,
    "PHI_E":                    0x53,
    "PID_FLUX_P_FLUX_I":        0x54,
    "PID_TORQUE_P_TORQUE_I":    0x56,
    "PID_VELOCITY_P_VELOCITY_I": 0x58,
    "PID_POSITION_P_POSITION_I": 0x5A,
    "PIDOUT_UQ_UD_LIMITS":      0x5D,
    "PID_TORQUE_FLUX_LIMITS":   0x5E,
    "PID_VELOCITY_LIMIT":       0x60,
    "PID_POSITION_LIMIT_LOW":   0x61,
    "PID_POSITION_LIMIT_HIGH":  0x62,
    "MODE_RAMP_MODE_MOTION":    0x63,
    "PID_TORQUE_FLUX_TARGET":   0x64,
    "PID_TORQUE_FLUX_OFFSET":   0x65,
    "PID_VELOCITY_TARGET":      0x66,
    "PID_VELOCITY_OFFSET":      0x67,
    "PID_POSITION_TARGET":      0x68,
    "PID_TORQUE_FLUX_ACTUAL":   0x69,
    "PID_VELOCITY_ACTUAL":      0x6A,
    "PID_POSITION_ACTUAL":      0x6B,
    "INTERIM_DATA":             0x6E,
    "INTERIM_ADDR":             0x6F,
    "ADC_VM_LIMITS":            0x75,
    "TMC4671_INPUTS_RAW":       0x76,
    "TMC4671_OUTPUTS_RAW":      0x77,
    "STATUS_FLAGS":             0x7C,
    "STATUS_MASK":              0x7D,
}

# Registers safe to read at any time (no read side effects, not bank
# switched).  Bank-switched pairs (ADC_RAW, CONFIG, INTERIM) must only
# be used by a single owner and are excluded from generic dumps.
ReadRegisters = [
    "CHIPINFO_DATA", "ADC_IWY_IUX", "ADC_IV", "PWM_POLARITIES",
    "PWM_MAXCNT", "PWM_BBM_H_BBM_L", "PWM_SV_CHOP",
    "MOTOR_TYPE_N_POLE_PAIRS", "OPENLOOP_VELOCITY_ACTUAL", "UQ_UD_EXT",
    "ABN_DECODER_MODE", "ABN_DECODER_PPR", "ABN_DECODER_COUNT",
    "ABN_DECODER_COUNT_N", "ABN_DECODER_PHI_E_PHI_M_OFFSET",
    "ABN_DECODER_PHI_E_PHI_M", "HALL_MODE",
    "HALL_PHI_E_INTERPOLATED_PHI_E", "HALL_PHI_M",
    "VELOCITY_SELECTION", "POSITION_SELECTION", "PHI_E_SELECTION",
    "PHI_E", "PID_FLUX_P_FLUX_I", "PID_TORQUE_P_TORQUE_I",
    "PID_VELOCITY_P_VELOCITY_I", "PID_POSITION_P_POSITION_I",
    "PIDOUT_UQ_UD_LIMITS", "PID_TORQUE_FLUX_LIMITS",
    "PID_VELOCITY_LIMIT", "MODE_RAMP_MODE_MOTION",
    "PID_TORQUE_FLUX_TARGET", "PID_VELOCITY_TARGET",
    "PID_POSITION_TARGET", "PID_TORQUE_FLUX_ACTUAL",
    "PID_VELOCITY_ACTUAL", "PID_POSITION_ACTUAL",
    "TMC4671_INPUTS_RAW", "TMC4671_OUTPUTS_RAW", "STATUS_FLAGS",
]

Fields = {}
Fields["MOTOR_TYPE_N_POLE_PAIRS"] = {
    "pole_pairs": 0xffff << 0, "motor_type": 0xff << 16,
}
Fields["PWM_POLARITIES"] = {
    "low_side_polarity": 0x01 << 0, "high_side_polarity": 0x01 << 1,
}
Fields["PWM_MAXCNT"] = { "pwm_maxcnt": 0xffff << 0 }
Fields["PWM_BBM_H_BBM_L"] = {
    "pwm_bbm_l": 0xff << 0, "pwm_bbm_h": 0xff << 8,
}
Fields["PWM_SV_CHOP"] = { "pwm_chop": 0xff << 0, "pwm_sv": 0x01 << 8 }
Fields["dsADC_MCFG_B_MCFG_A"] = {
    "cfg_dsmodulator_a": 0x03 << 0, "mclk_polarity_a": 0x01 << 2,
    "mdat_polarity_a": 0x01 << 3, "sel_nclk_mclk_i_a": 0x01 << 4,
    "cfg_dsmodulator_b": 0x03 << 16, "mclk_polarity_b": 0x01 << 18,
    "mdat_polarity_b": 0x01 << 19, "sel_nclk_mclk_i_b": 0x01 << 20,
}
Fields["dsADC_MCLK_A"] = { "dsadc_mclk_a": 0xffffffff << 0 }
Fields["dsADC_MCLK_B"] = { "dsadc_mclk_b": 0xffffffff << 0 }
Fields["dsADC_MDEC_B_MDEC_A"] = {
    "dsadc_mdec_a": 0xffff << 0, "dsadc_mdec_b": 0xffff << 16,
}
Fields["ADC_I0_SCALE_OFFSET"] = {
    "adc_i0_offset": 0xffff << 0, "adc_i0_scale": 0xffff << 16,
}
Fields["ADC_I1_SCALE_OFFSET"] = {
    "adc_i1_offset": 0xffff << 0, "adc_i1_scale": 0xffff << 16,
}
Fields["ADC_I_SELECT"] = {
    "adc_i0_select": 0xff << 0, "adc_i1_select": 0xff << 8,
    "adc_i_ux_select": 0x03 << 24, "adc_i_v_select": 0x03 << 26,
    "adc_i_wy_select": 0x03 << 28,
}
Fields["ABN_DECODER_MODE"] = {
    "apol": 0x01 << 0, "bpol": 0x01 << 1, "npol": 0x01 << 2,
    "use_abn_as_n": 0x01 << 3, "cln": 0x01 << 8, "direction": 0x01 << 12,
}
Fields["ABN_DECODER_PPR"] = { "abn_decoder_ppr": 0xffffff << 0 }
Fields["ABN_DECODER_COUNT"] = { "abn_decoder_count": 0xffffff << 0 }
Fields["ABN_DECODER_PHI_E_PHI_M_OFFSET"] = {
    "abn_decoder_phi_m_offset": 0xffff << 0,
    "abn_decoder_phi_e_offset": 0xffff << 16,
}
Fields["ABN_DECODER_PHI_E_PHI_M"] = {
    "abn_decoder_phi_m": 0xffff << 0, "abn_decoder_phi_e": 0xffff << 16,
}
Fields["HALL_MODE"] = {
    "hall_polarity": 0x01 << 0, "hall_synchronous_pwm_sampling": 0x01 << 4,
    "hall_interpolation": 0x01 << 8, "hall_direction": 0x01 << 12,
    "hall_blank": 0xfff << 16,
}
Fields["HALL_PHI_E_INTERPOLATED_PHI_E"] = {
    "hall_phi_e": 0xffff << 0, "hall_phi_e_interpolated": 0xffff << 16,
}
Fields["HALL_PHI_E_PHI_M_OFFSET"] = {
    "hall_phi_m_offset": 0xffff << 0, "hall_phi_e_offset": 0xffff << 16,
}
Fields["VELOCITY_SELECTION"] = {
    "velocity_selection": 0xff << 0, "velocity_meter_selection": 0xff << 8,
}
Fields["POSITION_SELECTION"] = { "position_selection": 0xff << 0 }
Fields["PHI_E_SELECTION"] = { "phi_e_selection": 0xff << 0 }
Fields["PHI_E_EXT"] = { "phi_e_ext": 0xffff << 0 }
Fields["UQ_UD_EXT"] = { "ud_ext": 0xffff << 0, "uq_ext": 0xffff << 16 }
Fields["PID_FLUX_P_FLUX_I"] = {
    "ki_flux": 0xffff << 0, "kp_flux": 0xffff << 16,
}
Fields["PID_TORQUE_P_TORQUE_I"] = {
    "ki_torque": 0xffff << 0, "kp_torque": 0xffff << 16,
}
Fields["PID_VELOCITY_P_VELOCITY_I"] = {
    "ki_velocity": 0xffff << 0, "kp_velocity": 0xffff << 16,
}
Fields["PID_POSITION_P_POSITION_I"] = {
    "ki_position": 0xffff << 0, "kp_position": 0xffff << 16,
}
Fields["PIDOUT_UQ_UD_LIMITS"] = { "pidout_uq_ud_limits": 0xffff << 0 }
Fields["PID_TORQUE_FLUX_LIMITS"] = { "pid_torque_flux_limits": 0xffff << 0 }
Fields["PID_VELOCITY_LIMIT"] = { "pid_velocity_limit": 0xffffffff << 0 }
Fields["PID_POSITION_LIMIT_LOW"] = {
    "pid_position_limit_low": 0xffffffff << 0 }
Fields["PID_POSITION_LIMIT_HIGH"] = {
    "pid_position_limit_high": 0xffffffff << 0 }
Fields["MODE_RAMP_MODE_MOTION"] = {
    "mode_motion": 0xff << 0, "mode_ff": 0xff << 16,
    "mode_pid_smpl": 0x7f << 24,
    "mode_pid_type": 0x01 << 31,
}
Fields["PID_POSITION_ACTUAL"] = {
    "pid_position_actual": 0xffffffff << 0 }
Fields["STATUS_FLAGS"] = { "status_flags": 0xffffffff << 0 }

SignedFields = [
    "abn_decoder_phi_m_offset", "abn_decoder_phi_e_offset",
    "abn_decoder_phi_m", "abn_decoder_phi_e", "hall_phi_e",
    "hall_phi_e_interpolated", "hall_phi_m_offset", "hall_phi_e_offset",
    "phi_e_ext", "ud_ext", "uq_ext", "pid_position_actual",
]

FieldFormatters = {}

# Latched STATUS_FLAGS bits treated as faults when they reappear after
# an explicit clear (TMC4671-LA datasheet chapter 5)
STATUS_NOT_PLL_LOCKED = 1 << 19
STATUS_ADC_I_CLIPPED = 1 << 26
STATUS_AENC_CLIPPED = 1 << 27
STATUS_FAULT_MASK = (STATUS_NOT_PLL_LOCKED | STATUS_ADC_I_CLIPPED
                     | STATUS_AENC_CLIPPED)

# Motion modes (MODE_RAMP_MODE_MOTION mode_motion field)
MODE_STOPPED = 0
MODE_TORQUE = 1
MODE_VELOCITY = 2
MODE_POSITION = 3
MODE_UQ_UD_EXT = 8

# phi_e source selection
PHI_E_EXT_SEL = 1
PHI_E_ABN = 3
PHI_E_HALL = 5

# Power stage off: pwm_chop=0 (gates inactive, freewheel), sv retained
PWM_CHOP_OFF = 0x00000100
# Power stage on: chop mode 7 (centered PWM for FOC)
PWM_CHOP_ON_MASK = 0x07


class ServoBrakePin:
    # Refcount-shared brake output: logical 1 = brake released.  The
    # pin starts and shuts down engaged (spring applied), so any mcu
    # shutdown drops the axis onto the brake independent of the host.
    def __init__(self, mcu_brake):
        self.mcu_brake = mcu_brake
        self.release_count = 0
        self.last_sched_time = 0.
    def release(self, print_time):
        if not self.release_count:
            self._transition(print_time, 1)
        self.release_count += 1
    def engage(self, print_time):
        self.release_count -= 1
        if not self.release_count:
            self._transition(print_time, 0)
    def _transition(self, print_time, value):
        # A transition scheduled behind the mcu's clock fires late
        # and shuts it down ("Timer too close"), and per-pin
        # transitions must carry non-decreasing clocks: floor every
        # transition at fresh mcu time and at the last scheduled
        # transition.  The stale-request case is real - an enable
        # can block on SPI verify reads long enough for its
        # motion-start print time to fall into the past.
        mcu = self.mcu_brake.get_mcu()
        reactor = mcu.get_printer().get_reactor()
        est = mcu.estimated_print_time(reactor.monotonic())
        print_time = max(print_time, self.last_sched_time + 0.001,
                         est + 0.100)
        self.last_sched_time = print_time
        self.mcu_brake.set_digital(print_time, value)

def lookup_brake_pin(config, pin):
    ppins = config.get_printer().lookup_object('pins')
    pin_params = ppins.lookup_pin(pin, can_invert=True,
                                  share_type='tmc4671_brake')
    brake = pin_params.get('class')
    if brake is not None:
        return brake
    mcu_brake = pin_params['chip'].setup_pin('digital_out', pin_params)
    mcu_brake.setup_max_duration(0.)
    mcu_brake.setup_start_value(0, 0)
    brake = pin_params['class'] = ServoBrakePin(mcu_brake)
    return brake


class MCU_TMC4671_SPI:
    # SPI access with single-frame reads (the TMC4671 replies within
    # the same 40 bit datagram) and verify-with-retry writes
    def __init__(self, config, name_to_reg, fields):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.mutex = self.printer.get_reactor().mutex()
        # 1MHz: the chip errata documents MSB read corruption at
        # higher pauseless-read clock rates
        self.spi = bus.MCU_SPI_from_config(config, 3, default_speed=1000000)
        self.name_to_reg = name_to_reg
        self.fields = fields
    def get_fields(self):
        return self.fields
    def get_tmc_frequency(self):
        return TMC_FREQUENCY
    def _do_read(self, reg):
        params = self.spi.spi_transfer([reg & 0x7f, 0, 0, 0, 0])
        pr = bytearray(params['response'])
        return (pr[1] << 24) | (pr[2] << 16) | (pr[3] << 8) | pr[4]
    def _do_write(self, reg, val, minclock=0):
        self.spi.spi_send([(reg | 0x80) & 0xff, (val >> 24) & 0xff,
                           (val >> 16) & 0xff, (val >> 8) & 0xff,
                           val & 0xff], minclock=minclock)
    def get_register(self, reg_name):
        reg = self.name_to_reg[reg_name]
        if self.printer.get_start_args().get('debugoutput') is not None:
            return 0
        with self.mutex:
            return self._do_read(reg)
    def set_register(self, reg_name, val, print_time=None, verify=None):
        reg = self.name_to_reg[reg_name]
        minclock = 0
        if print_time is not None:
            minclock = self.spi.get_mcu().print_time_to_clock(print_time)
            verify = False  # a scheduled write cannot be read back now
        if verify is None:
            verify = reg_name in VerifyRegisters
        if self.printer.get_start_args().get('debugoutput') is not None:
            verify = False
        with self.mutex:
            for retry in range(5):
                self._do_write(reg, val, minclock)
                if not verify:
                    return
                if self._do_read(reg) == val & 0xffffffff:
                    return
        raise self.printer.command_error(
            "Unable to write tmc4671 '%s' register %s" % (self.name,
                                                          reg_name))

# Static configuration registers - written with readback verification
# and scrubbed periodically against the intended values
VerifyRegisters = [
    "dsADC_MCFG_B_MCFG_A", "dsADC_MCLK_A", "dsADC_MCLK_B",
    "dsADC_MDEC_B_MDEC_A", "ADC_I0_SCALE_OFFSET", "ADC_I1_SCALE_OFFSET",
    "ADC_I_SELECT", "DS_ANALOG_INPUT_STAGE_CFG", "PWM_POLARITIES",
    "PWM_MAXCNT", "PWM_BBM_H_BBM_L", "MOTOR_TYPE_N_POLE_PAIRS",
    "ABN_DECODER_MODE", "ABN_DECODER_PPR", "HALL_MODE",
    "HALL_POSITION_060_000", "HALL_POSITION_180_120",
    "HALL_POSITION_300_240", "HALL_PHI_E_PHI_M_OFFSET", "HALL_DPHI_MAX",
    "VELOCITY_SELECTION", "POSITION_SELECTION",
    "PID_FLUX_P_FLUX_I", "PID_TORQUE_P_TORQUE_I",
    "PID_VELOCITY_P_VELOCITY_I", "PID_POSITION_P_POSITION_I",
    "PIDOUT_UQ_UD_LIMITS", "PID_TORQUE_FLUX_LIMITS",
    "PID_VELOCITY_LIMIT", "PID_POSITION_LIMIT_LOW",
    "PID_POSITION_LIMIT_HIGH", "STATUS_MASK",
]


class TMC4671:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.stepper_name = ' '.join(config.get_name().split()[1:])
        self.name = config.get_name().split()[-1]
        self.fields = tmc.FieldHelper(Fields, SignedFields, FieldFormatters)
        self.mcu_tmc = MCU_TMC4671_SPI(config, Registers, self.fields)
        self.mutex = self.printer.get_reactor().mutex()
        self.stepper = None
        self.stepper_enable = self.printer.load_object(config,
                                                       "stepper_enable")
        self.enabled = False
        self.config_failed = False
        self.adc_calibrated = False
        self.aligned = False
        self.fault_strikes = 0
        self.scrub_index = 0
        self.last_status = {}
        self.check_timer = None
        self.monitor_timer = None
        self.monitor_count = 0
        # Motor/encoder configuration (must match the machine)
        self.pole_pairs = config.getint('pole_pairs', minval=1, maxval=120)
        self.encoder_resolution = config.getint('encoder_resolution',
                                                minval=4)
        self.encoder_direction = config.getboolean('encoder_direction',
                                                   True)
        # Current control.  current_scale_ma_per_lsb depends on the
        # power stage (shunt + amplifier gain) and MUST be validated
        # for each board design before trusting any ampere value.
        self.current_scale = config.getfloat('current_scale_ma_per_lsb',
                                             1.0, above=0.)
        self.run_current = config.getfloat('run_current', above=0.)
        self.velocity_limit = config.getfloat('velocity_limit', 3000.,
                                              above=0.)
        self.dead_time_ns = config.getint('dead_time_ns', 250, minval=0,
                                          maxval=2550)
        # Encoder alignment strategy
        # Default: zero-motion startup - the hall angle seeds the ABN
        # commutation offset (encoder + halls are wired on every Z
        # motor).  'forced' rotates the rotor and is the commissioning
        # fallback until hall polarity/direction are verified.
        self.align_mode = config.getchoice('align_mode', {
            'forced': 'forced', 'hall': 'hall', 'manual': 'manual'},
            'hall')
        self.align_voltage = config.getint('align_voltage', 1000,
                                           minval=100, maxval=8000)
        self.align_delay = config.getfloat('align_delay', 1.0,
                                           minval=0.2, maxval=5.)
        # Electromagnetic holding brake (optional, shareable between
        # the Z servos): released while any sharing servo is enabled,
        # engaged before de-energizing and on any shutdown
        self.brake = None
        brake_pin = config.get('brake_pin', None)
        if brake_pin is not None:
            self.brake = lookup_brake_pin(config, brake_pin)
        self.brake_engage_time = config.getfloat('brake_engage_time',
                                                 0.200, minval=0.,
                                                 maxval=2.)
        self.brake_release_time = config.getfloat('brake_release_time',
                                                  0.200, minval=0.,
                                                  maxval=2.)
        # Board-specific analog frontend configuration
        self.adc_i_select = int(config.get('adc_i_select',
                                           '0x18000100'), 0)
        self.analog_input_cfg = int(config.get('analog_input_stage_cfg',
                                               '0x00044400'), 0)
        self._setup_register_defaults(config)
        # Event handlers
        self.printer.register_event_handler("klippy:mcu_identify",
                                            self._handle_mcu_identify)
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        self.printer.register_event_handler("gcode:request_restart",
                                            self._handle_request_restart)
        # Commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("INIT_TMC4671", "STEPPER", self.name,
                                   self.cmd_INIT_TMC4671,
                                   desc=self.cmd_INIT_TMC4671_help)
        gcode.register_mux_command("SET_TMC4671_FIELD", "STEPPER", self.name,
                                   self.cmd_SET_TMC4671_FIELD,
                                   desc=self.cmd_SET_TMC4671_FIELD_help)
        gcode.register_mux_command("SET_TMC4671_CURRENT", "STEPPER",
                                   self.name, self.cmd_SET_TMC4671_CURRENT,
                                   desc=self.cmd_SET_TMC4671_CURRENT_help)
        gcode.register_mux_command("DUMP_TMC4671", "STEPPER", self.name,
                                   self.cmd_DUMP_TMC4671,
                                   desc=self.cmd_DUMP_TMC4671_help)
        gcode.register_mux_command("TMC4671_CALIBRATE_ADC", "STEPPER",
                                   self.name, self.cmd_TMC4671_CALIBRATE_ADC,
                                   desc=self.cmd_TMC4671_CALIBRATE_ADC_help)
        gcode.register_mux_command("TMC4671_ALIGN_ENCODER", "STEPPER",
                                   self.name, self.cmd_TMC4671_ALIGN_ENCODER,
                                   desc=self.cmd_TMC4671_ALIGN_ENCODER_help)
        gcode.register_mux_command("TMC4671_STATUS", "STEPPER", self.name,
                                   self.cmd_TMC4671_STATUS,
                                   desc=self.cmd_TMC4671_STATUS_help)
        gcode.register_mux_command("TMC4671_MONITOR", "STEPPER", self.name,
                                   self.cmd_TMC4671_MONITOR,
                                   desc=self.cmd_TMC4671_MONITOR_help)
    def _calc_current_limit(self, current):
        limit = int(current * 1000. / self.current_scale + .5)
        return max(0, min(0x7fff, limit))
    def _setup_register_defaults(self, config):
        # Stage the full register configuration.  Values are the tuned
        # TMCL-IDE reference for the DMM DST640 where motor-bound, and
        # explicit config options where board-bound.  Every field
        # remains overridable via driver_<FIELD> options.
        setf = self.fields.set_field
        set_config_field = self.fields.set_config_field
        # Motor
        set_config_field(config, "motor_type", 3)
        setf("pole_pairs", self.pole_pairs)
        # PWM: 25kHz, polarities and dead time are power-stage-bound
        set_config_field(config, "low_side_polarity", 0)
        set_config_field(config, "high_side_polarity", 0)
        set_config_field(config, "pwm_maxcnt", 0x0F9F)
        bbm = max(0, min(0xff, (self.dead_time_ns + 5) // 10))
        setf("pwm_bbm_l", bbm)
        setf("pwm_bbm_h", bbm)
        # Power stage starts OFF (chop=0); enable turns it on
        setf("pwm_chop", 0)
        set_config_field(config, "pwm_sv", 1)
        # dsADC frontend: internal modulators, 25MHz group A clock,
        # sinc3 decimation of exactly one 25kHz PWM period (MDEC=334)
        set_config_field(config, "cfg_dsmodulator_a", 0)
        set_config_field(config, "mclk_polarity_a", 0)
        set_config_field(config, "mdat_polarity_a", 0)
        set_config_field(config, "sel_nclk_mclk_i_a", 1)
        set_config_field(config, "cfg_dsmodulator_b", 0)
        set_config_field(config, "mclk_polarity_b", 0)
        set_config_field(config, "mdat_polarity_b", 0)
        set_config_field(config, "sel_nclk_mclk_i_b", 1)
        set_config_field(config, "dsadc_mclk_a", 0x20000000)
        set_config_field(config, "dsadc_mclk_b", 0)
        set_config_field(config, "dsadc_mdec_a", 0x014E)
        set_config_field(config, "dsadc_mdec_b", 0x014E)
        # ADC current scaling; offsets are calibrated at every boot
        # (never transplanted between boards), config options exist
        # for bench diagnosis only
        set_config_field(config, "adc_i0_scale", 0x0100)
        set_config_field(config, "adc_i0_offset", 0x8000)
        set_config_field(config, "adc_i1_scale", 0x0100)
        set_config_field(config, "adc_i1_offset", 0x8000)
        # ADC_I_SELECT and the analog input stage are board-bound raw
        # values (see docs); stage them as whole registers
        self.reg_overrides = {
            "ADC_I_SELECT": self.adc_i_select,
            "DS_ANALOG_INPUT_STAGE_CFG": self.analog_input_cfg,
        }
        # ABN encoder: PPR is 4x the line count (quadrature)
        set_config_field(config, "apol", 0)
        set_config_field(config, "bpol", 0)
        set_config_field(config, "npol", 0)
        set_config_field(config, "use_abn_as_n", 0)
        set_config_field(config, "cln", 0)
        setf("direction", 1 if self.encoder_direction else 0)
        setf("abn_decoder_ppr", self.encoder_resolution)
        set_config_field(config, "abn_decoder_phi_m_offset", 0)
        set_config_field(config, "abn_decoder_phi_e_offset", 0)
        # Hall sensors: polarity/direction need bench commissioning;
        # interpolation stays off (silicon erratum with hall
        # interpolation + position feedback)
        set_config_field(config, "hall_polarity", 0)
        set_config_field(config, "hall_synchronous_pwm_sampling", 0)
        set_config_field(config, "hall_interpolation", 0)
        set_config_field(config, "hall_direction", 0)
        set_config_field(config, "hall_blank", 2)
        # Hall geometry registers: chip power-on defaults unless
        # overridden by config (commissioning results), written
        # explicitly so stale values cannot survive a host restart and
        # poison the hall-based alignment (they are also scrubbed)
        self.reg_overrides["HALL_POSITION_060_000"] = int(
            config.get('hall_position_060_000', '0x2AAA0000'), 0)
        self.reg_overrides["HALL_POSITION_180_120"] = int(
            config.get('hall_position_180_120', '0x80005555'), 0)
        self.reg_overrides["HALL_POSITION_300_240"] = int(
            config.get('hall_position_300_240', '0xD555AAAA'), 0)
        self.reg_overrides["HALL_DPHI_MAX"] = int(
            config.get('hall_dphi_max', '0x2AAA'), 0)
        # The phi offsets have field definitions - keep them
        # field-based so config, SET_TMC4671_FIELD and the register
        # scrub all agree on the intended value
        set_config_field(config, "hall_phi_m_offset", 0)
        set_config_field(config, "hall_phi_e_offset", 0)
        # Feedback selections: electrical-angle domain (keeps the
        # tuned PID gains valid; unit conversion is host-side)
        set_config_field(config, "velocity_selection", 0)
        set_config_field(config, "velocity_meter_selection", 0)
        set_config_field(config, "position_selection", 0)
        set_config_field(config, "phi_e_selection", PHI_E_ABN)
        # PID gains (tuned TMCL-IDE values for the DST640)
        set_config_field(config, "kp_torque", 0x00F5)
        set_config_field(config, "ki_torque", 0x00C1)
        set_config_field(config, "kp_flux", 0x00F5)
        set_config_field(config, "ki_flux", 0x00C1)
        set_config_field(config, "kp_velocity", 0x4E20)
        set_config_field(config, "ki_velocity", 0x04B0)
        set_config_field(config, "kp_position", 0x0050)
        set_config_field(config, "ki_position", 0x0014)
        # Limits
        set_config_field(config, "pidout_uq_ud_limits", 0x5A81)
        setf("pid_torque_flux_limits",
             self._calc_current_limit(self.run_current))
        vel_limit = int(self.velocity_limit * self.pole_pairs + .5)
        setf("pid_velocity_limit", vel_limit)
        set_config_field(config, "pid_position_limit_low", -0x80000000
                         + 1)
        set_config_field(config, "pid_position_limit_high", 0x7fffffff)
        # Motion mode: stopped, parallel PI, no ramp bits (dead on -LA)
        setf("mode_motion", MODE_STOPPED)
        set_config_field(config, "mode_ff", 1)
        set_config_field(config, "mode_pid_smpl", 0)
        set_config_field(config, "mode_pid_type", 0)
    def _handle_mcu_identify(self):
        force_move = self.printer.lookup_object("force_move")
        self.stepper = force_move.lookup_stepper(self.stepper_name)
        if self.stepper.get_pole_pairs() != self.pole_pairs:
            raise self.printer.config_error(
                "tmc4671 %s: pole_pairs (%d) must match the %s section"
                " (%d)" % (self.name, self.pole_pairs, self.stepper_name,
                           self.stepper.get_pole_pairs()))
        # Register shutdown-safe de-energize messages: replayed by the
        # mcu (spidev_shutdown) on ANY shutdown, in registration order
        spi = self.mcu_tmc.spi
        for reg_name, val in [
                ("MODE_RAMP_MODE_MOTION", 0),
                ("PID_VELOCITY_OFFSET", 0),
                ("UQ_UD_EXT", 0),
                ("PWM_SV_CHOP", PWM_CHOP_OFF)]:
            reg = Registers[reg_name]
            spi.setup_shutdown_msg([(reg | 0x80) & 0xff,
                                    (val >> 24) & 0xff, (val >> 16) & 0xff,
                                    (val >> 8) & 0xff, val & 0xff])
        # Wire enable/disable sequencing to the stepper enable line
        enable_line = self.stepper_enable.lookup_enable(self.stepper_name)
        enable_line.register_state_callback(self._handle_stepper_enable)
    def _handle_request_restart(self, print_time):
        # A host restart tears the session down before minclock-held
        # de-energize writes can flush, leaving the chip energized
        # until the next connect's safe-state init.  Disable now if
        # still enabled, then hold the restart until the scheduled
        # writes (brake engage + power-stage off) have gone out.
        if self.enabled:
            self._do_disable(print_time)
        reactor = self.printer.get_reactor()
        reactor.pause(reactor.monotonic() + self.brake_engage_time + 0.100)
    def _handle_stepper_enable(self, print_time, is_enable):
        if is_enable:
            cb = (lambda ev: self._do_enable(print_time))
        else:
            cb = (lambda ev: self._do_disable(print_time))
        self.printer.get_reactor().register_callback(cb)
    def _init_registers(self):
        # Full safe (de-energized) chip configuration.  Explicit
        # ordering: force-safe registers first, then static config.
        setr = self.mcu_tmc.set_register
        # Force safe state (chip may hold stale state across host
        # restarts; a whole-register write is the only STATUS clear)
        setr("PWM_SV_CHOP", PWM_CHOP_OFF, verify=True)
        setr("MODE_RAMP_MODE_MOTION", 0, verify=True)
        for reg in ["PID_TORQUE_FLUX_TARGET", "PID_TORQUE_FLUX_OFFSET",
                    "PID_VELOCITY_TARGET", "PID_VELOCITY_OFFSET",
                    "UQ_UD_EXT", "PHI_E_EXT", "OPENLOOP_MODE",
                    "OPENLOOP_ACCELERATION", "OPENLOOP_VELOCITY_TARGET",
                    "OPENLOOP_PHI"]:
            setr(reg, 0, verify=False)
        setr("STATUS_FLAGS", 0, verify=False)
        setr("STATUS_MASK", 0)
        # Static configuration
        for reg_name in list(self.fields.registers.keys()):
            if reg_name in ("PWM_SV_CHOP", "MODE_RAMP_MODE_MOTION"):
                continue
            val = self.reg_overrides.get(reg_name,
                                         self.fields.registers[reg_name])
            setr(reg_name, val)
        for reg_name, val in self.reg_overrides.items():
            setr(reg_name, val)
    def _full_bringup(self):
        # Safe register init, per-boot ADC offset calibration and
        # encoder alignment (the forced method can rotate the motor by
        # up to half an electrical revolution - use align_mode: hall
        # or manual for zero-motion bring-up once commissioned).
        with self.mutex:
            if self.enabled:
                # A deferred enable can win the race against a
                # bring-up requested while the axis looked disabled
                # (e.g. INIT_TMC4671's wait_moves yield): refuse
                # rather than safe-state a powered chip that the host
                # believes is enabled with the brake released
                raise self.printer.command_error(
                    "TMC4671 %s: motor enabled during bring-up -"
                    " disable and retry" % (self.name,))
            self.aligned = False
            self._init_registers()
            if self.printer.get_start_args().get('debugoutput') is None:
                self._calibrate_adc_offsets()
                if self.align_mode != 'manual':
                    self._stage_on()
                    try:
                        if self.align_mode == 'hall':
                            self._align_encoder_hall()
                        else:
                            self._align_encoder_forced()
                    finally:
                        self._stage_off()
        self.config_failed = False
    def _handle_connect(self):
        # Full bring-up happens at connect while nothing can move;
        # enable/disable afterwards only toggles power stage and mode.
        try:
            self._full_bringup()
        except self.printer.command_error as e:
            self.config_failed = True
            logging.info("TMC4671 %s failed to init: %s", self.name, str(e))
            return
        # Hand the SPI device to the serialservo mcu code for runtime
        # setpoint streaming, then anchor the host position frame
        try:
            self.stepper.setup_spi(self.mcu_tmc.spi.get_oid())
            self.stepper.note_homing_end()
        except Exception:
            logging.exception("TMC4671 %s: serialservo spi setup failed",
                              self.name)
    def _set_motion_mode(self, mode, print_time=None):
        reg_val = self.fields.set_field("mode_motion", mode)
        self.mcu_tmc.set_register("MODE_RAMP_MODE_MOTION", reg_val,
                                  print_time=print_time, verify=False)
    def _stage_on(self):
        chop = (self.fields.registers["PWM_SV_CHOP"] & ~0xff) \
            | PWM_CHOP_ON_MASK
        self.mcu_tmc.set_register("PWM_SV_CHOP", chop, verify=True)
    def _stage_off(self):
        chop = self.fields.registers["PWM_SV_CHOP"] & ~0xff
        self.mcu_tmc.set_register("PWM_SV_CHOP", chop, verify=True)
    def _pause(self, seconds):
        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()
        reactor.pause(eventtime + seconds)
    def _calibrate_adc_offsets(self):
        # Per-boot zero-current ADC offset calibration (power stage
        # must be off).  Never use transplanted offsets.
        if self.printer.get_start_args().get('debugoutput') is not None:
            self.adc_calibrated = True
            return
        setr = self.mcu_tmc.set_register
        getr = self.mcu_tmc.get_register
        chop = getr("PWM_SV_CHOP")
        if chop & 0xff:
            raise self.printer.command_error(
                "TMC4671 %s: ADC calibration requires power stage off"
                % (self.name,))
        setr("ADC_RAW_ADDR", 0, verify=False)
        total0 = total1 = 0
        count = 32
        for i in range(count):
            raw = getr("ADC_RAW_DATA")
            total0 += raw & 0xffff
            total1 += (raw >> 16) & 0xffff
            self._pause(0.002)
        offs0 = (total0 + count // 2) // count
        offs1 = (total1 + count // 2) // count
        # Sanity: offsets must be near mid scale (25%..75%)
        for offs, chan in ((offs0, "I0"), (offs1, "I1")):
            if not (0x4000 <= offs <= 0xC000):
                raise self.printer.command_error(
                    "TMC4671 %s: ADC %s offset calibration implausible"
                    " (0x%04X)" % (self.name, chan, offs))
        self.fields.set_field("adc_i0_offset", offs0)
        self.fields.set_field("adc_i1_offset", offs1)
        setr("ADC_I0_SCALE_OFFSET",
             self.fields.registers["ADC_I0_SCALE_OFFSET"])
        setr("ADC_I1_SCALE_OFFSET",
             self.fields.registers["ADC_I1_SCALE_OFFSET"])
        self.adc_calibrated = True
        logging.info("TMC4671 %s: ADC offsets calibrated I0=0x%04X"
                     " I1=0x%04X", self.name, offs0, offs1)
    def _align_encoder_forced(self):
        # Commissioning-grade forced rotor alignment: park the rotor
        # at phi_e=0 with a UD-only voltage, then zero the decoder.
        # The rotor can move up to half an electrical revolution.
        setr = self.mcu_tmc.set_register
        setr("PHI_E_SELECTION", PHI_E_EXT_SEL, verify=False)
        setr("PHI_E_EXT", 0, verify=False)
        setr("UQ_UD_EXT", 0, verify=False)
        self._set_motion_mode(MODE_UQ_UD_EXT)
        # Ramp UD to the alignment voltage
        steps = 8
        for i in range(1, steps + 1):
            setr("UQ_UD_EXT", (self.align_voltage * i) // steps,
                 verify=False)
            self._pause(0.05)
        self._pause(self.align_delay)
        setr("ABN_DECODER_COUNT", 0, verify=False)
        setr("ABN_DECODER_PHI_E_PHI_M_OFFSET", 0, verify=False)
        setr("UQ_UD_EXT", 0, verify=False)
        self._set_motion_mode(MODE_STOPPED)
        setr("PHI_E_SELECTION", PHI_E_ABN, verify=False)
        self.aligned = True
        logging.info("TMC4671 %s: forced encoder alignment complete",
                     self.name)
    def _align_encoder_hall(self):
        # Zero-motion alignment: copy the hall electrical angle into
        # the ABN phi_e offset (coarse, +-30deg electrical; requires
        # commissioned hall polarity/direction)
        setr = self.mcu_tmc.set_register
        getr = self.mcu_tmc.get_register
        setr("ABN_DECODER_PHI_E_PHI_M_OFFSET", 0, verify=False)
        hall = getr("HALL_PHI_E_INTERPOLATED_PHI_E")
        # The motor is stationary at bring-up: an unstable hall angle
        # means unwired/uncommissioned halls - fail safe rather than
        # energize with a garbage commutation offset
        self._pause(0.050)
        hall2 = getr("HALL_PHI_E_INTERPOLATED_PHI_E")
        if (hall ^ hall2) & 0xffff:
            raise self.printer.command_error(
                "TMC4671 %s: hall angle unstable (0x%04X vs 0x%04X) -"
                " check hall wiring/commissioning or set align_mode:"
                " forced" % (self.name, hall & 0xffff, hall2 & 0xffff))
        hall_phi_e = hall & 0xffff
        abn = getr("ABN_DECODER_PHI_E_PHI_M")
        abn_phi_e = (abn >> 16) & 0xffff
        offset = (hall_phi_e - abn_phi_e) & 0xffff
        self.fields.set_field("abn_decoder_phi_e_offset",
                              offset - 0x10000 if offset >= 0x8000
                              else offset)
        setr("ABN_DECODER_PHI_E_PHI_M_OFFSET", offset << 16, verify=False)
        setr("PHI_E_SELECTION", PHI_E_ABN, verify=False)
        self.aligned = True
        logging.info("TMC4671 %s: hall encoder alignment offset=0x%04X",
                     self.name, offset)
    def _seed_position(self):
        # Sync the position target to the measured position (writing
        # PID_POSITION_ACTUAL auto-copies into PID_POSITION_TARGET,
        # preventing any jump on loop closure)
        getr = self.mcu_tmc.get_register
        actual = getr("PID_POSITION_ACTUAL")
        self.mcu_tmc.set_register("PID_POSITION_ACTUAL", actual,
                                  verify=False)
    def _do_enable(self, print_time=None):
        # Fast path: power stage on and closed loop entry.  ADC
        # calibration and alignment already happened at connect.
        try:
            with self.mutex:
                if self.enabled:
                    return
                if self.printer.get_start_args().get('debugoutput') \
                   is not None:
                    self.enabled = True
                    return
                if self.config_failed:
                    raise self.printer.command_error(
                        "TMC4671 %s: bring-up failed, fix and run"
                        " INIT_TMC4671 before enabling" % (self.name,))
                if self.align_mode != 'manual' and not self.aligned:
                    raise self.printer.command_error(
                        "TMC4671 %s: encoder not aligned" % (self.name,))
                self._stage_on()
                self._seed_position()
                self.mcu_tmc.set_register("PID_VELOCITY_OFFSET", 0,
                                          verify=False)
                self._set_motion_mode(MODE_POSITION)
                if self.brake is not None and print_time is not None:
                    # Loop is closed and holding - release the brake
                    # ahead of the first move so the spring mechanism
                    # has physically disengaged when motion starts
                    reactor = self.printer.get_reactor()
                    est = self.brake.mcu_brake.get_mcu() \
                        .estimated_print_time(reactor.monotonic())
                    release_time = max(print_time - self.brake_release_time,
                                       est + 0.100)
                    self.brake.release(min(release_time, print_time))
                self.enabled = True
                self.fault_strikes = 0
            self._start_checks()
            logging.info("TMC4671 %s: enabled (closed loop position)",
                         self.name)
        except self.printer.command_error as e:
            logging.error("TMC4671 %s enable failed: %s", self.name, str(e))
            self.printer.invoke_shutdown(str(e))
    def _do_disable(self, print_time=None):
        # De-energize writes are scheduled at print_time (the end of
        # buffered motion) so M84 after buffered moves cannot cut the
        # power stage mid-motion.
        try:
            self._stop_checks()
            with self.mutex:
                if not self.enabled:
                    return
                self.enabled = False
                if self.printer.get_start_args().get('debugoutput') \
                   is not None:
                    return
                setr = self.mcu_tmc.set_register
                off_time = print_time
                if self.brake is not None and print_time is not None:
                    # Engage the brake while still holding torque,
                    # then de-energize once it has settled
                    self.brake.engage(print_time)
                    off_time = print_time + self.brake_engage_time
                self._set_motion_mode(MODE_STOPPED, print_time=off_time)
                setr("PID_VELOCITY_OFFSET", 0, print_time=off_time,
                     verify=False)
                setr("UQ_UD_EXT", 0, print_time=off_time, verify=False)
                setr("PWM_SV_CHOP", PWM_CHOP_OFF, print_time=off_time,
                     verify=(off_time is None))
            logging.info("TMC4671 %s: disabled (power stage off)",
                         self.name)
        except self.printer.command_error as e:
            logging.error("TMC4671 %s disable failed: %s", self.name,
                          str(e))
            self.printer.invoke_shutdown(str(e))
    def _start_checks(self):
        if self.check_timer is None:
            reactor = self.printer.get_reactor()
            curtime = reactor.monotonic()
            self.check_timer = reactor.register_timer(self._periodic_check,
                                                      curtime + 1.)
    def _stop_checks(self):
        if self.check_timer is not None:
            self.printer.get_reactor().unregister_timer(self.check_timer)
            self.check_timer = None
    def _periodic_check(self, eventtime):
        try:
            status = self.mcu_tmc.get_register("STATUS_FLAGS")
            self.last_status = {'status_flags': status}
            if status & STATUS_FAULT_MASK:
                self.fault_strikes += 1
                self.mcu_tmc.set_register("STATUS_FLAGS", 0, verify=False)
                logging.warning("TMC4671 %s: fault flags 0x%08X (strike"
                                " %d)", self.name, status,
                                self.fault_strikes)
                if self.fault_strikes >= 3:
                    raise self.printer.command_error(
                        "TMC4671 %s reports persistent fault flags 0x%08X"
                        % (self.name, status & STATUS_FAULT_MASK))
            else:
                self.fault_strikes = 0
            # Scrub one static register per cycle against its intended
            # value (the chip has no SPI CRC)
            reg_names = [r for r in VerifyRegisters
                         if r in self.fields.registers
                         or r in self.reg_overrides]
            if reg_names:
                self.scrub_index = (self.scrub_index + 1) % len(reg_names)
                reg_name = reg_names[self.scrub_index]
                expected = self.reg_overrides.get(
                    reg_name, self.fields.registers.get(reg_name, 0))
                actual = self.mcu_tmc.get_register(reg_name)
                if actual != expected & 0xffffffff:
                    raise self.printer.command_error(
                        "TMC4671 %s: register %s corrupted (0x%08X !="
                        " 0x%08X)" % (self.name, reg_name, actual,
                                      expected))
        except self.printer.command_error as e:
            self.printer.invoke_shutdown(str(e))
            return self.printer.get_reactor().NEVER
        return eventtime + 1.
    def get_status(self, eventtime=None):
        return {'enabled': self.enabled,
                'adc_calibrated': self.adc_calibrated,
                'aligned': self.aligned,
                'status_flags': self.last_status.get('status_flags'),
                'run_current': self.run_current}
    cmd_INIT_TMC4671_help = "Re-initialize TMC4671 registers (safe state)"
    def cmd_INIT_TMC4671(self, gcmd):
        if self.enabled:
            raise gcmd.error("TMC4671 %s: disable the motor before"
                             " INIT_TMC4671" % (self.name,))
        self.printer.lookup_object('toolhead').wait_moves()
        try:
            self._full_bringup()
        except self.printer.command_error as e:
            # The enabled-refusal raises before touching the chip: a
            # deferred enable won the race and the axis is running
            # normally - do not poison its state (config_failed=True
            # plus aligned=False would turn the next enable into an
            # invoke_shutdown on a healthy axis)
            if not self.enabled:
                self.config_failed = True
            raise gcmd.error(str(e))
        # Re-bind the SPI device to the serialservo mcu streamer and
        # re-anchor the host frame (mirrors the connect flow - without
        # this, motion after a recovery shut down the mcu with
        # "serialservo spi not configured")
        if self.stepper is not None:
            try:
                self.stepper.setup_spi(self.mcu_tmc.spi.get_oid())
                self.stepper.note_homing_end()
            except Exception:
                logging.exception("TMC4671 %s: serialservo spi re-bind"
                                  " failed", self.name)
        gcmd.respond_info("TMC4671 %s re-initialized (calibrated%s,"
                          " motor de-energized)"
                          % (self.name, ", aligned" if self.aligned
                             else ""))
    cmd_SET_TMC4671_FIELD_help = "Set a TMC4671 register field"
    def cmd_SET_TMC4671_FIELD(self, gcmd):
        field_name = gcmd.get('FIELD').lower()
        reg_name = self.fields.lookup_register(field_name, None)
        if reg_name is None:
            raise gcmd.error("Unknown field name '%s'" % (field_name,))
        value = gcmd.get_int('VALUE')
        # Whole-register overrides take precedence over the field
        # cache (in the scrub and at re-init): compose on the override
        # and keep it updated, or the runtime write would be flagged
        # as register corruption and silently reverted by INIT
        base = self.reg_overrides.get(reg_name)
        reg_val = self.fields.set_field(field_name, value, reg_value=base,
                                        reg_name=reg_name)
        with self.mutex:
            if reg_name in self.reg_overrides:
                self.reg_overrides[reg_name] = reg_val
            self.mcu_tmc.set_register(reg_name, reg_val)
    cmd_SET_TMC4671_CURRENT_help = "Set the TMC4671 torque/flux current limit"
    def cmd_SET_TMC4671_CURRENT(self, gcmd):
        run_current = gcmd.get_float('CURRENT', None, above=0.)
        if run_current is not None:
            self.run_current = run_current
            limit = self._calc_current_limit(run_current)
            self.fields.set_field("pid_torque_flux_limits", limit)
            with self.mutex:
                self.mcu_tmc.set_register(
                    "PID_TORQUE_FLUX_LIMITS",
                    self.fields.registers["PID_TORQUE_FLUX_LIMITS"])
        gcmd.respond_info("Run current: %.2fA (limit=%d lsb, scale=%.3f"
                          " mA/lsb)" % (self.run_current,
                                        self._calc_current_limit(
                                            self.run_current),
                                        self.current_scale))
    cmd_DUMP_TMC4671_help = "Read and display TMC4671 registers"
    def cmd_DUMP_TMC4671(self, gcmd):
        reg_name = gcmd.get('REGISTER', None)
        if reg_name is not None:
            reg_name = reg_name.upper()
            if reg_name not in Registers:
                raise gcmd.error("Unknown register name '%s'" % (reg_name,))
            if reg_name not in ReadRegisters:
                raise gcmd.error("Register '%s' is not safe for generic"
                                 " reads" % (reg_name,))
            val = self.mcu_tmc.get_register(reg_name)
            gcmd.respond_info(self.fields.pretty_format(reg_name, val))
            return
        gcmd.respond_info("========== Queried registers ==========")
        for reg_name in ReadRegisters:
            val = self.mcu_tmc.get_register(reg_name)
            gcmd.respond_info(self.fields.pretty_format(reg_name, val))
    cmd_TMC4671_CALIBRATE_ADC_help = "Run zero-current ADC offset calibration"
    def cmd_TMC4671_CALIBRATE_ADC(self, gcmd):
        if self.enabled:
            raise gcmd.error("TMC4671 %s: disable the motor before ADC"
                             " calibration" % (self.name,))
        with self.mutex:
            self._calibrate_adc_offsets()
        gcmd.respond_info(
            "ADC offsets: I0=0x%04X I1=0x%04X" % (
                self.fields.get_field("adc_i0_offset"),
                self.fields.get_field("adc_i1_offset")))
    cmd_TMC4671_ALIGN_ENCODER_help = "Run the encoder alignment procedure"
    def cmd_TMC4671_ALIGN_ENCODER(self, gcmd):
        mode = gcmd.get('MODE', self.align_mode)
        if mode not in ('forced', 'hall'):
            raise gcmd.error("MODE must be 'forced' or 'hall'")
        if not self.enabled:
            raise gcmd.error("TMC4671 %s: enable the motor first (power"
                             " stage must be on)" % (self.name,))
        self.printer.lookup_object('toolhead').wait_moves()
        with self.mutex:
            if not self.enabled:
                # A deferred disable can win the race during the
                # wait_moves yield: aligning a de-energized power
                # stage records a garbage commutation offset
                raise gcmd.error(
                    "TMC4671 %s: motor disabled during wait - enable"
                    " and retry" % (self.name,))
            # Verify the physical stage, not just the host flag (the
            # mirror of the stage-off assertion in ADC calibration)
            chop = self.mcu_tmc.get_register("PWM_SV_CHOP")
            if (chop & 0xff) != PWM_CHOP_ON_MASK:
                raise gcmd.error(
                    "TMC4671 %s: power stage is off - enable and"
                    " retry" % (self.name,))
            self._set_motion_mode(MODE_STOPPED)
            if mode == 'hall':
                self._align_encoder_hall()
            else:
                self._align_encoder_forced()
            self._seed_position()
            self._set_motion_mode(MODE_POSITION)
        self.stepper.note_homing_end()
        gcmd.respond_info("TMC4671 %s: encoder alignment (%s) complete"
                          % (self.name, mode))
    cmd_TMC4671_STATUS_help = "Report TMC4671 servo state"
    def cmd_TMC4671_STATUS(self, gcmd):
        status = self.mcu_tmc.get_register("STATUS_FLAGS")
        faults = []
        if status & STATUS_NOT_PLL_LOCKED:
            faults.append("not_PLL_locked")
        if status & STATUS_ADC_I_CLIPPED:
            faults.append("adc_i_clipped")
        if status & STATUS_AENC_CLIPPED:
            faults.append("aenc_clipped")
        abn = self.mcu_tmc.get_register("ABN_DECODER_PHI_E_PHI_M")
        hall = self.mcu_tmc.get_register("HALL_PHI_E_INTERPOLATED_PHI_E")
        count = self.mcu_tmc.get_register("ABN_DECODER_COUNT")
        msg = ["TMC4671 %s: enabled=%d aligned=%d adc_calibrated=%d"
               % (self.name, self.enabled, self.aligned,
                  self.adc_calibrated),
               "status_flags=0x%08X%s" % (
                   status, (" FAULTS: " + ",".join(faults)) if faults
                   else ""),
               "abn_count=%d abn_phi_e=%d hall_phi_e=%d" % (
                   count, (abn >> 16) & 0xffff, hall & 0xffff)]
        state = self.stepper.query_state()
        if state is not None:
            msg.append("target=%.4fmm actual=%.4fmm velocity=%.2fmm/s"
                       % (state['target'], state['actual'],
                          state['velocity']))
        gcmd.respond_info("\n".join(msg))
    cmd_TMC4671_MONITOR_help = "Periodically report servo state" \
        " (PERIOD= COUNT= or ENABLE=0)"
    def cmd_TMC4671_MONITOR(self, gcmd):
        enable = gcmd.get_int('ENABLE', 1)
        period = gcmd.get_float('PERIOD', 0.5, minval=0.05, maxval=10.)
        count = gcmd.get_int('COUNT', 20, minval=1, maxval=10000)
        reactor = self.printer.get_reactor()
        if self.monitor_timer is not None:
            reactor.unregister_timer(self.monitor_timer)
            self.monitor_timer = None
        if not enable:
            gcmd.respond_info("TMC4671 %s: monitor stopped" % (self.name,))
            return
        self.monitor_count = count
        gcode = self.printer.lookup_object("gcode")
        def monitor_event(eventtime):
            try:
                abn = self.mcu_tmc.get_register("ABN_DECODER_PHI_E_PHI_M")
                hall = self.mcu_tmc.get_register(
                    "HALL_PHI_E_INTERPOLATED_PHI_E")
                count_reg = self.mcu_tmc.get_register("ABN_DECODER_COUNT")
                state = self.stepper.query_state()
                parts = ["abn_count=%d" % (count_reg,),
                         "abn_phi_e=%d" % ((abn >> 16) & 0xffff,),
                         "hall_phi_e=%d" % (hall & 0xffff,)]
                if state is not None:
                    parts.append("target=%.4f actual=%.4f vel=%.2f"
                                 % (state['target'], state['actual'],
                                    state['velocity']))
                msg = "servo %s: %s" % (self.name, " ".join(parts))
                logging.info(msg)
                gcode.respond_info(msg)
            except Exception:
                logging.exception("TMC4671 monitor error")
                self.monitor_timer = None
                return reactor.NEVER
            self.monitor_count -= 1
            if self.monitor_count <= 0:
                self.monitor_timer = None
                return reactor.NEVER
            return eventtime + period
        curtime = reactor.monotonic()
        self.monitor_timer = reactor.register_timer(monitor_event,
                                                    curtime + period)
        gcmd.respond_info("TMC4671 %s: monitoring %d samples every %.2fs"
                          % (self.name, count, period))


def load_config_prefix(config):
    return TMC4671(config)

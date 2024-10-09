# Micro-controller clock synchronization
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math

RTT_AGE = .000010 / (60. * 60.)
DECAY = 1. / 30.
TRANSMIT_EXTRA = .001

class ClockSync:
    '''
    Clock synchronization class. It takes the main MCU clock as
    reference time, receives its value through serial and updates
    the internal clocks of the following structures:
        - serialqueue: data exanged through serial (z-axis, extruder, ...)
        - ethercatqueue: data exanged through ethercat (x/y-axis)
    '''
    def __init__(self, reactor):
        self.reactor = reactor
        self.serial = None
        self.ethercat = None
        self.get_clock_timer = reactor.register_timer(self._get_clock_event)
        self.get_clock_cmd = self.cmd_queue = None
        self.queries_pending = 0
        self.mcu_freq = 1.
        self.last_clock = 0
        self.clock_est = (0., 0., 0.)
        # Minimum round-trip-time tracking
        self.min_half_rtt = 999999999.9
        self.min_rtt_time = 0.
        # Linear regression of mcu clock and system sent_time
        self.time_avg = self.time_variance = 0.
        self.clock_avg = self.clock_covariance = 0.
        self.prediction_variance = 0.
        self.last_prediction_time = 0.
        
    def connect(self, serial, ethercat):
        '''
        Connect to serial and ethercat interfaces. There is no direct
        communication with the ethercat interface, however the internal
        ethercatqueue structure time needs to be updated through the
        ethercat class object.
        '''
        self.serial = serial
        self.ethercat = ethercat
        self.mcu_freq = serial.msgparser.get_constant_float('CLOCK_FREQ')
        # Load initial clock and frequency
        params = serial.send_with_response('get_uptime', 'uptime')
        self.last_clock = (params['high'] << 32) | params['clock']
        self.clock_avg = self.last_clock
        self.time_avg = params['#sent_time']
        self.clock_est = (self.time_avg, self.clock_avg, self.mcu_freq)
        self.prediction_variance = (.001 * self.mcu_freq)**2
        # Enable periodic get_clock timer
        for i in range(8):
            self.reactor.pause(self.reactor.monotonic() + 0.050)
            self.last_prediction_time = -9999.
            params = serial.send_with_response('get_clock', 'clock')
            self._handle_clock(params)
        self.get_clock_cmd = serial.get_msgparser().create_command('get_clock')
        self.cmd_queue = serial.alloc_command_queue()
        serial.register_response(self._handle_clock, 'clock')
        self.reactor.update_timer(self.get_clock_timer, self.reactor.NOW)
        
    def connect_file(self, serial, ethercat, pace=False):
        '''
        Connect to serial and ethercat interfaces (debug mode).
        '''
        self.serial = serial
        self.ethercat = ethercat
        self.mcu_freq = serial.msgparser.get_constant_float('CLOCK_FREQ')
        self.clock_est = (0., 0., self.mcu_freq)
        freq = 1000000000000.
        if pace:
            freq = self.mcu_freq
        serial.set_clock_est(freq, self.reactor.monotonic(), 0, 0)
        
    def _get_clock_event(self, eventtime):
        '''
        MCU clock querying (_handle_clock is invoked from background thread)
        '''
        self.serial.raw_send(self.get_clock_cmd, 0, 0, self.cmd_queue)
        self.queries_pending += 1
        '''
        Use an unusual time for the next event so clock messages
        don't resonate with other periodic events.
        '''
        return eventtime + .9839
    
    def _handle_clock(self, params):
        '''
        Update mcu clock estimate, this function is executed as callback when
        the mcu replies to a _get_clock_event() request.
        '''
        self.queries_pending = 0
        # extend clock to 64bit
        last_clock = self.last_clock
        clock_delta = (params['clock'] - last_clock) & 0xffffffff
        self.last_clock = clock = last_clock + clock_delta
        # check if this is the best round-trip-time seen so far
        sent_time = params['#sent_time']
        if not sent_time:
            return
        receive_time = params['#receive_time']
        half_rtt = .5 * (receive_time - sent_time)
        aged_rtt = (sent_time - self.min_rtt_time) * RTT_AGE
        if half_rtt < self.min_half_rtt + aged_rtt:
            self.min_half_rtt = half_rtt
            self.min_rtt_time = sent_time
            logging.debug("new minimum rtt %.3f: hrtt=%.6f freq=%d",
                          sent_time, half_rtt, self.clock_est[2])
        # filter out samples that are extreme outliers
        exp_clock = ((sent_time - self.time_avg) * self.clock_est[2] + self.clock_avg)
        clock_diff2 = (clock - exp_clock)**2
        if (clock_diff2 > 25. * self.prediction_variance
            and clock_diff2 > (.000500 * self.mcu_freq)**2):
            if clock > exp_clock and sent_time < self.last_prediction_time+10.:
                logging.debug("Ignoring clock sample %.3f:"
                              " freq=%d diff=%d stddev=%.3f",
                              sent_time, self.clock_est[2], clock - exp_clock,
                              math.sqrt(self.prediction_variance))
                return
            self.prediction_variance = (.001 * self.mcu_freq)**2
        else:
            self.last_prediction_time = sent_time
            self.prediction_variance = (
                (1. - DECAY) * (self.prediction_variance + clock_diff2 * DECAY))
        # add clock and sent_time to linear regression
        diff_sent_time = sent_time - self.time_avg
        self.time_avg += DECAY * diff_sent_time
        self.time_variance = (1. - DECAY) * (
            self.time_variance + diff_sent_time**2 * DECAY)
        diff_clock = clock - self.clock_avg
        self.clock_avg += DECAY * diff_clock
        self.clock_covariance = (1. - DECAY) * (
            self.clock_covariance + diff_sent_time * diff_clock * DECAY)
        # update prediction from linear regression
        new_freq = self.clock_covariance / self.time_variance
        pred_stddev = math.sqrt(self.prediction_variance)
        '''
        NOTE: serialqueue and ethercatqueue clocks are updated separately.
              Since for both a mutex is requested, it is possible that the
              two timers are not updated exactly simultaneously, however
              this is not a problem since absolute clock is used.
        '''
        # update serialqueue clock estimate
        self.serial.set_clock_est(new_freq, self.time_avg + TRANSMIT_EXTRA,
                                  int(self.clock_avg - 3. * pred_stddev), clock)
        # update ethercatqueue clock estimate
        if self.ethercat is not None:
            self.ethercat.set_clock_est(new_freq, self.time_avg + TRANSMIT_EXTRA,
                                        int(self.clock_avg - 3. * pred_stddev), clock)
        # update current mcu clock estimate
        self.clock_est = (self.time_avg + self.min_half_rtt, self.clock_avg, new_freq)
        
    def print_time_to_clock(self, print_time):
        '''
        Convert print time to mcu clock.
        '''
        return int(print_time * self.mcu_freq)
    
    def clock_to_print_time(self, clock):
        '''
        Convert mcu clock to print time.
        '''
        return clock / self.mcu_freq
    
    def get_clock(self, eventtime):
        '''
        Get eventtime (system time) in mcu clocks.
        '''
        sample_time, clock, freq = self.clock_est
        return int(clock + (eventtime - sample_time) * freq)
    
    def estimate_clock_systime(self, reqclock):
        '''
        Estimate system time for a future mcu clock.
        '''
        sample_time, clock, freq = self.clock_est
        return float(reqclock - clock)/freq + sample_time
    
    def estimated_print_time(self, eventtime):
        '''
        Estimate mcu clock for a future system time.
        '''
        return self.clock_to_print_time(self.get_clock(eventtime))
    
    def clock32_to_clock64(self, clock32):
        '''
        Convert uint32_t clock to uint64_t clock without overflow.
        '''
        last_clock = self.last_clock
        clock_diff = (clock32 - last_clock) & 0xffffffff
        clock_diff -= (clock_diff & 0x80000000) << 1
        return last_clock + clock_diff

    def is_active(self):
        '''
        Check if time synchronization is active, this can be deduces
        from the number of pending queries.
        '''
        return self.queries_pending <= 4
    
    def dump_debug(self):
        '''
        Prepare clock synchronization debug message.
        '''
        sample_time, clock, freq = self.clock_est
        return ("clocksync state: mcu_freq=%d last_clock=%d"
                " clock_est=(%.3f %d %.3f) min_half_rtt=%.6f min_rtt_time=%.3f"
                " time_avg=%.3f(%.3f) clock_avg=%.3f(%.3f)"
                " pred_variance=%.3f" % (
                    self.mcu_freq, self.last_clock, sample_time, clock, freq,
                    self.min_half_rtt, self.min_rtt_time,
                    self.time_avg, self.time_variance,
                    self.clock_avg, self.clock_covariance,
                    self.prediction_variance))
        
    def stats(self, eventtime):
        '''
        Prepare clock synchronization statistics, this is a virtual
        function for secondary MCUs.
        '''
        sample_time, clock, freq = self.clock_est
        return "freq=%d" % (freq,)
    
    def calibrate_clock(self, print_time, eventtime):
        '''
        Calibrate synchronization clock, this is a virtual
        function for secondary MCUs.
        '''
        return (0., self.mcu_freq)


class SecondarySync(ClockSync):
    '''
    Clock syncing code for secondary MCUs (whose clocks are synched to a
    primary MCU)
    '''
    def __init__(self, reactor, main_sync):
        ClockSync.__init__(self, reactor)
        self.main_sync = main_sync
        self.clock_adj = (0., 1.)
        self.last_sync_time = 0.
        
    def connect(self, serial, ethercat):
        '''
        Connect to serial and ethercat interfaces. There is no direct
        communication with the ethercat interface, however the internal
        ethercatqueue structure time needs to be updated through the
        ethercat class object. Take the main_sync (sync of main mcu) as
        reference and compensate according to the difference between
        main and local print time.
        '''
        ClockSync.connect(self, serial, ethercat)
        self.clock_adj = (0., self.mcu_freq)
        curtime = self.reactor.monotonic()
        main_print_time = self.main_sync.estimated_print_time(curtime)
        local_print_time = self.estimated_print_time(curtime)
        self.clock_adj = (main_print_time - local_print_time, self.mcu_freq)
        self.calibrate_clock(0., curtime)
        
    def connect_file(self, serial, ethercat, pace=False):
        '''
        Connect to serial and ethercat interfaces (debug mode).
        Take into account the different frequency between main
        and local mcu.
        '''
        ClockSync.connect_file(self, serial, ethercat, pace)
        self.clock_adj = (0., self.mcu_freq)
        
    def print_time_to_clock(self, print_time):
        '''
        Clock frequency conversions.
        '''
        adjusted_offset, adjusted_freq = self.clock_adj
        return int((print_time - adjusted_offset) * adjusted_freq)
    
    def clock_to_print_time(self, clock):
        '''
        Convert mcu clock to print time.
        '''
        adjusted_offset, adjusted_freq = self.clock_adj
        return clock / adjusted_freq + adjusted_offset
    
    def dump_debug(self):
        '''
        Prepare clock synchronization debug message.
        '''
        adjusted_offset, adjusted_freq = self.clock_adj
        return "%s clock_adj=(%.3f %.3f)" % (
            ClockSync.dump_debug(self), adjusted_offset, adjusted_freq)
        
    def stats(self, eventtime):
        '''
        Prepare clock synchronization statistics.
        '''
        adjusted_offset, adjusted_freq = self.clock_adj
        return "%s adj=%d" % (ClockSync.stats(self, eventtime), adjusted_freq)
    
    def calibrate_clock(self, print_time, eventtime):
        '''
        Calibrate synchronization clock.
        '''
        # calculate: est_print_time = main_sync.estimatated_print_time()
        ser_time, ser_clock, ser_freq = self.main_sync.clock_est
        main_mcu_freq = self.main_sync.mcu_freq
        est_main_clock = (eventtime - ser_time) * ser_freq + ser_clock
        est_print_time = est_main_clock / main_mcu_freq
        # determine sync1_print_time and sync2_print_time
        sync1_print_time = max(print_time, est_print_time)
        sync2_print_time = max(sync1_print_time + 4., self.last_sync_time,
                               print_time + 2.5 * (print_time - est_print_time))
        # calc sync2_sys_time (inverse of main_sync.estimatated_print_time)
        sync2_main_clock = sync2_print_time * main_mcu_freq
        sync2_sys_time = ser_time + (sync2_main_clock - ser_clock) / ser_freq
        # adjust freq so estimated print_time will match at sync2_print_time
        sync1_clock = self.print_time_to_clock(sync1_print_time)
        sync2_clock = self.get_clock(sync2_sys_time)
        adjusted_freq = ((sync2_clock - sync1_clock)
                         / (sync2_print_time - sync1_print_time))
        adjusted_offset = sync1_print_time - sync1_clock / adjusted_freq
        # apply new values
        self.clock_adj = (adjusted_offset, adjusted_freq)
        self.last_sync_time = sync2_print_time
        return self.clock_adj

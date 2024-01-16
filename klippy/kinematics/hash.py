# Code for handling the kinematics of cartesian robots
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import os, sys, logging
# add klippy dependency
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'klippy'))
import stepper, pvt_drive, toolhead

class HashKinematics:   
    def __init__(self, toolhead:toolhead.ToolHead, config):
        # get printer
        self.printer = config.get_printer()
        # setup axis rails
        self.dual_carriage_axis = None
        self.dual_carriage_rails = []
        # setup rail axis (pvt drive and stepper)        
        self.rails[0] = pvt_drive.LookupMultiRail(config.getsection('pvt_drive_x'))
        self.rails[1] = pvt_drive.LookupMultiRail(config.getsection('pvt_drive_y'))
        self.rails[2] = stepper.LookupMultiRail(config.getsection('stepper_z'))
        # setup kinematic solver
        self.rails[0].setup_itersolve('hash_drive_alloc', b'x')
        self.rails[1].setup_itersolve('hash_drive_alloc', b'y')
        self.rails[2].setup_itersolve('cartesian_stepper_alloc', b'z')
        # set trapezioidal queue and step generator for each stepper
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        # register motor shutdown handler
        self.printer.register_event_handler("stepper_enable:motor_off", self._motor_off)
        # setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel, above=0., maxval=max_accel)
        # same relative limits for all axes
        self.limits = [(1.0, -1.0)] * 3
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)
        # check for dual carriage support
        if config.has_section('dual_carriage'):
            dc_config = config.getsection('dual_carriage')
            # dual carrdge only on XY
            dc_axis = dc_config.getchoice('axis', {'x': 'x', 'y': 'y'})
            self.dual_carriage_axis = {'x': 0, 'y': 1}[dc_axis]
            dc_rail = pvt_drive.LookupMultiRail(dc_config)
            dc_rail.setup_itersolve('hash_drive_alloc', dc_axis.encode())
            for s in dc_rail.get_steppers():
                toolhead.register_step_generator(s.generate_steps)
            self.dual_carriage_rails = [self.rails[self.dual_carriage_axis], dc_rail]
            self.printer.lookup_object('gcode').register_command(
                'SET_DUAL_CARRIAGE', self.cmd_SET_DUAL_CARRIAGE,
                desc=self.cmd_SET_DUAL_CARRIAGE_help)
            
    def get_steppers(self):
        '''
        Get all printer steppers and drives.
        '''
        rails = self.rails
        if self.dual_carriage_axis is not None:
            dca = self.dual_carriage_axis
            rails = rails[:dca] + self.dual_carriage_rails + rails[dca+1:]
        return [s for rail in rails for s in rail.get_steppers()]
    
    def calc_position(self, stepper_positions):
        '''
        Get position of all steppers and drives.
        '''
        return [stepper_positions[rail.get_name()] for rail in self.rails]
    
    def set_position(self, newpos, homing_axes):
        '''
        Set printer position (each axis in masked with flag).
        '''
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()
                
    def note_z_not_homed(self):
        '''
        Helper for safe Z home.
        '''
        self.limits[2] = (1.0, -1.0)
        
    def _home_axis(self, homing_state, axis, rail):
        '''
        Determine movement and perform axis homing.
        '''
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # perform homing
        homing_state.home_rails([rail], forcepos, homepos)
        
    def home(self, homing_state):
        '''
        Perform printer homing (each axis is homed independently and in order).
        '''
        for axis in homing_state.get_axes():
            if axis == self.dual_carriage_axis:
                dc1, dc2 = self.dual_carriage_rails
                altc = self.rails[axis] == dc2
                self._activate_carriage(0)
                self._home_axis(homing_state, axis, dc1)
                self._activate_carriage(1)
                self._home_axis(homing_state, axis, dc2)
                self._activate_carriage(altc)
            else:
                self._home_axis(homing_state, axis, self.rails[axis])
                
    def _motor_off(self, print_time):
        '''
        Signal motor off.
        '''
        self.limits = [(1.0, -1.0)] * 3
        
    def _check_endstops(self, move):
        '''
        Check endstop position.
        '''
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
            
    def check_move(self, move):
        '''
        Check move axis and limits.
        '''
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # normal XY move (use defaults)
            return
        # move with Z (update velocity and accel for slower Z axis)
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
        
    def get_status(self, eventtime):
        '''
        Get homing status.
        '''
        axes = [a for a, (l, h) in zip("xyz", self.limits) if l <= h]
        return \
        {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }
        
    def _activate_carriage(self, carriage):
        '''
        Activate dual carridge.
        '''
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        dc_rail = self.dual_carriage_rails[carriage]
        dc_axis = self.dual_carriage_axis
        self.rails[dc_axis].set_trapq(None)
        dc_rail.set_trapq(toolhead.get_trapq())
        self.rails[dc_axis] = dc_rail
        pos = toolhead.get_position()
        pos[dc_axis] = dc_rail.get_commanded_position()
        toolhead.set_position(pos)
        if self.limits[dc_axis][0] <= self.limits[dc_axis][1]:
            self.limits[dc_axis] = dc_rail.get_range()
    
    def cmd_SET_DUAL_CARRIAGE(self, gcmd):
        '''
        Command to activate dual carridge.
        '''
        carriage = gcmd.get_int('CARRIAGE', minval=0, maxval=1)
        self._activate_carriage(carriage)
        
    '''
    Class attributes.
    '''
    cmd_SET_DUAL_CARRIAGE_help = "Set which carriage is active"

def load_kinematics(toolhead, config):
    return HashKinematics(toolhead, config)

import numpy as np
import random

class gait_pattern_generator():
    def __init__(self):
        self.state = 0
        self.t = 0
        self.t0 = 0
        self.t_transition = 0
        
        self.fl_leg = leg()
        self.fr_leg = leg()
        self.rl_leg = leg()
        self.rr_leg = leg()
        self.legs = [self.fl_leg,self.fr_leg,self.rl_leg,self.rr_leg]        
    
    # fixed time walk position pattern generator
    def ftw_position_generator(self):
        self.ftw_position_transitions()
        
        # for each leg, generate the desired controller commands
        for leg in self.legs:
            # empty out the command dict
            leg.command_dict = dict.fromkeys(leg.keys)
            # Init state
            if leg.state == 0:
                leg.command_dict['cmd_type'] = "position"
                leg.init_pos = [0.0,0.21]
                leg.command_dict['position'] = leg.init_pos
            
            # Flight state
            if leg.state == 1 and leg.transition == True:
                # Generate the desired position for the leg to stop in
                leg.command_dict['cmd_type'] = "position_trajectory"
                leg.command_dict['end_position'] = [.05,0.21]
                leg.command_dict['position_end_time'] = .3
                leg.command_dict['position_kp'] = 1.0
                leg.command_dict['position_kd'] = 0.0
                leg.transition = False
            elif leg.state == 1 and leg.transition == False:
                leg.command_dict['cmd_type'] = "none"
            
            # Stance state
            if leg.state == 2 and leg.transition == True: 
                leg.command_dict['cmd_type'] = "position_trajectory"
                leg.command_dict['end_position'] = [-.05,0.21]
                leg.command_dict['position_end_time'] = .1
                leg.command_dict['position_kp'] = 1.0
                leg.command_dict['position_kd'] = 0.0
                leg.transition = False
            elif leg.state == 2 and leg.transition == False:
                leg.command_dict['cmd_type'] = "none"

        return [leg.command_dict for leg in self.legs]
        
    def ftw_position_transitions(self):
        for leg in self.legs:
            # Start standing still, then transition to taking a step
            if leg.state == 0:
                if self.t > 5:
                    leg.state = 1
                    leg.transition = True
                    self.t_transition = self.t
            # Transition to stance
            if leg.state == 1:
                if self.t - self.t_transition > .3:
                    leg.state = 2
                    leg.transition = True
                    self.t_transition = self.t
            # Transition to flight
            #if leg.state == 2:
            #    if self.t - self.t_transition > .1:
            #        leg.state = 0
            #        leg.transition = True
            #        self.t_transition = self.t
            
# Leg class - used to store useful information about each leg     
class leg():
    def __init__(self):
        self.init_pos = [np.pi/4,np.pi/2]
        self.state = 0                                  # 0 = standing still, 1 = taking a step
        self.transition = False                         # True = the leg is currently in the transition regime (should not be True for more than one pass of the pattern generator)
        self.curr_pos = self.init_pos
        
        self.keys = ['cmd_type','position','end_position','position_end_time','position_kp','position_kd','total_impulse','impulse_end_time']
        self.command_dict = dict.fromkeys(self.keys)
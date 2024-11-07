# This message contains commands to a leg. 

string      cmd_type            # command type (position trajectory, impulse trajectory, etc)

# position control commands
float64[]   position            # desired position

# position trajectory control commands
float64[]   end_position        # desired end position [x,y]
float64     position_end_time   # the time at which the stance should be completed
float64     position_kp         # proportional gain for position control
float64     position_kd         # derivative gain for position control

# impulse control commands
float64     total_impulse       # the total impulse to be exerted in the stance phase
float64     impulse_end_time    # the time at which the stance should be completed

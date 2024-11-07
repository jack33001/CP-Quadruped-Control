# This message contains the robot's state.

# overarching robot values
float64[]   attitude            # attitude of the robot [roll, pitch, yaw]
float64[]   angular_velocity    # angular velocity of the robot [roll, pitch, yaw]
float64[]   linear_velocity     # linear velocity of the robot [x, y, z]

# leg values
# front left leg
float64[]   fl_position         # position of the front left leg [knee, hip]
float64[]   fl_velocity         # velocity of the front left leg [knee, hip]
float64[]   fl_torque           # torque of the front left leg [knee, hip]
bool        fl_contact          # contact state of the front left leg

# front right leg
float64[]   fr_position         # position of the front right leg [knee, hip]
float64[]   fr_velocity         # velocity of the front right leg [knee, hip]
float64[]   fr_torque           # torque of the front right leg [knee, hip]
bool        fr_contact          # contact state of the front right leg

# rear left leg
float64[]   rl_position         # position of the rear left leg [knee, hip]
float64[]   rl_velocity         # velocity of the rear left leg [knee, hip]
float64[]   rl_torque           # torque of the rear left leg [knee, hip]
bool        rl_contact          # contact state of the rear left leg

# rear right leg
float64[]   rr_position         # position of the rear right leg [knee, hip]
float64[]   rr_velocity         # velocity of the rear right leg [knee, hip]
float64[]   rr_torque           # torque of the rear right leg [knee, hip]
bool        rr_contact          # contact state of the rear right leg
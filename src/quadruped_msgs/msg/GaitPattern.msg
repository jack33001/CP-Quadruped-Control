# Desired foot impact locations
geometry_msgs/Vector3 foot1_step_position  # x, y, z in meters
geometry_msgs/Vector3 foot2_step_position
geometry_msgs/Vector3 foot3_step_position
geometry_msgs/Vector3 foot4_step_position

# Desired COM location
geometry_msgs/Vector3 com_position  # x, y, z in meters

# Phases for each foot (0.0 to 1.0)
float32 foot1_phase
float32 foot2_phase
float32 foot3_phase
float32 foot4_phase

# States for each foot
float32 foot1_state
float32 foot2_state
float32 foot3_state
float32 foot4_state
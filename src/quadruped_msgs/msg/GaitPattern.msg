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
int32 foot1_state
int32 foot2_state
int32 foot3_state
int32 foot4_state

# Step height parameter
float32 step_height

# Prediction arrays (flattened 2D arrays)
# Format: [foot1_stage1, foot1_stage2, ..., foot2_stage1, ..., foot4_stageN]
int32[] predicted_states
float32[] predicted_phases

# Prediction metadata
int32 prediction_stages
float32 prediction_horizon
float32 prediction_timestep
# Foot positions in cartesian coordinates
geometry_msgs/Point p1  # Front left foot position
geometry_msgs/Point p2  # Front right foot position
geometry_msgs/Point p3  # Rear left foot position
geometry_msgs/Point p4  # Rear right foot position
geometry_msgs/Point pc  # Center of mass position

geometry_msgs/Point h1  # Hip position 1
geometry_msgs/Point h2  # Hip position 2
geometry_msgs/Point h3  # Hip position 3
geometry_msgs/Point h4  # Hip position 4

# Joint states
float64[] joint_positions  # Joint positions (8)
float64[] joint_velocities # Joint velocities (8)
float64[] joint_efforts   # Joint efforts (8)

# IMU state
geometry_msgs/Quaternion orientation  # Body orientation
geometry_msgs/Vector3 angular_velocity  # Angular velocity

# Center of mass velocity
geometry_msgs/Vector3 com_velocity  # Center of mass velocity

# Jacobians (stored as row-major arrays)
float64[] j1  # Front left leg Jacobian (3x2=6 elements)
float64[] j2  # Front right leg Jacobian (3x2=6 elements)
float64[] j3  # Rear left leg Jacobian (3x2=6 elements)
float64[] j4  # Rear right leg Jacobian (3x2=6 elements)

# Contact states
bool contact_1  # Front left foot contact state
bool contact_2  # Front right foot contact state
bool contact_3  # Rear left foot contact state
bool contact_4  # Rear right foot contact state

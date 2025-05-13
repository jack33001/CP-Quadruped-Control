# Message to hold force commands for each foot
# Each foot has a 3D force vector (x, y, z)

# Force vectors (N) for each foot in the foot frame
# Positive z is pushing the foot down
geometry_msgs/Vector3 foot1_force  # Front left
geometry_msgs/Vector3 foot2_force  # Front right
geometry_msgs/Vector3 foot3_force  # Rear left
geometry_msgs/Vector3 foot4_force  # Rear right

# Sum of all foot forces
geometry_msgs/Vector3 total_force
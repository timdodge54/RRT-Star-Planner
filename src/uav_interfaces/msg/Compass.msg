# Provides a compass reading and associate variance
# A variance of 0 will be considered an unknown.
std_msgs/Header header
float64 angle # Angle in radians, positive rotation about z-axis
float64 variance # Standard deviation squared
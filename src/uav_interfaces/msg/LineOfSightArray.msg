# Provides line of sight information from the UAV to set features/buoys.
# The arrays are in the same order as they are in the buoy message.
# All features are included in this array, even if they are out of range of the UAV.

std_msgs/Header header
LineOfSight[]   los


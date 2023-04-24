########################## Message constants ############################
# Options for path type
string TYPE_STRAIGHT_LINE="straight_line" # No consideration of curvature
string TYPE_FILLET="fillet" # Arc-based fillet path
string TYPE_DUBINS="dubins" # Dubins-path

########################## Message fields ################################
# General parameters
string id # Unique ID for the waypoint path (could use unique_identifier_msgs, but had hard time with that package)
string type # Type of waypoint path
float64 min_radius # Minimum radius to be used for the path

# Points to be followed
uav_interfaces/UavWaypoint[] points # Vector of waypoint points

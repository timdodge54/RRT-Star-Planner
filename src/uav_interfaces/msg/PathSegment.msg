########################## Message constants ############################
# Options for the path type
string TYPE_LINE="line"
string TYPE_ORBIT="orbit"

# Options for orbit direction
string ORBIT_CW="CW"
string ORBIT_CCW="CCW"

########################## Message fields ################################
# General parameters
std_msgs/Header header
string type                             # type='line' means straight line following, type='orbit' means orbit following
float64 airspeed                        # desired airspeed along the path
geometry_msgs/Point origin              # origin of the straight path line (r) or the center of the orbit (c)

# Line specific parameters
geometry_msgs/Vector3 line_direction    # Unit vector pointing along direction of line (q)

# Orbit specific parameters
float64 orbit_radius                    # radius of the orbit (rho)
string orbit_direction                  # orbit direction: 'CW'==clockwise, 'CCW'==counter clockwise

# Message contains the following states for the UAV
#   north: float = north  # inertial north position in meters
#   east: float = east  # inertial east position in meters
#   altitude: float = altitude  # inertial altitude in meters
#   phi: float = phi  # roll angle in radians
#   theta: float = theta  # pitch angle in radians
#   psi: float = psi  # yaw angle in radians
#   Va: float = Va  # airspeed in meters/sec
#   alpha: float = alpha  # angle of attack in radians
#   beta: float = beta  # sideslip angle in radians
#   p: float = p  # roll rate in radians/sec
#   q: float = q  # pitch rate in radians/sec
#   r: float = r  # yaw rate in radians/sec
#   Vg: float = Vg  # groundspeed in meters/sec
#   gamma: float = gamma  # flight path angle in radians
#   chi: float = chi  # course angle in radians
#   bx: float = bx  # gyro bias along roll axis in radians/sec
#   by: float = by  # gyro bias along pitch axis in radians/sec
#   bz: float = bz  # gyro bias along yaw axis in radians/sec

#   wn: float = wn  # inertial windspeed in north direction in meters/sec
#   we: float = we  # inertial windspeed in east direction in meters/sec

# Position and orientation
#   position: (x,y,z) = (north,east,down) if defined in ned frame
#   orientation: quaternion of the vehicle
geometry_msgs/PoseStamped pose

# Velocities
#   linear: (x,y,z) = (u,v,w) if defined in body frame
#   angular: (x,y,z) = (p,q,r) if defined in body frame
geometry_msgs/TwistStamped twist

# Gyro bias
#   vector: (x,y,z) = (bx, by, bz) if defined in body frame
geometry_msgs/Vector3Stamped gyro_bias

# Attitude Euler angles
float64 phi     # roll angle in radians   (vehicle_2 -> body frame)
float64 theta   # pitch angle in radians  (vehicle_1 -> vehicle_2 frame)
float64 psi     # yaw angle in radians    (vehicle -> vehicle_1 frame)

# Wind variables
float64 w_n     # Wind in the north direction
float64 w_e     # Wind in the east direction
float64 v_a     # airspeed in meters/sec
float64 v_g     # groundspeed in meters/sec
float64 alpha   # angle of attack in radians    (body -> stability frame)
float64 beta    # sideslip angle in radians     (stability -> wind frame)
float64 gamma   # flight path angle in radians
float64 chi     # course angle in radians

"""planner_parameters.py defines the parameters used in the planner
"""
import mav_sim.parameters.aerosonde_parameters as MAV
import numpy as np

# size of the waypoint array used for the path planner.  This is the
# maximum number of waypoints that might be transmitted to the path
# manager.
size_waypoint_array: int = 100

# airspeed commanded by planner
Va0: float = MAV.u0

# max possible roll angle
phi_max: float = np.radians(25.0)

# minimum turn radius
R_min: float = Va0 ** 2.0 / MAV.gravity / np.tan(phi_max)
print(R_min)

# create random city map
city_width: float = 2000.0  # the city is of size (width)x(width)
building_height: float = 300.0  # maximum height of buildings
num_blocks: int = 5  # number of blocks in city
street_width: float = 0.8  # percent of block that is street.

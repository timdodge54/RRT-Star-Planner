from distutils.command.build import build
from importlib.resources import path
from multiprocessing.dummy import Value
import rclpy
from mav_sim.message_types.msg_state import MsgState
from mav_sim.message_types.msg_autopilot import MsgAutopilot
from mav_sim.message_types.msg_delta import MsgDelta
from mav_sim.message_types.msg_path import MsgPath
from mav_sim.message_types.msg_waypoints import MsgWaypoints
from mav_sim.message_types.msg_world_map import MsgWorldMap
from uav_interfaces.msg import UavState, AutoPilotCommands, ControlSurfaceCommands, PathSegment, UavWaypoints, UavWaypoint, BuildingWorld
import numpy as np
from mav_sim.tools import types
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from mav_sim.chap3.mav_dynamics import IND, DynamicState


################### Conversions from ROS to msg format ######################################
def state_ros_to_msg(state_in: UavState) -> MsgState:
    """ Creates the MsgState representation from the UavState representation
    """
    # Check the latest state frames
    if state_in.pose.header.frame_id != "ned":
        #TODO Should implement transform
        raise ValueError("State_in pose is not in ned frame, conversion cannot be made")

    if state_in.twist.header.frame_id != "body":
        #TODO Should implement transform
        raise ValueError("State_in twist is not in body frame, conversion cannot be made")

    state_out = MsgState()
    state_out.north = state_in.pose.pose.position.x      # Position
    state_out.east = state_in.pose.pose.position.y
    state_out.altitude = -state_in.pose.pose.position.z
    state_out.phi = state_in.phi                         # Attitude
    state_out.theta = state_in.theta
    state_out.psi = state_in.psi
    state_out.p = state_in.twist.twist.angular.x         # Angular rates
    state_out.q = state_in.twist.twist.angular.y
    state_out.r = state_in.twist.twist.angular.z
    state_out.wn = state_in.w_n                          # Wind parameters
    state_out.we = state_in.w_e
    state_out.Va = state_in.v_a
    state_out.alpha = state_in.alpha
    state_out.beta = state_in.beta
    state_out.Vg = state_in.v_g
    state_out.gamma = state_in.gamma
    state_out.chi = state_in.chi
    return state_out

def state_ros_to_vector(state_in: UavState) -> types.DynamicState:
    """ Creates the 13x1, quaternion version of the vector from the UavState representation
    """
    # Check the latest state frames
    if state_in.pose.header.frame_id != "ned":
        #TODO Should implement transform
        raise ValueError("state_in pose is not in ned frame, conversion cannot be made")

    if state_in.twist.header.frame_id != "body":
        #TODO Should implement transform
        raise ValueError("state_in twist is not in body frame, conversion cannot be made")

    state_out = np.zeros((13,1))
    state_out[IND.NORTH] = state_in.pose.pose.position.x        # Position
    state_out[IND.EAST] = state_in.pose.pose.position.y
    state_out[IND.DOWN] = state_in.pose.pose.position.z
    state_out[IND.U] = state_in.twist.twist.linear.x            # Linear velocity
    state_out[IND.V] = state_in.twist.twist.linear.y
    state_out[IND.W] = state_in.twist.twist.linear.z
    state_out[IND.E0] = state_in.pose.pose.orientation.w        # Attitude
    state_out[IND.E1] = state_in.pose.pose.orientation.x
    state_out[IND.E2] = state_in.pose.pose.orientation.y
    state_out[IND.E3] = state_in.pose.pose.orientation.z
    state_out[IND.P] = state_in.twist.twist.angular.x           # Angular rates
    state_out[IND.Q] = state_in.twist.twist.angular.y
    state_out[IND.R] = state_in.twist.twist.angular.z
    return state_out

def autopilot_ros_to_msg(cmd_in: AutoPilotCommands) -> MsgAutopilot:
    """ Creates the MsgAutopilot from the AutoPilotCommands representation
    """
    cmd_out = MsgAutopilot()
    cmd_out.airspeed_command = cmd_in.airspeed_command
    cmd_out.altitude_command = cmd_in.altitude_command
    cmd_out.course_command   = cmd_in.course_command
    cmd_out.phi_feedforward  = cmd_in.phi_feedforward
    return cmd_out

def path_ros_to_msg(path_in: PathSegment) -> MsgPath:
    """ Creates MsgPath from PathSegment. Raises a ValueError if invalid values are found.
    """
    # Check for valid values
    if path_in.header.frame_id != "ned":
        #TODO Should implement transform
        raise ValueError("Path segment is not in ned frame")

    if path_in.type != PathSegment.TYPE_LINE and path_in.type != PathSegment.TYPE_ORBIT:
        msg = "Incorrect path type, value should be in set {" + PathSegment.TYPE_LINE + ", " \
              + PathSegment.TYPE_ORBIT + "}"
        raise ValueError(msg)

    if path_in.airspeed <= 0.:
        raise ValueError("Airspeed must be positive")

    if path_in.type == PathSegment.TYPE_LINE:
        # A line must have a non-zero direction in north or east values
        if path_in.line_direction.x == 0. and path_in.line_direction.y == 0.:
            raise ValueError("Line must have nonzero entries for north or east directions")

    if path_in.type == PathSegment.TYPE_ORBIT:
        # An orbit must have a positive radius and the direction must be valid
        if path_in.orbit_radius <= 0:
            raise ValueError("Orbit radius must be positive")

        if path_in.orbit_direction != PathSegment.ORBIT_CW and \
            path_in.orbit_direction != PathSegment.ORBIT_CCW:
            raise ValueError("Orbit direction must be " + PathSegment.ORBIT_CW + \
                " or " + PathSegment.ORBIT_CCW)

    # Create the MsgPath version of the message
    path_out = MsgPath()
    path_out.type = path_in.type
    path_out.airspeed = path_in.airspeed
    path_out.line_origin = np.array([[path_in.origin.x], [path_in.origin.y], [path_in.origin.z]])
    path_out.line_direction = np.array([[path_in.line_direction.x], [path_in.line_direction.y], [path_in.line_direction.z]])
    path_out.orbit_center = np.array([[path_in.origin.x], [path_in.origin.y], [path_in.origin.z]])
    path_out.orbit_radius = path_in.orbit_radius
    path_out.orbit_direction = path_in.orbit_direction
    return path_out

def waypoints_ros_to_msg(waypnts_in: UavWaypoints) -> MsgWaypoints:
    """ Creates MsgWaypoints from UavWaypoints. Raises a Value Error if invalid values are found
    """
    # Check for valid type
    if  waypnts_in.type != UavWaypoints.TYPE_DUBINS and \
        waypnts_in.type != UavWaypoints.TYPE_FILLET and \
        waypnts_in.type != UavWaypoints.TYPE_STRAIGHT_LINE:
        raise ValueError("Invalid waypoints type: " + waypnts_in.type)

    # Check for valid radius
    if waypnts_in.min_radius <= 0.:
        raise ValueError("Min radius must be positive")

    # Extract waypoint
    waypnts_out = MsgWaypoints()
    waypnts_out.type = waypnts_in.type
    for waypnt in waypnts_in.points:
        # Check position
        if waypnt.position.header.frame_id != "ned":
            raise ValueError("Waypoints need to be defined in ned frame")

        # Check airspeed
        if waypnt.airspeed <= 0.:
            raise ValueError("Airspeed must be positive. Airspeed commanded: " + str(waypnt.airspeed))

        # Create the ned vector
        ned = np.array([[waypnt.position.point.x], [waypnt.position.point.y], [waypnt.position.point.z]])
        waypnts_out.add(ned=ned, airspeed=waypnt.airspeed, course=waypnt.course)
    return waypnts_out

def world_ros_to_msg(world_in: BuildingWorld) -> MsgWorldMap:
    """ Creates MsgWorldMap from BuildingWorld
    """
    # Extract the building_height
    counter = 0
    building_height = np.zeros((world_in.num_city_blocks, world_in.num_city_blocks))
    for north_ind in range(world_in.num_city_blocks):
        for east_ind in range(world_in.num_city_blocks):
            building_height[north_ind, east_ind] = world_in.building_height[counter]
            counter += 1

    # Create the
    world_out = MsgWorldMap(city_width=world_in.city_width, num_city_blocks=world_in.num_city_blocks, \
        building_max_height=world_in.building_max_height, street_width=world_in.street_width_val, \
        building_heights=building_height)

    return world_out

def convert_mav_state_to_msg(time: Time, msg_state: MsgState,
    dyn_state: DynamicState, velocity_scale: float) -> UavState:
        ''' Extracts the state from the mav and stores it in a UavState message

        Inputs:
            time: The time at which the data is being produced
            msg_state: The state data
            dyn_state: Containes quaternion and velocity data not in msg_state
            velocity_scale: Scale on the velocities. This allows the velocities to easily be zeroed out when
                            the sim is not moving
        Returns:
            The current state in the mav
        '''

        # Create initial message
        uav_state = UavState()

        # Position and orientation
        uav_state.pose.header.stamp = time.to_msg()
        uav_state.pose.header.frame_id = 'ned'
        uav_state.pose.pose.position.x = msg_state.north
        uav_state.pose.pose.position.y = msg_state.east
        uav_state.pose.pose.position.z = -msg_state.altitude
        uav_state.pose.pose.orientation.w = dyn_state.e0
        uav_state.pose.pose.orientation.x = dyn_state.e1
        uav_state.pose.pose.orientation.y = dyn_state.e2
        uav_state.pose.pose.orientation.z = dyn_state.e3
        uav_state.phi = msg_state.phi
        uav_state.theta = msg_state.theta
        uav_state.psi = msg_state.psi

        # Velocities
        uav_state.twist.header.stamp = time.to_msg()
        uav_state.twist.header.frame_id = 'body'
        uav_state.twist.twist.linear.x = dyn_state.u*velocity_scale
        uav_state.twist.twist.linear.y = dyn_state.v*velocity_scale
        uav_state.twist.twist.linear.z = dyn_state.w*velocity_scale
        uav_state.twist.twist.angular.x = msg_state.p*velocity_scale
        uav_state.twist.twist.angular.y = msg_state.q*velocity_scale
        uav_state.twist.twist.angular.z = msg_state.r*velocity_scale

        # Gyro bias
        uav_state.gyro_bias.header.stamp = time.to_msg()
        uav_state.gyro_bias.header.frame_id = 'body'
        uav_state.gyro_bias.vector.x = 0.
        uav_state.gyro_bias.vector.y = 0.
        uav_state.gyro_bias.vector.z = 0.

        # Wind variables
        uav_state.w_n = msg_state.wn
        uav_state.w_e = msg_state.we
        uav_state.v_a = msg_state.Va*velocity_scale
        uav_state.v_g = msg_state.Vg*velocity_scale
        uav_state.alpha = msg_state.alpha
        uav_state.beta = msg_state.beta
        uav_state.gamma = msg_state.gamma
        uav_state.chi = msg_state.chi

        return uav_state

################### Conversions from msg to ROS format ######################################
def ctrl_msg_to_ros(ctrl_in: MsgDelta) -> ControlSurfaceCommands:
    """ Creates the ControlSurfaceCommands message from MsgDelta
    """
    ctrl_out = ControlSurfaceCommands()
    ctrl_out.aileron =  ctrl_in.aileron
    ctrl_out.elevator = ctrl_in.elevator
    ctrl_out.rudder =   ctrl_in.rudder
    ctrl_out.throttle = ctrl_in.throttle
    return ctrl_out

def autopilot_msg_to_ros(cmd_in: MsgAutopilot) -> AutoPilotCommands:
    """ Creates the AutoPilotCommands from the MsgAutopilot representation
    """
    cmd_out = AutoPilotCommands()
    cmd_out.airspeed_command = cmd_in.airspeed_command
    cmd_out.altitude_command = cmd_in.altitude_command
    cmd_out.course_command   = cmd_in.course_command
    cmd_out.phi_feedforward  = cmd_in.phi_feedforward
    return cmd_out

def path_msg_to_ros(path_in: MsgPath, stamp: Time) -> PathSegment:
    """ Creates PathSegment from MsgPath.

    Args:
        path_in: path to be converted to ROS message
        stamp: Time stamp for publication

    Returns
        ROS path segment message
    """
    # Create the message header
    path_out = PathSegment()
    path_out.header.frame_id = "ned"
    path_out.header.stamp = stamp

    # Store general parameters
    path_out.type = path_in.type
    path_out.airspeed = path_in.airspeed
    if path_out.type == PathSegment.TYPE_LINE:
        path_out.origin.x = path_in.line_origin.item(0)
        path_out.origin.y = path_in.line_origin.item(1)
        path_out.origin.z = path_in.line_origin.item(2)
    else:
        path_out.origin.x = path_in.orbit_center.item(0)
        path_out.origin.y = path_in.orbit_center.item(1)
        path_out.origin.z = path_in.orbit_center.item(2)

    # Store line specific parameters
    path_out.line_direction.x = path_in.line_direction.item(0)
    path_out.line_direction.y = path_in.line_direction.item(1)
    path_out.line_direction.z = path_in.line_direction.item(2)

    # Store orbit specific parameters
    path_out.orbit_radius = float(path_in.orbit_radius)
    path_out.orbit_direction = path_in.orbit_direction

    return path_out

def waypoints_msg_to_ros(waypnts_in: MsgWaypoints, min_radius: float, id: str, stamp: Time) -> UavWaypoints:
    """ Creates a UavWaypoints from MsgWaypoints.

    Args:
        waypnts_in: Waypoint information
        min_radius: The minimum radius used in planning
        id: The identification of the plan (uuid)
        stamp: Time stamp for ned position creation

    Returns:
        Converged ros message
    """
    # Genearl waypoint parameters
    waypnts_out = UavWaypoints()
    waypnts_out.id = id
    waypnts_out.min_radius = min_radius
    waypnts_out.type = waypnts_in.type

    # Extract individual waypoints
    for k in range(waypnts_in.num_waypoints):
        # Get the waypoint from the message
        waypnt_msg = waypnts_in.get_waypoint(k)

        # Extract data for ros message
        waypnt_ros = UavWaypoint()
        waypnt_ros.course = 0.#TODO waypnt_msg.course
        waypnt_ros.airspeed = waypnt_msg.airspeed
        waypnt_ros.position.header.frame_id = "ned"
        waypnt_ros.position.header.stamp = stamp
        waypnt_ros.position.point.x = waypnt_msg.ned.item(0)
        waypnt_ros.position.point.y = waypnt_msg.ned.item(1)
        waypnt_ros.position.point.z = waypnt_msg.ned.item(2)

        # Add ros waypoint to waypoints
        waypnts_out.points.append(waypnt_ros)
    return waypnts_out

def world_msg_to_ros(world_in: MsgWorldMap, id: str) -> BuildingWorld:
    """ Creates BuildingWorld message from MsgWorldMap
    """
    world_out = BuildingWorld()
    world_out.uuid = id
    world_out.city_width = world_in.city_width
    world_out.num_city_blocks = world_in.num_city_blocks
    world_out.street_width_val = world_in.street_width_val
    world_out.building_max_height = world_in.building_max_height

    for north_ind in range(np.size(world_in.building_height,0)):
        for east_ind in range(np.size(world_in.building_height,1)):
            world_out.building_height.append(world_in.building_height[north_ind, east_ind])
    return world_out



################### Other conversions ######################################################
def create_ned_pose(vec: types.NP_MAT, stamp: Time) -> PoseStamped:
    """ Creates a PoseStamped message from a 3x1 np array assuming the "ned" frame

    Args:
        vec: np 3x1 array
        stamp: the time stamp to add to the pose

    Returns:
        The pose stamped message
    """
    # Create header
    pose = PoseStamped()
    pose.header.frame_id = "ned"
    pose.header.stamp = stamp

    # Copy over data and return result
    pose.pose.position.x = vec.item(0)
    pose.pose.position.y = vec.item(1)
    pose.pose.position.z = vec.item(2)
    return pose


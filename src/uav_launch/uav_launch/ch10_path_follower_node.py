from typing import Optional
import rclpy
from rclpy.node import Node
from uav_interfaces.msg import UavState, AutoPilotCommands, PathSegment
from mav_sim.chap10.path_follower import PathFollower
from mav_sim.message_types.msg_path import MsgPath
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray
from uav_launch.conversions import state_ros_to_msg, autopilot_msg_to_ros, path_ros_to_msg, create_ned_pose
from nav_msgs.msg import Path
import numpy as np

class PathFollowerNode(Node):
    """ Calculates the autopilot commands for a path to be followed

        Subscriptions:
            /uav_state_estimate: (uav_interfaces/msg/UavState) state of the mav
            /path_segment: (uav_interfaces/msg/Path) Path segment to be followed

        Publications:
            /autopilot_command: (uav_interfaces/msg/AutoPilotCommands) Commanded values for aircraft
            /path_segment_viz: (nav_msgs/Path) Segment of path to be followed
            /diagnostics: (diagnostic_msgs/DiagnosticArray) Diagnostics for the path follower
    """

    def __init__(self) -> None:
        """ Initializes the subscription and publication
        """

        # Initialize the node
        super().__init__(node_name="path_follower")

        # Create parameter for simulation time
        self._ts = 0.1
        self.declare_parameter(name="ts", value=self._ts, descriptor=ParameterDescriptor(description='Simulation period or time step'))
        self._ts = self.get_parameter("ts").value

        self._path_lookahead = 500.
        self.declare_parameter(name="plot_ahead", value=self._path_lookahead, descriptor=ParameterDescriptor(description='Look ahead for publishing path'))
        self._path_lookahead = self.get_parameter("plot_ahead").value

        self._orbit_res = 10.
        self.declare_parameter(name="orbit_res", value=self._orbit_res, descriptor=ParameterDescriptor(description='Resolution of published orbit path in meters'))
        self._orbit_res = self.get_parameter("orbit_res").value

        self.add_on_set_parameters_callback(self._param_callback)


        # Create the path follower
        self._path_follower = PathFollower()

        # Create the subscribers
        self._state_est: Optional[UavState] = None
        self._sub_state_est = self.create_subscription(UavState, "uav_state_estimate", self._state_estimate_callback, 1)
        self._path_segment: Optional[PathSegment] = None
        self._sub_path_segment = self.create_subscription(PathSegment, "path_segment", self._path_segment_callback, 1)

        # Create the publishers and loop
        self._pub_command = self.create_publisher(AutoPilotCommands, "autopilot_command", 1) # For control surface command
        self._pub_path_viz = self.create_publisher(Path, "path_segment_viz", 1) # For path visualizaiton
        self._main_timer = self.create_timer(self._ts, self._main_loop)

        # Create diagnostics variables
        self._pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 1)
        self._status = DiagnosticStatus() # Stores the current status of the path follower
        self._status.message = "Initialized"
        self._status.level = DiagnosticStatus.OK
        self._status.name = "Path Follower"
        self._status.hardware_id = "Sim"
        self._diagnostic_timer = self.create_timer(1., self._diagnostic_publishing)


    def _param_callback(self, params: list[rclpy.parameter.Parameter]) -> SetParametersResult:
        """ Set all of the parameters that are passed in
        """
        # Default the success as true
        successful = True

        # Loop through any parameters and set them accordingly
        for param in params:
            match param.name:
                # Process the simulation period
                case "ts":
                    # The simulation period must be greater than zero
                    if param.value > 0.:
                        # Only update the simulation period if it is new
                        if self._ts != param.value:
                            self._ts = param.value # Update the node value

                            # Recreate the timer
                            self.destroy_timer(self._main_timer)
                            self._main_timer = self.create_timer(self._ts, self._main_loop)
                    else:
                        successful = False
                case "plot_ahead":
                    if param.value >= 0.:
                        self._path_lookahead = param.value
                    else:
                        self._path_lookahead = 0.
                case "orbit_res":
                    if param.value > 0.:
                        self._orbit_res = param.value
                    else:
                        successful = False

        return SetParametersResult(successful=successful)

    def _path_segment_callback(self, msg: PathSegment) -> None:
        """ Stores the latest path segment
        """
        self._path_segment = msg

    def _state_estimate_callback(self, msg: UavState) -> None:
        """ Stores the latest state estimate
        """
        self._state_est = msg

    def _main_loop(self) -> None:
        """ Calculates and publishes the latest command from the autopilot.
        """
        # Do nothing if the incoming messages have not yet been received
        if self._state_est is None:
            #self.get_logger().warn("State estimate not yet recieved, autopilot command will not be produced")
            self._status.level = DiagnosticStatus.WARN
            self._status.message = "State estimate not yet recieved, autopilot command will not be produced"
            return
        if self._path_segment is None:
            #self.get_logger().warn("Path segment not yet recieved, autopilot command will not be produced")
            self._status.level = DiagnosticStatus.WARN
            self._status.message = "Path segment not yet recieved, autopilot command will not be produced"
            return

        # Extract saved data
        try:
            state = state_ros_to_msg(self._state_est)
            path_segment = path_ros_to_msg(self._path_segment)
        except ValueError as err:
            #self.get_logger().error("Not generating autopilot command due to invalid input message: " + str(err) )
            self._status.level = DiagnosticStatus.ERROR
            self._status.message = "Not generating autopilot command due to invalid input message: " + str(err)
            return

        # Calculate the autopilot command
        autopilot_commands = self._path_follower.update(path_segment, state)

        # Publish the autopilot command
        self._pub_command.publish( autopilot_msg_to_ros(autopilot_commands) )

        # Publish the path visualization
        if path_segment.type == PathSegment.TYPE_LINE:
            path = generate_line_path(path_segment, self._state_est, self._path_lookahead)
        else:
            path = generate_orbit_path(path_segment, self._state_est, self._path_lookahead, self._orbit_res)
        self._pub_path_viz.publish(path)

        self._status.level = DiagnosticStatus.OK
        self._status.message = "Autopilot command being generated"

    def _diagnostic_publishing(self) -> None:
        """Updates the diagnostic message
        """
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "path_follower"
        msg.status.append(self._status)
        self._pub_diag.publish(msg)

def generate_line_path(path_segment: MsgPath, state: UavState, path_ahead_dist: float) -> Path:
    """ Creates a nav_msg::Path message for the current state for a line

    Args:
        path_segment: The path to plot
        state: The starting state of the UAV
        path_ahead_dist: the distance ahead of the uav to plot

    Returns:
        path: The path to be published for plotting
    """

    # Ensure that the frame for the points is correct
    if state.pose.header.frame_id != "ned":
        raise ValueError("Pose frame must be ned")

    # Calculate the projection of the vehicle onto the line (this assumes that the state and path are in same frame)
    q = np.reshape(path_segment.line_origin, (3,1))
    u = np.reshape(path_segment.line_direction, (3,1)) / np.linalg.norm(path_segment.line_direction) # Unit vector for line direction
    p = np.array([[state.pose.pose.position.x], [state.pose.pose.position.y], [state.pose.pose.position.z]]) # Position of uav
    v = p-q # Vector from line origin to vehicle
    proj = q + u * (v.T @ u)

    # Create the path header
    path = Path()
    path.header.frame_id = "ned"
    path.header.stamp = state.pose.header.stamp

    # Place the projection point as the first point
    path.poses.append( create_ned_pose(proj, state.pose.header.stamp) )

    # Place the look-ahead point as the second point
    p_end = proj + path_ahead_dist * u
    path.poses.append( create_ned_pose(p_end, state.pose.header.stamp) )

    return path

def generate_orbit_path(path_segment: MsgPath, state: UavState, path_ahead_dist: float, orbit_res: float) -> Path:
    """ Creates a nav_msg::Path message for the current state for an orbit

    Args:
        path_segment: The path to plot
        state: The starting state of the UAV
        path_ahead_dist: the distance ahead of the uav to plot
        orbit_res: The resolution of the orbit path in meters

    Returns:
        path: The path to be published for plotting
    """

    # Ensure that the frame for the points is correct
    if state.pose.header.frame_id != "ned":
        raise ValueError("Pose frame must be ned")

    # Define point of interest with center at the origin
    p = np.array([[state.pose.pose.position.x], [state.pose.pose.position.y], [state.pose.pose.position.z]])
    p_c = p - path_segment.orbit_center

    # Calculate the angle around the orbit to p_c
    theta = np.arctan2(p_c.item(1), p_c.item(0))

    # Calculate the arc length and arc-length step
    gamma = path_ahead_dist/path_segment.orbit_radius
    gamma_step = orbit_res/path_segment.orbit_radius
    if path_segment.orbit_direction == PathSegment.ORBIT_CW:
        terminal_angle = theta+gamma+gamma_step
        continuation = lambda c: c < terminal_angle
    else:
        gamma_step = -gamma_step
        terminal_angle = theta-gamma-gamma_step
        continuation = lambda c: c > terminal_angle

    # Create the path header
    path = Path()
    path.header.frame_id = "ned"
    path.header.stamp = state.pose.header.stamp

    # Add points to the path
    gamma_k = theta
    while continuation(gamma_k):
        # Calculate point on orbit
        p_o = path_segment.orbit_center + path_segment.orbit_radius * np.array([[np.cos(gamma_k)],[np.sin(gamma_k)],[0.]])

        # Add point to path
        path.poses.append( create_ned_pose(p_o, state.pose.header.stamp))

        # Update gamma_k for next iteration
        gamma_k += gamma_step

    return path


def main(args=None):
    rclpy.init(args=args)

    # Create the autopilot node
    path_follower_node = PathFollowerNode()
    rclpy.spin(path_follower_node)

    # Shutdown the node
    path_follower_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

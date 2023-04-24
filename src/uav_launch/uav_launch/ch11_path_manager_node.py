from typing import Optional
import rclpy
from rclpy.node import Node
from uav_interfaces.msg import UavState, PathSegment, UavWaypoints
from mav_sim.chap11.path_manager import PathManager
from mav_sim.message_types.msg_waypoints import MsgWaypoints
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from uav_launch.conversions import state_ros_to_msg, path_msg_to_ros, waypoints_ros_to_msg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from mav_sim.parameters.planner_parameters import R_min
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray

class PathManagerNode(Node):
    """ Manages the path segments to be followed

        Subscriptions:
            /uav_state_estimate: (uav_interfaces/msg/UavState) state of the mav
            /waypoints: (uav_interfaces/msg/UavWaypoints) Waypoints to be followed

        Publications:
            /path_segment: (uav_interfaces/msg/Path) Path segment to be followed
            /waypoints_viz: (nav_msgs/Path) Waypoints used to produce the path
            /diagnostics: (diagnostic_msgs/DiagnosticArray) Diagnostics for the path manager
    """

    def __init__(self) -> None:
        """ Initializes the subscription and publication
        """

        # Initialize the node
        super().__init__(node_name="path_manager")

        # Create parameter for simulation time
        self._ts = 1.0
        self.declare_parameter(name="ts", value=self._ts, descriptor=ParameterDescriptor(description='Simulation period or time step'))
        self._ts = self.get_parameter("ts").value
        self.add_on_set_parameters_callback(self._param_callback)

        # Create the path follower
        self._path_manager = PathManager()

        # Create the subscribers
        self._state_est: Optional[UavState] = None
        self._sub_state_est = self.create_subscription(UavState, "uav_state_estimate", self._state_estimate_callback, 1)
        self._waypoints_msg: Optional[MsgWaypoints] = None
        self._waypoints_ros: Optional[UavWaypoints] = None
        self._waypoints_viz_published = False
        self._min_radius = R_min # Minimum radius for path generation
        self._sub_waypoints = self.create_subscription(UavWaypoints, "waypoints", self._waypoints_callback, 1)

        # Create the publishers and loop
        self._pub_path_seg = self.create_publisher(PathSegment, "path_segment", 1) # For path segment command
        self._pub_path_viz = self.create_publisher(Path, "path_viz", 1) # Path that the uav should follow
        self._pub_waypoints_viz = self.create_publisher(Path, "waypoints_viz", 1) # Waypoints used to create the path
        self._main_timer = self.create_timer(self._ts, self._main_loop)

        # Create diagnostics variables
        self._pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 1)
        self._status = DiagnosticStatus() # Stores the current status of the path follower
        self._status.message = "Initialized"
        self._status.level = DiagnosticStatus.OK
        self._status.name = "Path Manager"
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

        return SetParametersResult(successful=successful)

    def _waypoints_callback(self, msg: UavWaypoints) -> None:
        """ Converts to mav msg format and sets the waypoints in the manager
        """
        # Check to see if the waypoints match the previously published waypoints
        if self._waypoints_ros is not None:
            if self._waypoints_ros.id == msg.id:
                return

        # Extract the waypoints
        try:
            self._waypoints_msg = waypoints_ros_to_msg(msg)
            self._waypoints_ros = msg
            self._waypoints_viz_published = False # Indicate that the visualization needs to be republished
            self._min_radius = msg.min_radius

            # Update the waypoints in the path manager
            self._path_manager.set_waypoints(self._waypoints_msg)
        except ValueError as err:
            self.get_logger().error("Incorrect value, rejecting waypoints: " + str(err) )
            self._waypoints_msg = None
            self._waypoints_ros = None
            self._waypoints_viz_published = False



    def _state_estimate_callback(self, msg: UavState) -> None:
        """ Stores the latest state estimate
        """
        self._state_est = msg

    def _main_loop(self) -> None:
        """ Calculates and publishes the latest path segment to be followed
        """
        # Do nothing if the incoming messages have not yet been received
        if self._state_est is None:
            #self.get_logger().warn("State estimate not yet recieved, path segment will not be produced")
            self._status.level = DiagnosticStatus.WARN
            self._status.message = "State estimate not yet recieved, path segment will not be produced"
            return
        if self._waypoints_msg is None:
            #self.get_logger().warn("Waypoints not yet recieved, path segment will not be produced")
            self._status.level = DiagnosticStatus.WARN
            self._status.message = "Waypoints not yet recieved, path segment will not be produced"
            return

        # Extract saved data
        try:
            state = state_ros_to_msg(self._state_est)
        except ValueError as err:
            #self.get_logger().error("Not generating path segment command due to invalid input message: " + str(err) )
            self._status.level = DiagnosticStatus.ERROR
            self._status.message = "Not generating path segment command due to invalid input message: " + str(err)
            return

        # Calculate the path segment
        path_segment = self._path_manager.update(radius=self._min_radius, state=state)

        # Publish the path segment command
        self._pub_path_seg.publish( path_msg_to_ros(path_segment, self._state_est.pose.header.stamp) )

        # Publish the waypoints
        self._pub_waypoints_viz.publish( generate_waypoint_visualization(self._waypoints_ros))

        self._status.level = DiagnosticStatus.OK
        self._status.message = "Published path segment command"

    def _diagnostic_publishing(self) -> None:
        """Updates the diagnostic message
        """
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "path_manager"
        msg.status.append(self._status)
        self._pub_diag.publish(msg)


def generate_waypoint_visualization(waypoints: UavWaypoints) -> Path:
    """ Generate a path visualization from the waypoints passed in
    """
    path = Path()
    path.header.frame_id = "ned"

    for point in waypoints.points:
        pose = PoseStamped()
        pose.header = point.position.header
        pose.pose.position = point.position.point
        path.poses.append(pose)

    return path


def main(args=None):
    rclpy.init(args=args)

    # Create the autopilot node
    path_manager_node = PathManagerNode()
    rclpy.spin(path_manager_node)

    # Shutdown the node
    path_manager_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
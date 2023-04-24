from optparse import Option
from typing import Optional
import rclpy
from rclpy.node import Node
from uav_interfaces.msg import UavState, UavWaypoints, BuildingWorld
from mav_sim.chap12.path_planner import PathPlanner, PlannerType
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from uav_launch.conversions import state_ros_to_msg, waypoints_msg_to_ros, world_ros_to_msg
from geometry_msgs.msg import PoseStamped
from mav_sim.message_types.msg_world_map import MsgWorldMap, map_height
from mav_sim.parameters.planner_parameters import R_min
from rcl_interfaces.msg import IntegerRange
from std_srvs.srv import Empty
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray
import numpy as np
import uuid

class PathPlannerNode(Node):
    """ Creates plans from the latest position of the vehicle to the goal point

        Subscriptions:
            /uav_state_estimate: (uav_interfaces/msg/UavState) state of the mav
            /goal_pose: (geometry_msgs/msg/PoseStamped) goal pose for mav (note only position is used)
            /world: (uav_interfaces/msg/BuildingWorld) Message for communicating the buildings

        Publications:
            /waypoints: (uav_interfaces/msg/UavWaypoints) Waypoints to be followed
            /diagnostics: (diagnostic_msgs/DiagnosticArray) Diagnostics for the path planner

        Services:
            /replan: (Empty) Service indicating that planning should be performed
    """

    def __init__(self) -> None:
        """ Initializes the subscription, publication, and service server
        """

        # Initialize the node
        super().__init__(node_name="path_planner")

        # Create parameter for simulation time
        self._ts = 1.0
        self.declare_parameter(name="ts", value=self._ts, descriptor=ParameterDescriptor(description='Simulation period or time step'))
        self._ts = self.get_parameter("ts").value

        self._goal_altitude = 10.
        self.declare_parameter(name="goal_altitude", value=self._goal_altitude, descriptor=ParameterDescriptor(description='Altitude when goal defined in enu frame'))
        self._goal_altitude = self.get_parameter("goal_altitude").value

        self._planner_type = PlannerType.rrt_straight
        self.declare_parameter(name="planner_type", value=self._planner_type, descriptor= \
            ParameterDescriptor(description='Planner: 1 - simple_fillet, 2-simple_dubins, 3-rrt_straight, 4-rrt_dubins', \
            integer_range=[IntegerRange(from_value=1, to_value=4, step=1)]))
        self._planner_type = self.get_parameter("planner_type").value

        self.add_on_set_parameters_callback(self._param_callback)

        # Create the world and path planner
        self._new_plan_required = True
        self._path_planner = PathPlanner()

        # Create the subscribers
        self._state_est: Optional[UavState] = None
        self._sub_state_est = self.create_subscription(UavState, "uav_state_estimate", self._state_estimate_callback, 1)
        self._goal: Optional[PoseStamped] = None
        self._sub_goal = self.create_subscription(PoseStamped, "goal_pose", self._goal_callback, 1)
        self._world_map: Optional[MsgWorldMap] = None
        self._world_map_id = ""
        self._sub_world = self.create_subscription(BuildingWorld, "world", self._world_callback, 1)

        # Create the publisher
        self._waypoints: Optional[UavWaypoints] = None
        self._plan_uuid: str = ""
        self._pub_waypoints = self.create_publisher(UavWaypoints, "waypoints", 1) # Waypoints used to create the path
        self._main_timer = self.create_timer(self._ts, self._main_loop)

        # Create the replan service
        self._replan_srv = self.create_service(Empty, "replan", self._replan_callback)

        # Create diagnostics variables
        self._pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 1)
        self._status = DiagnosticStatus() # Stores the current status of the path follower
        self._status.message = "Initialized"
        self._status.level = DiagnosticStatus.OK
        self._status.name = "Path Planner"
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
                case "goal_altitude":
                    if self._goal_altitude != param.value:
                        self._goal_altitude = param.value
                        self._new_plan_required = True

                case "planner_type":
                    if param.value > 4 or param.value < 1:
                        successful = False
                    elif self._planner_type != param.value:
                        self._planner_type = param.value
                        self._new_plan_required = True

        return SetParametersResult(successful=successful)

    def _state_estimate_callback(self, msg: UavState) -> None:
        """ Stores the latest state estimate
        """
        self._state_est = msg

    def _goal_callback(self, msg: PoseStamped) -> None:
        """ Stores the goal if it is in the ned or enu frame
        """
        # Check for valid frame
        if msg.header.frame_id != "ned" and msg.header.frame_id != "enu":
            self.get_logger().error("Goal location ignored. Goal must be defined in ned or enu frame")

        # Store the goal location
        self._goal = msg
        self._new_plan_required = True

    def _replan_callback(self, req: Empty.Request, res:Empty.Response) -> Empty.Response:
        """Flips planning flag to true
        """
        self._new_plan_required = True
        return res

    def _world_callback(self, msg: BuildingWorld) -> None:
        """ Stores the new world message data
        """
        # Only update the world if the id is different
        if self._world_map_id == msg.uuid:
            return

        self._world_map = world_ros_to_msg(msg)

    def _main_loop(self) -> None:
        """ Calculates and publishes the latest path segment to be followed
        """
        # Do nothing if the incoming messages have not yet been received
        if self._state_est is None:
            #self.get_logger().warn("State estimate not yet recieved, plan will not be produced")
            self._status.level = DiagnosticStatus.WARN
            self._status.message = "State estimate not yet recieved, plan will not be produced"
            return
        if self._goal is None:
            #self.get_logger().warn("Goal not yet recieved, plan will not be produced")
            self._status.level = DiagnosticStatus.WARN
            self._status.message = "Goal not yet recieved, plan will not be produced"
            return
        if self._world_map is None:
            #self.get_logger().warn("World map not yet recieved, plan will not be produced")
            self._status.level = DiagnosticStatus.WARN
            self._status.message = "World map not yet recieved, plan will not be produced"
            return

        # Create the plan
        if self._new_plan_required:
            # Extract saved data
            try:
                state = state_ros_to_msg(self._state_est)
            except ValueError as err:
                msg = "Not generating plan command due to invalid state message: " + str(err)
                #self.get_logger().error(msg )
                self._status.level = DiagnosticStatus.ERROR
                self._status.message = msg
                return

            # Create the goal
            if self._goal.header.frame_id == "ned":
                goal = np.array([[self._goal.pose.position.x], [self._goal.pose.position.y], [self._goal.pose.position.z]])
            elif self._goal.header.frame_id == "enu":
                goal = np.array([[self._goal.pose.position.y], [self._goal.pose.position.x], [-self._goal_altitude]])
                goal[2] = -(map_height(world_map=self._world_map, point=goal) + self._goal_altitude)
            else:
                #self.get_logger().error("Invalid frame id for goal. frame_id = " + self._goal.header.frame_id +". Not planning.")
                self._status.level = DiagnosticStatus.ERROR
                self._status.message = "Invalid frame id for goal. frame_id = " + self._goal.header.frame_id +". Not planning."
                return

            # Create the plan
            try:
                self.get_logger().debug("New plan creation started")
                waypoints_msg = self._path_planner.update(world_map=self._world_map, state=state, planner_type=self._planner_type, end_pose_in=goal)
                self._plan_uuid = uuid.uuid4().hex
                self.get_logger().debug("New plan completed")
                self.get_logger().debug("plan = " + str(waypoints_msg))
            except ValueError as err:
                self.get_logger().error("Error occured during planning, plan will not be published: " + str(err))
                self._new_plan_required = False
                self._status.level = DiagnosticStatus.ERROR
                self._status.message = "Error occured during planning, plan will not be published: " + str(err)
                return

            # Convert the plan to a ros waypoints message
            self._waypoints = waypoints_msg_to_ros(waypnts_in=waypoints_msg, min_radius=R_min, id=self._plan_uuid, stamp=self.get_clock().now().to_msg())

            # Indicate that a new plan is no longer required
            self._new_plan_required = False

            # Set planning status
            self._status.level = DiagnosticStatus.OK
            self._status.message = "Waypoint plan generated"

        # Publish the path waypoints
        if self._waypoints is None:
            self.get_logger().error("Waypoint plan not yet generated")
            self._status.level = DiagnosticStatus.ERROR
            self._status.message = "Waypoint plan not yet generated"
        else:
            self._pub_waypoints.publish(self._waypoints)

    def _diagnostic_publishing(self) -> None:
        """Updates the diagnostic message
        """
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "path_planner"
        msg.status.append(self._status)
        self._pub_diag.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # Create the autopilot node
    path_planner_node = PathPlannerNode()
    rclpy.spin(path_planner_node)

    # Shutdown the node
    path_planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
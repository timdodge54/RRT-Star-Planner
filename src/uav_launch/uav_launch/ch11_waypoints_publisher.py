import rclpy
from rclpy.node import Node
from uav_interfaces.msg import UavWaypoints, UavWaypoint
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from uuid import uuid4

class WaypointsPublisher(Node):
    """ Publishes a series of waypoints for manually testing the PathManagerNode

        Subscriptions:
            None

        Publications:
            /waypoints: (uav_interfaces/msg/UavWaypoints) Waypoints to be followed
    """

    def __init__(self) -> None:
        """ Initializes the subscription and publication
        """

        # Initialize the node
        super().__init__(node_name="waypoint_publisher")

        # Create the path identifier
        self._uuid = uuid4()

        # Create parameter for simulation time
        self._ts = 0.1
        self.declare_parameter(name="ts", value=self._ts, descriptor=ParameterDescriptor(description='Simulation period or time step'))
        self._ts = self.get_parameter("ts").value
        self.add_on_set_parameters_callback(self._param_callback)

        # Create the publishers and loop
        self._pub_waypoints = self.create_publisher(UavWaypoints, "waypoints", 1)
        self._main_timer = self.create_timer(self._ts, self._main_loop)


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

    def _main_loop(self) -> None:
        """ Calculates and publishes the latest path segment to be followed
        """

        # Waypoints general variables
        waypoints = UavWaypoints()
        waypoints.type = UavWaypoints.TYPE_FILLET
        waypoints.min_radius = 150.
        waypoints.id = self._uuid.hex

        # Waypoint 0
        waypoint = UavWaypoint()
        waypoint.course = 0.
        waypoint.airspeed = 25.
        waypoint.position.header.frame_id = "ned"
        waypoint.position.header.stamp = self.get_clock().now().to_msg()
        waypoint.position.point.x = 0.
        waypoint.position.point.y = 0.
        waypoint.position.point.z = -100.
        waypoints.points.append(waypoint)

        # Waypoint 1
        waypoint = UavWaypoint()
        waypoint.course = 0.
        waypoint.airspeed = 25.
        waypoint.position.header.frame_id = "ned"
        waypoint.position.header.stamp = self.get_clock().now().to_msg()
        waypoint.position.point.x = 2000.
        waypoint.position.point.y = 0.
        waypoint.position.point.z = -100.
        waypoints.points.append(waypoint)

        # Waypoint 2
        waypoint = UavWaypoint()
        waypoint.course = 0.
        waypoint.airspeed = 25.
        waypoint.position.header.frame_id = "ned"
        waypoint.position.header.stamp = self.get_clock().now().to_msg()
        waypoint.position.point.x = 2000.
        waypoint.position.point.y = 2000.
        waypoint.position.point.z = -100.
        waypoints.points.append(waypoint)

        # Waypoint 3
        waypoint = UavWaypoint()
        waypoint.course = 0.
        waypoint.airspeed = 25.
        waypoint.position.header.frame_id = "ned"
        waypoint.position.header.stamp = self.get_clock().now().to_msg()
        waypoint.position.point.x = -1000.
        waypoint.position.point.y = 2000.
        waypoint.position.point.z = -100.
        waypoints.points.append(waypoint)

        # Waypoint 4
        waypoint = UavWaypoint()
        waypoint.course = 0.
        waypoint.airspeed = 25.
        waypoint.position.header.frame_id = "ned"
        waypoint.position.header.stamp = self.get_clock().now().to_msg()
        waypoint.position.point.x = -0.
        waypoint.position.point.y = 0.
        waypoint.position.point.z = -100.
        waypoints.points.append(waypoint)

        # Publish the autopilot command
        self._pub_waypoints.publish(waypoints)


def main(args=None):
    rclpy.init(args=args)

    # Create the autopilot node
    waypoint_node = WaypointsPublisher()
    rclpy.spin(waypoint_node)

    # Shutdown the node
    waypoint_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from uav_interfaces.msg import UavWaypoints, UavWaypoint
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from uuid import uuid4

class WaypointsRelay(Node):
    """ Listens to a nav_msgs/path message, converts it to a UavWaypoints message, and
        republishes it

        Subscriptions:
            waypoints_in: (nav_msgs/msg/Path) Waypoints produced externally

        Publications:
            waypoints_out: (uav_interfaces/msg/UavWaypoints) Waypoints to be followed
    """

    def __init__(self) -> None:
        """ Initializes the subscription and publication
        """

        # Initialize the node
        super().__init__(node_name="waypoint_relay")

        # Create the subscriber
        self._sub_path = self.create_subscription(Path, "waypoints_in", self._path_callback, 1)

        # Create the publisher
        self._pub_waypoints = self.create_publisher(UavWaypoints, "waypoints_out", 1)


    def _path_callback(self, msg: Path) -> None:
        """ Converts the incoming path to a UavWaypoints and publishes the resulting message
        """
        # Waypoints general variables
        waypoints = UavWaypoints()
        waypoints.type = UavWaypoints.TYPE_FILLET
        waypoints.min_radius = 150.
        waypoints.id = uuid4().hex

        # Add a waypoint for each waypoint in Path
        for pose in msg.poses:
            waypoint = UavWaypoint()
            waypoint.course = 0.
            waypoint.airspeed = 25.
            waypoint.position.header.frame_id = pose.header.frame_id
            waypoint.position.header.stamp = pose.header.stamp
            waypoint.position.point.x = pose.pose.position.x
            waypoint.position.point.y = pose.pose.position.y
            waypoint.position.point.z = pose.pose.position.z
            waypoints.points.append(waypoint)

        # Publish the autopilot command
        self._pub_waypoints.publish(waypoints)


def main(args=None):
    """ Create the autopilot node, spin, and then shutdown
    """
    rclpy.init(args=args)

    # Create the autopilot node
    waypoint_node = WaypointsRelay()
    rclpy.spin(waypoint_node)

    # Shutdown the node
    waypoint_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

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

class DiagnosticsAggregatorNode(Node):
    """ Calculates the autopilot commands for a path to be followed

        Subscriptions:
            /diagnostics: (diagnostic_msgs/DiagnosticArray) Diagnostics for any of the systems

        Publications:
            /diagnostics_agg: (diagnostic_msgs/DiagnosticArray) Diagnostics for any of the systems
    """

    def __init__(self) -> None:
        """ Initializes the subscription and publication
        """

        # Initialize the node
        super().__init__(node_name="simple_diagnostic_aggregator")

        # Create the subscriber
        self._diagnostics: dict[str, DiagnosticArray] = {}
        self._sub_diagnostics = self.create_subscription(DiagnosticArray, "diagnostics", self._diagnostics_callback, 1)

        # Create the publishers and loop
        self._pub_diagnostics = self.create_publisher(DiagnosticArray, "diagnostics_agg", 1)
        self._main_timer = self.create_timer(1., self._main_loop)


    def _diagnostics_callback(self, msg: DiagnosticArray) -> None:
        """ Stores the diagnostics data
        """
        self._diagnostics[msg.header.frame_id] = msg

    def _main_loop(self) -> None:
        """ Calculates and publishes the latest command from the autopilot.
        """
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        for diagnostics in self._diagnostics.values():
            for status in diagnostics.status:
                msg.status.append(status)
        self._pub_diagnostics.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    # Create the diagnostics aggregator node
    diagnostics_agg_node = DiagnosticsAggregatorNode()
    rclpy.spin(diagnostics_agg_node)

    # Shutdown the node
    diagnostics_agg_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
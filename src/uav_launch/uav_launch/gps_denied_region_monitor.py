from geometry_msgs.msg import Point as GeomPoint
from mav_sim.chap3.mav_dynamics import IND
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from shapely.geometry import Point, Polygon
from std_msgs.msg import Bool
from typing import Optional
from uav_interfaces.msg import UavState
from uav_launch.conversions import state_ros_to_vector
from visualization_msgs.msg import Marker, MarkerArray

class GpsMonitor():
    """ Monitors the uav to determine whether or not it is in a gps denied region.

    Subscriptions:
        /uav_state: (uav_interfaces/msg/UavState) state of the mav

    Publications
        /gps_denied: (std_msgs/msg/Bool) true => operating in GPS denied region, false => gps should not be denied
        /gps_denied_regions: (visualization_msgs/msg/MarkerArray) Drawing of the gps denied region
    """

    def __init__(self, node: Node) -> None:
        """ Initializes the parameters, publications, and subscriptions
        """
        # Store input parameters
        self._node = node
        self._gps_denied_polys: list[Polygon] = [] # 2D polygons defining north-east regions where gps is denied
        self._ts = 10. # Period for checking the gps denied area
        self.declare_parameters()

        # Subscribe to the state message
        self._state: Optional[UavState] = None
        self._sub_state = self._node.create_subscription(UavState, "uav_state", self._state_callback, 1)

        # Create the gps_denied publisher
        self._pub_denied = self._node.create_publisher(Bool, "gps_denied", 1)

        # Create the gps denied visualization message
        self._gps_denied_viz: MarkerArray = MarkerArray()
        self._create_viz_msg()
        self._pub_gps_denied = self._node.create_publisher(MarkerArray, "gps_denied_regions", 1)

        # Create the denied navigation loop
        self._gps_eval_timer = self._node.create_timer(self._ts, self._gps_denied_eval)
        self._gps_viz_timer = self._node.create_timer(60., self._pub_gps_denied_viz)
        self._pub_gps_denied_viz()

    def declare_parameters(self) -> None:
        """Declare and read in parameters"""
        # Read in the sampling time
        self._node.declare_parameter(name="ts", value=self._ts,
            descriptor=ParameterDescriptor(description="Period at which gps denial should be evaluated"))
        self._ts = self._node.get_parameter(name="ts").value

        # GPS denied region parameters
        self._node.declare_parameter(name="num_gps_denied_regions", value=0)
        num_regions = self._node.get_parameter(name="num_gps_denied_regions").value
        for k in range(num_regions):
            # Get the (x,y) coordinates
            self._node.declare_parameter(name='gps_denied_'+ str(k)+'_x', value=[0.,])
            self._node.declare_parameter(name='gps_denied_'+ str(k)+'_y', value=[0.,])
            x_vals = self._node.get_parameter(name='gps_denied_'+ str(k)+'_x').value
            y_vals = self._node.get_parameter(name='gps_denied_'+ str(k)+'_y').value

            # Form the coordinate tuples
            coord_tuples: list[tuple[float, float]] = []
            for (x,y) in zip(x_vals, y_vals):
                coord_tuples.append((x,y))

            # Create the polygon
            poly = Polygon(coord_tuples)
            self._gps_denied_polys.append(poly)

    def _state_callback(self, msg: UavState) -> None:
        """ Stores the latest state
        """
        self._state = msg

    def _create_viz_msg(self) -> None:
        """ Create a marker message for each polygon
        """
        self._gps_denied_viz = MarkerArray()
        count = 0 # Keeps the count of the current polygon
        scale = 10.
        latest_time = self._node.get_clock().now().to_msg()
        for poly in self._gps_denied_polys:
            # Create the new marker
            line = Marker()
            line.header.frame_id = "ned"
            line.header.stamp = latest_time
            line.ns = "gps_denied"
            line.id = count
            line.type = Marker.LINE_STRIP
            line.action = 0
            line.scale.x = scale
            line.scale.y = 1. #scale
            line.scale.z = 1. #scale
            line.color.r = 1.
            line.color.a = 1.0
            #line.lifetime =  0
            line.frame_locked = True
            count += 1

            # Add points
            for coord in poly.exterior.coords:
                point =GeomPoint()
                point.x = coord[0]
                point.y = coord[1]
                line.points.append(point)

            # Add the line to the marker array
            self._gps_denied_viz.markers.append(line)

    def _pub_gps_denied_viz(self) -> None:
        """ publishes the gps denied region points
        """
        # Update the marker time values
        latest_time = self._node.get_clock().now().to_msg()
        for marker in self._gps_denied_viz.markers:
            marker.header.stamp = latest_time

        # Publish the gps denied region
        self._pub_gps_denied.publish(self._gps_denied_viz)

    def _gps_denied_eval(self) -> None:
        """Evaluate the current state of the uav to determine if it is in a gps denied region
        """
        # Extract the state
        if self._state is None:
            return
        state = state_ros_to_vector(self._state)

        # Check to see if uav in the gps denied region, exit if in gps denied region
        in_denied = Bool()
        in_denied.data = False
        point = Point( (state[IND.NORTH], state[IND.EAST]))
        for poly in self._gps_denied_polys:
            if poly.contains(point):
                in_denied.data = True
                break

        # Publish whether it is in the denied region
        self._pub_denied.publish(in_denied)

def main(args=None):
    rclpy.init(args=args)

    # Create the autopilot node
    denied_node = rclpy.create_node(node_name="gps_denied_monitor")
    monitor = GpsMonitor(node=denied_node)
    rclpy.spin(denied_node)

    # Shutdown the node
    denied_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


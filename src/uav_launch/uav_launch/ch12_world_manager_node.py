import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from uav_interfaces.msg import BuildingWorld
from uav_launch.conversions import world_msg_to_ros
from geometry_msgs.msg import Point
from mav_sim.message_types.msg_world_map import MsgWorldMap
from std_srvs.srv import Empty
import uuid
from visualization_msgs.msg import MarkerArray, Marker

class WorldManagerNode(Node):
    """ Creates and maintains the world used for planning

        Publications:
            /world_viz: (visualization_msgs/msg/MarkerArray) Visualization message for each of the buildings
            /world: (uav_interfaces/msg/BuildingWorld) Message for communicating the buildings

        Services:
            /regen_world: (Empty) Regenerates the world
    """

    def __init__(self) -> None:
        """ Initializes the subscription, publication, and service server
        """

        # Initialize the node
        super().__init__(node_name="world_manager")

        # Create parameter for simulation time
        self._ts = 1.0
        self.declare_parameter(name="ts", value=self._ts, descriptor=ParameterDescriptor(description='Simulation period or time step'))
        self._ts = self.get_parameter("ts").value
        self.add_on_set_parameters_callback(self._param_callback)

        # Create the world and path planner
        self._world_map = MsgWorldMap()
        self._world_msg: BuildingWorld = world_msg_to_ros(self._world_map, uuid.uuid4().hex)

        # Create the publishers
        self._pub_world_viz = self.create_publisher(MarkerArray, "world_viz", 1)
        self._pub_world = self.create_publisher(BuildingWorld, "world", 1)
        self._main_timer = self.create_timer(self._ts, self._main_loop)

        # Create regen service
        self._regen_srv = self.create_service(Empty, "regen_world", self._regen_world_callback)

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

    def _regen_world_callback(self, req: Empty.Request, res:Empty.Response) -> Empty.Response:
        """Regenerates the world
        """
        self._world_map = MsgWorldMap()
        self._world_msg = world_msg_to_ros(self._world_map, uuid.uuid4().hex)
        return res

    def _main_loop(self) -> None:
        """ Calculates and publishes the latest path segment to be followed
        """
        # Create the marker visualization message
        msg = MarkerArray()

        # Loop through all of the buildings and add them to the marker array
        counter = 0
        for north_ind in range(self._world_map.num_city_blocks):
            for east_ind in range(self._world_map.num_city_blocks):
                # Create the generic parameters
                bldg = Marker()
                bldg.header.frame_id = "enu"
                bldg.header.stamp = self.get_clock().now().to_msg()
                bldg.ns = "world"
                bldg.type = Marker.CUBE
                bldg.action = 0 # Add/modify
                bldg.color.a = 1.
                bldg.color.r = 1.
                bldg.color.g = 0.
                bldg.color.b = 0.
                bldg.frame_locked = True
                bldg.scale.x = self._world_map.building_width
                bldg.scale.y = self._world_map.building_width


                # Building label
                bldg.text = "building_" + str(counter)
                bldg.id = counter

                # Building height
                bldg.scale.z = self._world_map.building_height[north_ind,east_ind]

                # Building position (recall enu frame has x-axis as east and y-axis as north)
                #                   (recall that the position is in the center of each dimension, thus way each dimension has a + val/2)
                bldg.pose.position.x = self._world_map.building_east.item(east_ind) + self._world_map.building_width/2.
                bldg.pose.position.y = self._world_map.building_north.item(north_ind) + self._world_map.building_width/2.
                bldg.pose.position.z = bldg.scale.z/2.


                msg.markers.append(bldg)
                counter += 1

        # Add the region boundary
        bound = Marker()
        bound.header.frame_id = "enu"
        bound.header.stamp = self.get_clock().now().to_msg()
        bound.ns = "boundary"
        bound.type = Marker.LINE_STRIP
        bound.action = 0 # Add/modify
        bound.color.a = 1.
        bound.color.r = 0.
        bound.color.g = 1.
        bound.color.b = 0.
        bound.frame_locked = True
        bound.scale.x = 10.
        bound.text = "World boundary"
        bound.id = counter
        point0 = Point()                # Origin
        point0.x = 0.
        point0.y = 0.
        point0.z = 0.
        bound.points.append(point0)
        point1 = Point()                # East corner
        point1.x = self._world_map.city_width
        point1.y = 0.
        bound.points.append(point1)
        point2 = Point()                # North-East corner
        point2.x = self._world_map.city_width
        point2.y = self._world_map.city_width
        bound.points.append(point2)
        point3 = Point()                # North corner
        point3.x = 0.
        point3.y = self._world_map.city_width
        bound.points.append(point3)
        bound.points.append(point0)     # Complete loop
        msg.markers.append(bound)


        # Publish the messages
        self._pub_world_viz.publish(msg)
        self._pub_world.publish(self._world_msg)


def main(args=None):
    rclpy.init(args=args)

    # Create the autopilot node
    world_manager_node = WorldManagerNode()
    rclpy.spin(world_manager_node)

    # Shutdown the node
    world_manager_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
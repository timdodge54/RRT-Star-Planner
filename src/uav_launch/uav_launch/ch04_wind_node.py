import rclpy
from rclpy.node import Node
from uav_interfaces.msg import WindVector
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from mav_sim.chap4.wind_simulation import WindSimulation


class WindNode(Node):
    """ Produces state updates based upon the current state, the low-level commands, and the wind

        Subscriptions:
            None

        Publications:
            /wind: (uav_interfaces/msg/WindVector) Steady-state and gust wind vectors

        Services:
            None
    """
    def __init__(self) -> None:
        """ Initializes the publications and subscriptions
        """

        # Initialize the node
        super().__init__(node_name="wind_node")

        # Create parameter for simulation time
        self._ts = 0.01
        self.declare_parameter(name="ts", value=self._ts, descriptor=ParameterDescriptor(description='Simulation period or time step'))
        self._ts = self.get_parameter("ts").value
        self._gust_flag = True
        self.declare_parameter(name="use_gust", value=self._gust_flag, descriptor=ParameterDescriptor(description='False turns off gust'))
        self.add_on_set_parameters_callback(self.param_callback)
        self._gust_flag = self.get_parameter("use_gust")

        # Initialize the wind
        self._wind = WindSimulation(Ts=self._ts, gust_flag=self._gust_flag)

        # Create the publishers
        self._pub_wind = self.create_publisher(WindVector, "wind", 1)

        # Create the main loop callback timer
        self._main_timer = self.create_timer(self._ts, self.main_loop)

    def param_callback(self, params: list[rclpy.parameter.Parameter]) -> SetParametersResult:
        """ Set all of the parameters that are passed in
        """
        # Default the success as true
        successful = True

        # Loop through any parameters and set them accordingly
        recreate_wind = False
        for param in params:
            match param.name:
                # Process the simulation period
                case "ts":
                    # The simulation period must be greater than zero
                    if param.value > 0.:
                        # Only update the simulation period if it is new
                        if self._ts != param.value:
                            self._ts = param.value # Update the node value
                            self._mav.ts_simulation = self._ts # Update the mav value
                            recreate_wind = True # Indicate that the wind must be recreated

                            # Recreate the timer
                            self.destroy_timer(self._main_timer)
                            self._main_timer = self.create_timer(self._ts, self.main_loop)
                    else:
                        successful = False
                case "use_gust":
                    if self._gust_flag != param.value:
                        self._gust_flag = param.value
                        recreate_wind = True # Indicate that the wind must be recreated

        if recreate_wind:
            self._wind = WindSimulation(Ts=self._ts, gust_flag=self._gust_flag)

        return SetParametersResult(successful=successful)

    def main_loop(self) -> None:
        """Executes the main loop of the node
        """
        # Update the wind
        wind_vec = self._wind.update()

        # Publish the updated state
        wind_msg = WindVector()
        time_stamp = self.get_clock().now()
        wind_msg.steady_state.header.stamp = time_stamp.to_msg() # Stead state
        wind_msg.steady_state.header.frame_id = "ned"
        wind_msg.steady_state.vector.x = wind_vec.item(0)
        wind_msg.steady_state.vector.y = wind_vec.item(1)
        wind_msg.steady_state.vector.z = wind_vec.item(2)

        wind_msg.gust.header.stamp = time_stamp.to_msg() # Stead state
        wind_msg.gust.header.frame_id = "body"
        wind_msg.gust.vector.x = wind_vec.item(3)
        wind_msg.gust.vector.y = wind_vec.item(4)
        wind_msg.gust.vector.z = wind_vec.item(5)

        self._pub_wind.publish(wind_msg)

def main(args=None):
    rclpy.init(args=args)

    # Create the kinematic node
    wind_node = WindNode()
    rclpy.spin(wind_node)

    # Shutdown the node
    wind_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

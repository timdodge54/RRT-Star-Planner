from argon2 import Type
from pytest import param
import rclpy
from rclpy.node import Node
from uav_interfaces.msg import UavState
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult

import numpy as np

class StateReconfigure(Node):
    """ This class publishes a fixed state. The uav state is reconfigurable via parameters

    """
    def __init__(self) -> None:
        """ Initializes the subscription to the uav state
        """

        # Initialize the node
        super().__init__(node_name="state_reconfigure")
        self.get_logger().info("Starting the node")

        # Declare the parameters
        self.declare_parameter(name='x', value=0.,      descriptor=ParameterDescriptor(description='North position'))
        self.declare_parameter(name='y', value=0.,      descriptor=ParameterDescriptor(description='East position'))
        self.declare_parameter(name='z', value=0.,      descriptor=ParameterDescriptor(description='Down position'))
        self.declare_parameter(name='phi', value=0.,    descriptor=ParameterDescriptor(description='Roll'))
        self.declare_parameter(name='theta', value=0.,  descriptor=ParameterDescriptor(description='Pitch'))
        self.declare_parameter(name='psi', value=0.,    descriptor=ParameterDescriptor(description='Yaw'))
        self.declare_parameter(name='alpha', value=0.,  descriptor=ParameterDescriptor(description='Angle of attack'))
        self.declare_parameter(name='beta', value=0.,   descriptor=ParameterDescriptor(description='Side slip'))

        # Read the parameters
        self._state = UavState()
        self._state.pose.pose.position.x = self.get_parameter(name='x').value
        self._state.pose.pose.position.y = self.get_parameter(name='y').value
        self._state.pose.pose.position.z = self.get_parameter(name='z').value
        self._state.phi = self.get_parameter(name='phi').value
        self._state.theta = self.get_parameter(name='theta').value
        self._state.psi = self.get_parameter(name='psi').value
        self._state.alpha = self.get_parameter(name='alpha').value
        self._state.beta = self.get_parameter(name='beta').value

        # Create a publisher
        self._pub_state = self.create_publisher(UavState, "uav_state", 1)

        # Create a periodic loop for publishing the state
        timer_period = .05  # seconds
        self.timer = self.create_timer(timer_period, self.state_publisher)

        # Create parameter callback
        self.add_on_set_parameters_callback(self.param_callback)

    def state_publisher(self) -> None:
        """ Publishes the state of the UAV given the stored parameters
        """

        # Set time
        self._state.pose.header.stamp = self.get_clock().now().to_msg()

        # Publish state
        self._pub_state.publish(self._state)


    def param_callback(self, params: list[rclpy.parameter.Parameter]) -> SetParametersResult:
        """ Set all of the parameters that are passed in
        """
        # Loop through all of the parameters and store the corresponding value
        for param in params:
            match param.name:
                case "x":
                    self._state.pose.pose.position.x = param.value
                case "y":
                    self._state.pose.pose.position.y = param.value
                case "z":
                    self._state.pose.pose.position.z = param.value
                case "phi":
                    self._state.phi = param.value
                case "theta":
                    self._state.theta = param.value
                case "psi":
                    self._state.psi = param.value
                case "alpha":
                    self._state.alpha = param.value
                case "beta":
                    self._state.beta = param.value

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)

    # Create the transform
    state_reconfigure = StateReconfigure()
    rclpy.spin(state_reconfigure)


if __name__ == '__main__':
    main()

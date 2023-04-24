from typing import Optional

from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray
from geometry_msgs.msg import WrenchStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node, Timer
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Empty
from uav_interfaces.msg import UavState, UavStateWrench, ControlSurfaceCommands, WindVector
from uav_interfaces.srv import SetUavState
from uav_launch.dynamics_interface import DynamicsData, update_state, publish_state_wrench
from uav_launch.clock_node import run_clock_node

class DynamicRosInterface:
    """ Produces state updates based upon the current state, the low-level commands, and the wind

        Subscriptions:
            /command: (uav_interfaces/msg/ControlSurfaceCommands) Control surface commands for the MAV
            /wind: (uav_interfaces/msg/WindVector) Steady-state and gust wind vectors

        Publications:
            /uav_state: (uav_interfaces/msg/UavState) state of the mav
            /forces_moments: (geometry_msgs/msg/WrenchStamped)
            /diagnostics: (diagnostic_msgs/DiagnosticArray) Diagnostics for the dynamics

        Services:
            /reset_state: (Empty) Resets the state to the initial state
            /set_state: (SetUavState) Sets the initial state to the desired value
            /toggle_execution: (Empty) Toggles the execution on and off
    """
    def __init__(self, data: DynamicsData, node: Node) -> None:
        """ Initializes the publications and subscriptions

        Inputs:
            data: Class for storing the data required for the dynamics
            node: Node used to create ros interfaces
        """

        # Store passed in data
        self._node = node
        self._data = data

        # Create the publishers
        self._pub_state = self._node.create_publisher(UavState, "uav_state", 1)               # State of the aircraft
        self._pub_fm = self._node.create_publisher(WrenchStamped, "forces_moments", 1)        # Resulting forces and moments
        self._pub_state_fm = self._node.create_publisher(UavStateWrench, "state_wrench", 1)   # State and resulting force in one message

        # Create the subscibers
        self.sub_command = self._node.create_subscription(ControlSurfaceCommands, "command", self._data.command_callback, 1)
        self.sub_wind = self._node.create_subscription(WindVector, "wind", self._data.wind_callback, 1)

        # Create the state services
        self._reset_state_srv = self._node.create_service(Empty, 'reset_state', self._data.reset_state_callback)
        self._set_state_srv = self._node.create_service(SetUavState, 'set_state', self._data.set_state_callback)
        self._toggle_srv = self._node.create_service(Empty, 'toggle_execution', self._data.toggle_callback)

        # Create diagnostics variables
        self._pub_diag = self._node.create_publisher(DiagnosticArray, "/diagnostics", 1)
        self._status = DiagnosticStatus() # Stores the current status of the path follower
        self._status.message = "Initialized"
        self._status.level = DiagnosticStatus.OK
        self._status.name = "Dynamics"
        self._status.hardware_id = "Sim"
        self._diagnostic_timer: Optional[Timer] = None

    def declare_parameters(self):
        """Declares and reads in a parameter"""
        self._node.declare_parameter(name="ts", value=self._data.ts, descriptor=ParameterDescriptor(description='Simulation period or time step'))
        self.set_time_step(self._node.get_parameter("ts").value)

    def create_timers(self) -> None:
        """Creates the timer loops. Separate from the __init__ funciton to allow nodes to utilize different timers"""
        self._diagnostic_timer = self._node.create_timer(1., self._diagnostic_publishing)

    def set_time_step(self, ts: float) -> bool:
        """Sets the time step for the simulation"""
        if ts <= 0.:
            self._node.get_logger().error("Attempting to set a non-positive time step")
            return False
        self._data.ts = ts # Update the node value
        self._data.mav._ts_simulation = self._data.ts # Update the mav sim value

        return True

    def param_callback(self, params: list[rclpy.parameter.Parameter]) -> SetParametersResult:
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
                        if self._data.ts != param.value:
                            self.set_time_step(param.value)
                    else:
                        successful = False


        return SetParametersResult(successful=successful)

    def publish_state_wrench(self) -> UavStateWrench:
        """ Publishes the state and wrench messages
        """
        return publish_state_wrench(data=self._data, pub_state = self._pub_state, pub_wrench=self._pub_fm, pub_combined=self._pub_state_fm)

    def take_step(self) -> Time:
        """Executes the main loop of the node
        """
        # Get the time increment and wind for simulation
        self._data.extract_wind()
        time_delta = Duration(seconds=self._data.ts)

        # Update the state
        update_state(data=self._data, delta=self._data.command, time_step=self._data.ts)

        # Update the sim time and the next time update
        self._data.time_latest += time_delta

        # Publish the resulting state and wrench data
        self.publish_state_wrench()

        # Set the diagnostic data
        if self._data.executing:
            self._status.level = DiagnosticStatus.OK
            self._status.message = "Calculating dynamic updates"
        else:
            self._status.level = DiagnosticStatus.WARN
            self._status.message = "Waiting for a call to the toggle_execution service"

        return self._data.time_latest

    def _diagnostic_publishing(self) -> None:
        """Updates the diagnostic message
        """
        msg = DiagnosticArray()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "dynamics"
        msg.status.append(self._status)
        self._pub_diag.publish(msg)



def main(args=None):
    def interface_constructor(ts: float, time_latest: Time, node: Node) \
        -> tuple[DynamicRosInterface, DynamicsData]:
        """Creates the ros interface
        """
        dynamics_data = DynamicsData(ts=ts, time_latest=time_latest, node=node )
        ros_interface = DynamicRosInterface(data=dynamics_data, node=node)
        ros_interface.declare_parameters()
        node.add_on_set_parameters_callback(ros_interface.param_callback)
        ros_interface.create_timers()

        return (ros_interface, dynamics_data)

    run_clock_node(node_name="clock_dyn_node", get_ros_interface=interface_constructor)

if __name__ == '__main__':
    main()

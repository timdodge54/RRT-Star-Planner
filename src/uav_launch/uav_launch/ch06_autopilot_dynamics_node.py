from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node, Timer
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from typing import Optional
from uav_launch.ch04_dynamic_mav_node import DynamicRosInterface
from uav_launch.ch06_autopilot_node import AutopilotInterface
from uav_launch.dynamics_interface import DynamicsData
from uav_launch.dynamics_interface import DynamicsData, update_state
from uav_launch.clock_node import run_clock_node

class AutoPilotDynamics:
    """Runs both the autopilot and the dynamics in the same node to allow for use of
       latest state information. The state estimate for the autopilot is not used. Instead,
       the truth state of the uav is employed for control.

       Subscriptions:
            /autopilot_command: (uav_interfaces/msg/AutoPilotCommands) Commanded values for aircraft
            /uav_state_estimate: (uav_interfaces/msg/UavState) state of the mav
            /command: (uav_interfaces/msg/ControlSurfaceCommands) Control surface commands for the MAV
            /wind: (uav_interfaces/msg/WindVector) Steady-state and gust wind vectors

        Publications:
            /command: (uav_interfaces/msg/ControlSurfaceCommands) Control surface commands for the MAV
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

        # Initialize the interface
        self._node = node
        self._data = data
        self._autopilot = AutopilotInterface(node=node)
        self._dynamics = DynamicRosInterface(data=self._data, node=node)

        # Create diagnostics variables
        self._pub_diag = self._node.create_publisher(DiagnosticArray, "/diagnostics", 1)
        self._status = DiagnosticStatus() # Stores the current status of the path follower
        self._status.message = "Initialized"
        self._status.level = DiagnosticStatus.OK
        self._status.name = "Autopilot_Dynamics"
        self._status.hardware_id = "Sim"
        self._diagnostic_timer: Optional[Timer] = None

        # Declare the time step parameter
        self._node.declare_parameter(name="ts", value=self._data.ts, descriptor=ParameterDescriptor(description='Simulation period or time step'))
        ts = self._node.get_parameter("ts").value
        self._autopilot.set_time_step(ts)
        self._dynamics.set_time_step(ts)

        # Create timers
        self._diagnostic_timer = self._node.create_timer(1., self._diagnostic_publishing)

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
                            ts = param.value
                            successful = successful and self._autopilot.set_time_step(ts)
                            successful = successful and self._dynamics.set_time_step(ts)

                    else:
                        successful = False


        return SetParametersResult(successful=successful)

    def take_step(self) -> Time:
        """Simulate through and calculate the control and dynamics update
           and return the updated time
        """
        # Get the time increment and wind for simulation
        self._data.extract_wind()
        time_delta = Duration(seconds=self._data.ts)

        # Create the control command
        delta = self._autopilot.calculate_autopilot(state=self._data.mav.true_state)

        # Update the state
        update_state(data=self._data, delta=delta, time_step=self._data.ts)

        # Update the sim time and the next time update
        self._data.time_latest += time_delta

        # Publish the resulting state and wrench data
        self._dynamics.publish_state_wrench()
        self._autopilot.publish_command(delta)


        # Set the diagnostic data
        if self._data.executing:
            self._status.level = DiagnosticStatus.OK
            self._status.message = "Dynamics updated with true state"
        else:
            self._status.level = DiagnosticStatus.WARN
            self._status.message = "Waiting for a call to the toggle_execution service"

        return self._data.time_latest

    def _diagnostic_publishing(self) -> None:
        """Updates the diagnostic message
        """
        msg = DiagnosticArray()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "autopilot_dynamics"
        msg.status.append(self._status)
        self._pub_diag.publish(msg)

def main(args=None):
    def interface_constructor(ts: float, time_latest: Time, node: Node) \
        -> tuple[AutoPilotDynamics, DynamicsData]:
        """Creates the ros interface
        """
        dynamics_data = DynamicsData(ts=ts, time_latest=time_latest, node=node )
        ros_interface = AutoPilotDynamics(data=dynamics_data, node=node)
        node.add_on_set_parameters_callback(ros_interface.param_callback)

        return (ros_interface, dynamics_data)

    run_clock_node(node_name="clock_dyn_autopilot_node", get_ros_interface=interface_constructor)

if __name__ == '__main__':
    main()

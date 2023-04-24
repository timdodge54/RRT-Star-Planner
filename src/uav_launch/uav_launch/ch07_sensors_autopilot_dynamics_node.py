from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray
from mav_sim.message_types.msg_delta import MsgDelta
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node, Timer
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
from typing import Optional
from uav_launch.ch04_dynamic_mav_node import DynamicRosInterface
from uav_launch.ch06_autopilot_node import AutopilotInterface
from uav_launch.ch07_sensors_node import SensorParameters, SensorsInterface, create_imu_measurement
from uav_launch.conversions import state_ros_to_msg
from uav_launch.dynamics_interface import DynamicsData, update_state
from uav_launch.clock_node import run_clock_node

class SensorsAutopilotDynamics:
    """Runs both the autopilot and the dynamics in the same node to allow for use of
       latest state information. The state estimate for the autopilot is not used. Instead,
       the truth state of the uav is employed for control.

       Subscriptions:
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

    def __init__(self, data: DynamicsData, sensor_params: SensorParameters, node: Node) -> None:
        """ Initializes the publications and subscriptions

        Inputs:
            data: Class for storing the data required for the dynamics
            sensor_params: Parameters for producing sensor measurements
            node: Node used to create ros interfaces
        """
        # Initialize the interface
        self._node = node
        self._data = data
        self._sp = sensor_params
        self._sensors = SensorsInterface(node=self._node, sensor_parameters=self._sp)
        self._autopilot = AutopilotInterface(node=self._node)
        self._dynamics = DynamicRosInterface(data=self._data, node=self._node)

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
        self._sp.imu_period = ts

        # Declare the true/nav state
        self._node.declare_parameter(name="use_nav_state", value=False, descriptor=ParameterDescriptor(description='True=>use navigation state for sim, False=> use true state'))
        self._use_nav_state = self._node.get_parameter("use_nav_state").value

        # Create timers
        self._diagnostic_timer = self._node.create_timer(1., self._diagnostic_publishing)

    def param_callback(self, params: list[rclpy.parameter.Parameter]) -> SetParametersResult:
        """ Set all of the parameters that are passed in
        """
        # Default the success as true
        successful = True

        # Loop through any parameters and set them accordingly
        for param in params:
            # Process the simulation period
            if param.name == "ts":
                # The simulation period must be greater than zero
                if param.value > 0.:
                    # Only update the simulation period if it is new
                    if self._data.ts != param.value:
                        ts = param.value
                        successful = successful and self._autopilot.set_time_step(ts)
                        successful = successful and self._dynamics.set_time_step(ts)
                        self._sp.imu_period = ts
                else:
                    successful = False

            # Ignore the imu_period as the imu will be processed at the same rate as the dynamics
            elif param.name == "imu_period":
                self._node.get_logger().error("Attempting to set imu period, but this configuration requires setting ts")

            # Read in whether to use the navigation state
            elif param.name == "use_nav_state":
                self._use_nav_state = param.value

            # All other parameters are assumed to belong to the sensor interface
            else:
                successful = successful and self._sensors.process_param(param=param)
        return SetParametersResult(successful=successful)

    def take_step(self) -> Time:
        """Calculate the control and dynamics and return the updated time
        """
        # Get the time increment and wind for simulation
        self._data.extract_wind()
        time_delta = Duration(seconds=self._data.ts)

        # Calculate the control
        state_ready = True # Flag used to indicate if the navigation state is ready
        if self._use_nav_state:
            state_ros = self._autopilot.state_est

            # Calculate control only if there is a valid state
            if state_ros is None:
                state_ready = False
                delta = MsgDelta()
            else:
                state = state_ros_to_msg(state_in=state_ros)
                delta = self._autopilot.calculate_autopilot(state=state)

        else: # => use the true state for control calculation
            # Create the control command
            state = self._data.mav.true_state
            delta = self._autopilot.calculate_autopilot(state=state)

        # Update the state
        update_state(data=self._data, delta=delta, time_step=self._data.ts)
        self._data.time_latest += time_delta

        # Create sensor measurements
        state = self._data.mav.get_state()
        forces = self._data.mav._forces
        psi = self._data.mav.true_state.psi
        v_a = self._data.mav.true_state.Va
        sm = create_imu_measurement(sp=self._sp, state=state, psi=psi, v_a=v_a, forces=forces)

        # Publish the sensor measurements
        self._sensors.publish_imu(sm=sm, time=self._data.time_latest.to_msg())

        # Publish the control comamnd
        self._autopilot.publish_command(delta)

        # Publish the resulting state and wrench data
        state_wrench = self._dynamics.publish_state_wrench()
        self._sensors.state_callback(state_wrench) # Update the latest state/wrench in sensors for gps
        self._autopilot.publish_command(delta)

        # Set the diagnostic data
        if not state_ready:
            self._status.level = DiagnosticStatus.WARN
            self._status.message = "No state estimate, control not calculated"

        if self._data.executing:
            self._status.level = DiagnosticStatus.OK
            if self._use_nav_state:
                self._status.message = "Dynamics updated with nav state"
            else:
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
        msg.header.frame_id = "sensor_auto_dyn"
        msg.status.append(self._status)
        self._pub_diag.publish(msg)

def main(args=None):
    def interface_constructor(ts: float, time_latest: Time, node: Node) \
        -> tuple[SensorsAutopilotDynamics, DynamicsData]:
        """Creates the ros interface
        """
        dynamics_data = DynamicsData(ts=ts, time_latest=time_latest, node=node )
        sensor_params = SensorParameters()
        ros_interface = SensorsAutopilotDynamics(data=dynamics_data,
            sensor_params=sensor_params, node=node)
        node.add_on_set_parameters_callback(ros_interface.param_callback)

        return (ros_interface, dynamics_data)

    run_clock_node(node_name="clock_sense_dyn_autopilot_node", get_ros_interface=interface_constructor)


if __name__ == '__main__':
    main()

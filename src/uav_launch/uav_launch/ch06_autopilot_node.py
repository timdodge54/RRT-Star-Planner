from typing import Optional

from mav_sim.message_types.msg_state import MsgState
import rclpy
from rclpy.node import Node, Timer
from uav_interfaces.msg import UavState, ControlSurfaceCommands, AutoPilotCommands
from mav_sim.chap6.autopilot import Autopilot
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from uav_launch.conversions import state_ros_to_msg, autopilot_ros_to_msg, ctrl_msg_to_ros
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray
from mav_sim.message_types.msg_delta import MsgDelta

class AutopilotInterface:
    """ Calculates the control for the uav to follow a desired airspeed, course command, and altitude with
        knowledge of a feedforward roll (phi) angle

        Subscriptions:
            /autopilot_command: (uav_interfaces/msg/AutoPilotCommands) Commanded values for aircraft
            /uav_state_estimate: (uav_interfaces/msg/UavState) state of the mav

        Publications:
            /command: (uav_interfaces/msg/ControlSurfaceCommands) Control surface commands for the MAV
            /diagnostics: (diagnostic_msgs/DiagnosticArray) Diagnostics for the autopilot

    """

    def __init__(self, node: Node) -> None:
        """ Initializes the subscription and publication

        Inputs:
            node: node used to create the subscriptions, publications, and timers
        """

        # Store the node
        self._node = node

        # Create the subscribers
        self._sub_autopilot_cmd = self._node.create_subscription(AutoPilotCommands, "autopilot_command", self._autopilot_command_callback, 1)
        self._sub_state_est = self._node.create_subscription(UavState, "uav_state_estimate", self._state_estimate_callback, 1)

        # Create the default autopilot command
        self._autopilot_cmd = AutoPilotCommands()
        self._autopilot_cmd.airspeed_command = 25.
        self._autopilot_cmd.altitude_command = 100.
        self._autopilot_cmd.course_command = 0.
        self._autopilot_cmd.phi_feedforward = 0.

        # Create the control surface command publisher and loop
        self.state_est: Optional[UavState] = None
        self._pub_command = self._node.create_publisher(ControlSurfaceCommands, "command", 1)
        self._main_timer: Optional[Timer] = None

        # Create diagnostics variables
        self._pub_diag = self._node.create_publisher(DiagnosticArray, "/diagnostics", 1)
        self._status = DiagnosticStatus() # Stores the current status of the path follower
        self._status.message = "Initialized"
        self._status.level = DiagnosticStatus.OK
        self._status.name = "Autopilot"
        self._status.hardware_id = "Sim"
        self._diagnostic_timer: Optional[Timer] = None

        # Create parameter for simulation time
        self._ts = 0.01
        self._autopilot = Autopilot(self._ts)

    def declare_parameters(self) -> None:
        """Declares the time step parameter"""
        self._node.declare_parameter(name="ts", value=self._ts, descriptor=ParameterDescriptor(description='Simulation period or time step'))
        self.set_time_step(self._node.get_parameter("ts").value)

    def create_timers(self) -> None:
        """Creates the main and diagnostic timers"""
        self._main_timer = self._node.create_timer(self._ts, self._main_loop)
        self._diagnostic_timer = self._node.create_timer(1., self._diagnostic_publishing)

    def state_estimate_received(self) -> bool:
        """Returns true if ready, false otherwise. If not ready it sets the status message to indicate
           such.
        """
        if self.state_est is None:
            #self.get_logger().warn("State estimate not yet recieved, autopilot will not produce command")
            self._status.level = DiagnosticStatus.WARN
            self._status.message = "State estimate not yet recieved, autopilot will not produce command"
            return False
        return True

    def set_time_step(self, ts: float) -> bool:
        """Sets the time step for the simulation"""
        if ts <= 0.:
            self._node.get_logger().error("Attempting to set a non-positive time step")
            return False

        # Store the time step
        self._ts = ts
        self._autopilot = Autopilot(self._ts)

        # Recreate the timer
        if self._main_timer is not None:
            self._node.destroy_timer(self._main_timer)
            self._main_timer = self._node.create_timer(self._ts, self._main_loop)

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
                        if self._ts != param.value:
                            self.set_time_step(param.value)
                    else:
                        successful = False


        return SetParametersResult(successful=successful)

    def publish_command(self, cmd: MsgDelta) -> None:
        """Publishes the autopilot command"""
        # Publish the autopilot command
        delta_msg = ctrl_msg_to_ros(cmd)
        self._pub_command.publish(delta_msg)


    def _autopilot_command_callback(self, msg: AutoPilotCommands) -> None:
        """ Stores the latest autopilot command
        """
        self._autopilot_cmd = msg

    def _state_estimate_callback(self, msg: UavState) -> None:
        """ Stores the latest state estimate
        """
        self.state_est = msg

    def _main_loop(self) -> None:
        """ Calculates and publishes the latest command from the autopilot.
        """
        # Do nothing if the data is not yet ready
        if not self.state_estimate_received():
            return

        # Calculate the autopilot command
        state = state_ros_to_msg(state_in=self.state_est)
        delta = self.calculate_autopilot(state=state)
        self.publish_command(delta)

        # Set the status
        self._status.level = DiagnosticStatus.OK
        self._status.message = "Generating autopilot commands"

    def calculate_autopilot(self, state: MsgState) -> MsgDelta:
        """Calculates the autopilot command given an estimated state"""
        # Ingest the recorded command and input state
        cmd = autopilot_ros_to_msg(self._autopilot_cmd)

        # Calculate the autopilot command
        delta, _ = self._autopilot.update(cmd=cmd, state=state)

        # Return the command
        return delta

    def _diagnostic_publishing(self) -> None:
        """Updates the diagnostic message
        """
        msg = DiagnosticArray()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "autopilot"
        msg.status.append(self._status)
        self._pub_diag.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    # Create the autopilot node
    autopilot_node = rclpy.create_node(node_name="autopilot_node")
    autopilot_interface = AutopilotInterface(autopilot_node)
    autopilot_interface.declare_parameters()
    autopilot_node.add_on_set_parameters_callback(autopilot_interface.param_callback)
    autopilot_interface.create_timers()
    rclpy.spin(autopilot_node)

    # Shutdown the node
    autopilot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

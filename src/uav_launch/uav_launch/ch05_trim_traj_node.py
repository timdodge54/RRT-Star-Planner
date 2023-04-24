import rclpy
import numpy as np
from rclpy.node import Node
from uav_interfaces.msg import UavState, ControlSurfaceCommands
from uav_interfaces.srv import SetUavState
from mav_sim.chap3.mav_dynamics import DynamicState
from mav_sim.chap5.trim import compute_trim
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray

class TrimTrajNode(Node):
    """ Calculates trim and produces trim trajectory inputs

        Subscriptions:
            None

        Publications:
            /command: (uav_interfaces/msg/ControlSurfaceCommands) Control surface commands for the MAV
            /diagnostics: (diagnostic_msgs/DiagnosticArray) Diagnostics for the path planner
    """

    def __init__(self) -> None:
        """ Initializes the parameters and command publication
        """

        # Initialize the node
        super().__init__(node_name="trim_traj_node")

        # Create parameters
        self._ts = 0.01     # Simulation time / publication time
        self.declare_parameter(name="ts", value=self._ts, descriptor=ParameterDescriptor(description='Simulation period or time step'))
        self._ts = self.get_parameter("ts").value

        self._va_trim = 35.  # Trim airspeed
        self.declare_parameter(name="va_trim", value=self._va_trim, descriptor=ParameterDescriptor(description='Airspeed for trim trajectory'))
        self._va_trim = self.get_parameter("va_trim").value

        self._gamma_trim = 0. # Trim flight path angle
        self.declare_parameter(name="gamma_trim", value=self._gamma_trim, descriptor=ParameterDescriptor(description='Flight path angle for trim trajectory'))
        self._gamma_trim = self.get_parameter("gamma_trim").value

        self._radius_trim =  -1. # Trim radius
        self.declare_parameter(name="radius_trim", value=self._radius_trim, descriptor=ParameterDescriptor(description='Radius of trim trajectory'))
        self._radius_trim = self.get_parameter("radius_trim").value

        self.add_on_set_parameters_callback(self._param_callback)

        # Initialize the command publication
        (uav_trim_state, self._command) = self._calculate_trim()
        self._pub_command = self.create_publisher(ControlSurfaceCommands, "command", 1)

        # Create the main publication loop
        self._main_timer = self.create_timer(self._ts, self._main_loop)



        # Create diagnostics variables
        self._pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 1)
        self._status = DiagnosticStatus() # Stores the current status of the path follower
        self._status.message = "Initialized"
        self._status.level = DiagnosticStatus.OK
        self._status.name = "Path Planner"
        self._status.hardware_id = "Sim"
        self._diagnostic_timer = self.create_timer(1., self._diagnostic_publishing)

        # Set the uav trim state
        self._reset_trim_state(uav_trim_state=uav_trim_state)

    def _param_callback(self, params: list[rclpy.parameter.Parameter]) -> SetParametersResult:
        """Processes any changes in parameters and updates the main loop and trim state commands accordingly
        """
        # Initialize flags
        successful = True # Default the success as true
        recalculate_trim = False # Do not recalculate trim unless trim parameter modified

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

                case "va_trim":
                    self._va_trim = param.value
                    recalculate_trim = True

                case "gamma_trim":
                    self._gamma_trim = param.value
                    recalculate_trim = True

                case "radius_trim":
                    self._radius_trim = param.value
                    recalculate_trim = True

        if recalculate_trim:
            (uav_trim_state, self._command) = self._calculate_trim()
            self._reset_trim_state(uav_trim_state=uav_trim_state)

        return SetParametersResult(successful=successful)

    def _calculate_trim(self) -> tuple[UavState, ControlSurfaceCommands]:
        """ Calculates the trim trajectory and produces the corresponding UavState message and command message
        """

        # Calculate the trim trajectory
        state_init = DynamicState()
        if self._radius_trim <= 0.:
            radius = np.inf
        else:
            radius = self._radius_trim
        trim_state, trim_input = compute_trim(state0=state_init.convert_to_numpy(), Va=self._va_trim, gamma=self._gamma_trim, R=radius)
        trim_state_struct = DynamicState(trim_state)

        # Store the trim state
        uav_state = UavState()

        uav_state.pose.header.stamp = self.get_clock().now().to_msg() # Position and orientation
        uav_state.pose.header.frame_id = 'ned'
        uav_state.pose.pose.position.x = trim_state_struct.north
        uav_state.pose.pose.position.y = trim_state_struct.east
        uav_state.pose.pose.position.z = trim_state_struct.down
        (uav_state.phi, uav_state.theta, uav_state.psi) = trim_state_struct.extract_euler()

        uav_state.twist.header.stamp = self.get_clock().now().to_msg() # Velocities
        uav_state.twist.header.frame_id = 'body'
        uav_state.twist.twist.linear.x = trim_state_struct.u
        uav_state.twist.twist.linear.y = trim_state_struct.v
        uav_state.twist.twist.linear.z = trim_state_struct.w
        uav_state.twist.twist.angular.x = trim_state_struct.p
        uav_state.twist.twist.angular.y = trim_state_struct.q
        uav_state.twist.twist.angular.z = trim_state_struct.r

        # Store the trim commands
        trim_commands = ControlSurfaceCommands()
        trim_commands.aileron = trim_input.aileron
        trim_commands.elevator = trim_input.elevator
        trim_commands.rudder = trim_input.rudder
        trim_commands.throttle = trim_input.throttle

        # Return the state and control commands
        return (uav_state, trim_commands)

    def _reset_trim_state(self, uav_trim_state: UavState) -> None:
        """ Calls the reset service to set the uav state
        """
        # Create the reset state client
        set_client = self.create_client(SetUavState, 'set_state')
        while not set_client.wait_for_service(timeout_sec=1.0) and rclpy.ok():
            #self.get_logger().warn('set_state service not available, waiting again...')
            self._status.level = DiagnosticStatus.WARN
            self._status.message = 'Waiting for set_state service to become available'

        # Call the reset state with the trim trajectory
        req = SetUavState.Request()
        req.desired_state = uav_trim_state
        set_client.call_async(req)

        self._status.level = DiagnosticStatus.OK
        self._status.message = 'Trim state calculated'

    def _main_loop(self) -> None:
        """ Publishes the command inputs
        """
        self._pub_command.publish(self._command)

    def _diagnostic_publishing(self) -> None:
        """Updates the diagnostic message
        """
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "path_planner"
        msg.status.append(self._status)
        self._pub_diag.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # Create the trim trajectory node
    trim_node = TrimTrajNode()
    rclpy.spin(trim_node)

    # Shutdown the node
    trim_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

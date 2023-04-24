import rclpy
from rclpy.node import Node
from uav_interfaces.msg import UavState
from geometry_msgs.msg import Wrench
from builtin_interfaces.msg import Time
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
import numpy as np
from mav_sim.chap3.mav_dynamics import DynamicState, ForceMoments, MavDynamics
from math import sqrt
from std_srvs.srv import Empty


class KinematicMavNode(Node):
    """ Produces state updates based upon the current state and the input forces
        and moments vector

        Subscriptions:
            /forces_moments: (geometry_msgs/msg/WrenchStamped)

        Publications:
            /uav_state: (uav_interfaces/msg/UavState) state of the mav

        Services:
            /reset_state: Resets the state to the initial state and forces/moments to zero
    """
    def __init__(self) -> None:
        """ Initializes the publications and subscriptions
        """

        # Initialize the node
        super().__init__(node_name="kinematic_mav_node")

        # Create parameter for simulation time
        self._ts = 0.01
        self.declare_parameter(name="ts", value=self._ts, descriptor=ParameterDescriptor(description='Simulation period or time step'))
        self.add_on_set_parameters_callback(self.param_callback)

        # Initialize the mav
        self._state_init: DynamicState
        self._initialize_state()
        self._fm = ForceMoments(force_moment=np.zeros([6,1]))
        self._mav = MavDynamics(self._ts, self._state_init)

        # Create the state message
        self._time_latest: Time = self.get_clock().now()

        # Create the state publisher
        self._pub_state = self.create_publisher(UavState, "uav_state", 1)

        # Create the subsciber for the force-moments message
        self.sub_fm = self.create_subscription(Wrench, "forces_moments", self.fm_callback, 1)

        # Create the reset state service
        self.reset_state_srv = self.create_service(Empty, 'reset_state', self.reset_state_callback)

        # Create the main loop callback timer
        self._main_timer = self.create_timer(self._ts, self.main_loop)

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
                            self._ts = param.value # Update the node value
                            self._mav.ts_simulation = self._ts # Update the mav value

                            # Recreate the timer
                            self.destroy_timer(self._main_timer)
                            self._main_timer = self.create_timer(self._ts, self.main_loop)
                    else:
                        successful = False


        return SetParametersResult(successful=successful)

    def fm_callback(self, msg: Wrench) -> None:
        '''Stores the latest forces and moments
        '''
        # Store forces
        self._fm.fx = msg.force.x
        self._fm.fy = msg.force.y
        self._fm.fz = msg.force.z

        # Store moments
        self._fm.l = msg.torque.x
        self._fm.m = msg.torque.y
        self._fm.n = msg.torque.z

    def reset_state_callback(self, req: Empty.Request, res: Empty.Response) -> Empty.Response:
        """Resets the state to the default. Note that both the request and response are empty"""
        self._initialize_state()
        self._fm = ForceMoments(force_moment=np.zeros([6,1]))
        self._mav = MavDynamics(self._ts, self._state_init)
        return res

    def _initialize_state(self) -> None:
        """ Initializes the state to default values with zero velocity"""
        self._state_init = DynamicState()
        self._state_init.u = 0.
        self._state_init.v = 0.
        self._state_init.w = 0.


    def main_loop(self) -> None:
        """Executes the main loop of the node
        """
        # Calculate the time difference from the previous loop
        time_new = self.get_clock().now()
        diff = (time_new - self._time_latest).nanoseconds/1e9
        self._time_latest = time_new

        # Check the time difference to ensure it is valid prior to integrating the state
        if diff <= 0. or diff > 10.*self._ts:
            self.get_logger().warn("Time difference = " + str(diff) + ". Non-positive or too large, not updating the state")
            return

        # Update the state
        self._mav.update(forces_moments=self._fm.to_array(), time_step=diff)

        # Publish the updated state
        state = self._mav.get_state()
        phi, theta, psi = self._mav.get_euler()
        uav_state = UavState()

        uav_state.pose.header.stamp = self._time_latest.to_msg() # Position and orientation
        uav_state.pose.header.frame_id = 'ned'
        uav_state.pose.pose.position.x = state.north
        uav_state.pose.pose.position.y = state.east
        uav_state.pose.pose.position.z = state.down
        uav_state.pose.pose.orientation.x = state.e0
        uav_state.pose.pose.orientation.y = state.e1
        uav_state.pose.pose.orientation.z = state.e2
        uav_state.pose.pose.orientation.w = state.e3
        uav_state.phi = phi
        uav_state.theta = theta
        uav_state.psi = psi

        uav_state.twist.header.stamp = self._time_latest.to_msg() # Velocities
        uav_state.twist.header.frame_id = 'body'
        uav_state.twist.twist.linear.x = state.u
        uav_state.twist.twist.linear.y = state.v
        uav_state.twist.twist.linear.z = state.w
        uav_state.twist.twist.angular.x = state.p
        uav_state.twist.twist.angular.y = state.q
        uav_state.twist.twist.angular.z = state.r

        uav_state.gyro_bias.header.stamp = self._time_latest.to_msg() # Gyro bias
        uav_state.gyro_bias.header.frame_id = 'body'
        uav_state.gyro_bias.vector.x = 0.
        uav_state.gyro_bias.vector.y = 0.
        uav_state.gyro_bias.vector.z = 0.

        uav_state.v_a = sqrt(state.u**2 + state.v**2 + state.w**2) # Wind variables
        uav_state.v_g = uav_state.v_a
        uav_state.alpha = 0.
        uav_state.beta = 0.
        uav_state.gamma = theta
        uav_state.chi = psi

        self._pub_state.publish(uav_state)



def main(args=None):
    rclpy.init(args=args)

    # Create the kinematic node
    kinematic_node = KinematicMavNode()
    rclpy.spin(kinematic_node)

    # Shutdown the node
    kinematic_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
"""Provides interface functionality to the mav sim dynamics"""

from rclpy.node import Node, Publisher
from rclpy.time import Time, Duration
from uav_interfaces.msg import UavState, UavStateWrench, ControlSurfaceCommands, WindVector
from uav_interfaces.srv import SetUavState
from geometry_msgs.msg import WrenchStamped, Vector3Stamped
import numpy as np
from mav_sim.chap3.mav_dynamics import DynamicState, ForceMoments
from mav_sim.chap4.mav_dynamics import MavDynamics
from mav_sim.message_types.msg_delta import MsgDelta
from mav_sim.tools import types
from mav_sim.tools.rotations import Euler2Quaternion
from std_srvs.srv import Empty
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from uav_launch.conversions import convert_mav_state_to_msg as convert_state_to_msg

class DynamicsData:
    """Stores all of the data needed to make function calls to the dynamics"""
    def __init__(self, ts: float, time_latest: Time, node: Node) -> None:
        """Create the dynamics variables

        Inputs:
            ts: Simulation time step
            node: Node used to determine initial time and tf data
        """
        self.ts = ts # Simulation time step
        self.executing: bool = False # Flag inidicating whether or not the state is executing

        # MAV state and inputs data
        self._state_init: DynamicState = DynamicState()
        self.fm = ForceMoments(force_moment=np.zeros([6,1]))
        self.mav = MavDynamics(self.ts, self._state_init)
        self.time_latest: Time = time_latest # Stores the latest time seen

        # Create the transform listener
        self.tf_buffer = Buffer()
        self._tf_listener = TransformListener(self.tf_buffer, node)

        # Create storage for data passed in from node subscribers
        self.command = MsgDelta()
        self._wind: types.WindVector = types.WindVector(np.zeros((6,1)) )
        self._wind_ss = Vector3Stamped() # Stores the steady state wind, initialized to zero with the ned frame
        self._wind_ss.header.frame_id = "ned"
        self._wind_ss.header.stamp = self.time_latest.to_msg()
        self._wind_gust = Vector3Stamped() # Stores the wind gust, initialized to zero with the body frame
        self._wind_gust.header.frame_id = "body"
        self._wind_gust.header.stamp = self.time_latest.to_msg()

    def command_callback(self, msg: ControlSurfaceCommands) -> None:
        '''Stores the control surface commands for use by the dynamics
        '''
        self.command.elevator = msg.elevator
        self.command.aileron = msg.aileron
        self.command.rudder = msg.rudder
        self.command.throttle = msg.throttle

    def wind_callback(self, msg: WindVector) -> None:
        '''Stores the most recent wind vector information
        '''
        self._wind_ss = msg.steady_state
        self._wind_gust = msg.gust

    def extract_wind(self) -> None:
        '''Takes the steady state and gust wind values, converts them to the correct frame, and stores
           them in the wind structure
        '''
        # Transform steady state vector
        if self._wind_ss.header.frame_id == "ned":
            wind_ss_trans = self._wind_ss
        else:
            try:
                #TODO: Fix the transform code below
                self.get_logger().error("Transforms not yet implemented")
                self.get_logger().warn("Wind_ss frame = " + self._wind_ss.header.frame_id)
                wind_ss_trans = self._tf_buffer.transform(object_stamped=self._wind_ss, target_frame="ned")
            except TransformException as ex:
                self.get_logger().warn('could not transform steady stead wind from ' + self._wind_ss.header.frame_id + ' to ned')
                wind_ss_trans = Vector3Stamped()

        # Transform gust vector
        if self._wind_gust.header.frame_id == "body":
            wind_gust_trans = self._wind_gust
        else:
            try:
                #TODO: Fix the transform code below
                self.get_logger().error("Transforms not yet implemented")
                self.get_logger().warn("Wind_gust frame = " + self._wind_gust.header.frame_id)
                wind_gust_trans = self._tf_buffer.transform(object_stamped=self._wind_gust, target_frame="body")
            except TransformException as ex:
                self.get_logger().warn('could not transform gust wind from ' + self._wind_gust.header.frame_id + ' to body')
                wind_gust_trans = Vector3Stamped()

        # Extract wind variables
        self._wind[0] = wind_ss_trans.vector.x
        self._wind[1] = wind_ss_trans.vector.y
        self._wind[2] = wind_ss_trans.vector.z
        self._wind[3] = wind_gust_trans.vector.x
        self._wind[4] = wind_gust_trans.vector.y
        self._wind[5] = wind_gust_trans.vector.z

    def reset_state_callback(self, req: Empty.Request, res: Empty.Response) -> Empty.Response:
        """Resets the state and inputs to the default. Note that both the request and response are empty"""
        self.reset_state()
        return res

    def reset_state(self) -> None:
        """Resets the state and inputs to the default."""
        self.mav = MavDynamics(self.ts, self._state_init)
        self.command = MsgDelta()
        self._wind: types.WindVector = types.WindVector(np.zeros((6,1)) )

    def set_state_callback(self, req: SetUavState.Request, res: SetUavState.Response) -> SetUavState.Response:
        """Sets the state to the desired value"""
        # Check position frame
        if req.desired_state.pose.header.frame_id != "ned":
            self.get_logger().warn("SetState service request position does not have ned frame, setting assuming ned frame")

        # Set position
        self._state_init.north = float(req.desired_state.pose.pose.position.x)
        self._state_init.east = req.desired_state.pose.pose.position.y
        self._state_init.down = req.desired_state.pose.pose.position.z

        # Set orientation
        quat = Euler2Quaternion(phi=req.desired_state.phi, theta=req.desired_state.theta, psi=req.desired_state.psi)
        self._state_init.e0 = quat.item(0)
        self._state_init.e1 = quat.item(1)
        self._state_init.e2 = quat.item(2)
        self._state_init.e3 = quat.item(3)

        # Check twist frame
        if req.desired_state.twist.header.frame_id != "body":
            self.get_logger().warn("SetState service request velocities (twist) does not have body frame, setting assuming body frame")

        # Set linear velocities
        self._state_init.u = req.desired_state.twist.twist.linear.x
        self._state_init.v = req.desired_state.twist.twist.linear.y
        self._state_init.w = req.desired_state.twist.twist.linear.z

        # Set rotational velocities
        self._state_init.p = req.desired_state.twist.twist.angular.x
        self._state_init.q = req.desired_state.twist.twist.angular.y
        self._state_init.r = req.desired_state.twist.twist.angular.z

        # Check other variables and set warning message
        if  req.desired_state.pose.pose.orientation.x != 0. or \
            req.desired_state.pose.pose.orientation.y != 0. or \
            req.desired_state.pose.pose.orientation.z != 0. or \
            req.desired_state.pose.pose.orientation.w != 1. or \
            req.desired_state.gyro_bias.vector.x != 0. or \
            req.desired_state.gyro_bias.vector.y != 0. or \
            req.desired_state.gyro_bias.vector.z != 0. or \
            req.desired_state.v_a != 0. or \
            req.desired_state.v_g != 0. or \
            req.desired_state.alpha != 0. or \
            req.desired_state.beta != 0. or \
            req.desired_state.gamma != 0. or \
            req.desired_state.chi != 0. :
            self.get_logger().warn("SetState Service only uses position, euler angles, and twist. Another variable was seen to be non-zero")

        # Reset the state and populate the resulting state
        self.reset_state()
        res.resulting_state = convert_mav_state_to_msg(self, 1.)
        return res

    def toggle_callback(self, req: Empty.Request, res: Empty.Response) -> Empty.Response:
        """Toggles whether or not the state is updating"""
        self.executing = not self.executing
        return res

def convert_mav_state_to_msg(data: DynamicsData, velocity_scale: float) -> UavState:
    ''' Extracts the state from the mav and stores it in a UavState message

    Inputs:
        velocity_scale: Scale on the velocities. This allows the velocities to easily be zeroed out when
                        the sim is not moving
    Returns:
        The current state in the mav
    '''
    state = data.mav.get_struct_state()
    return convert_state_to_msg(time=data.time_latest, msg_state=data.mav.true_state,
        dyn_state=state, velocity_scale=velocity_scale)

def update_state(data: DynamicsData, delta: MsgDelta, time_step: float) -> bool:
    """Takes a single simulation step for the mav. The internal state of data.mav is updated.

    Inputs:
        data: The dynamics data storing the mav data
        delta: The command from the autopilot
        time_step: The size of the timestep for simulation

    Returns:
        True if state updated, false otherwise
    """
    if data.executing:
        data.mav.update(delta=delta, wind=data._wind, time_step=time_step)
    return data.executing

def publish_state_wrench(data: DynamicsData, pub_state: Publisher,
        pub_wrench: Publisher, pub_combined: Publisher) -> UavStateWrench:
    """Publishes the state, wrench, and combined messages

    Inputs:
        data: The dynamics data storing the mav data
        pub_state: Publisher of the UavState state message
        pub_wrench: Publisher of the WrenchStamped message
        pub_combined: Publisher of the UavStateWrench message

    Returns:
        combined state/wrench message
    """
    # Publish the updated state
    state = convert_mav_state_to_msg(data=data, velocity_scale=1.0)
    pub_state.publish(state)

    # Publish the forces and moments vector
    wrench_struct = data.mav.get_fm_struct()
    wrench = WrenchStamped()
    wrench.header.stamp = data.time_latest.to_msg()
    wrench.header.frame_id = 'body'
    wrench.wrench.force.x = wrench_struct.fx
    wrench.wrench.force.y = wrench_struct.fy
    wrench.wrench.force.z = wrench_struct.fz
    wrench.wrench.torque.x = wrench_struct.l
    wrench.wrench.torque.y = wrench_struct.m
    wrench.wrench.torque.z = wrench_struct.n
    pub_wrench.publish(wrench)

    # Publish the combined state and wrench
    state_wrench = UavStateWrench()
    state_wrench.state = state
    state_wrench.wrench = wrench
    pub_combined.publish(state_wrench)
    return state_wrench

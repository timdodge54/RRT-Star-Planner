import bisect
from rclpy.node import Node
from rclpy.time import Time
from builtin_interfaces.msg import Time as TimeMsg
from sensor_msgs.msg import Imu
import threading
from uav_interfaces.msg import UavState, ControlSurfaceCommands, Compass, Gps, Pressure

def trim_data(time: list[float], data: list[float], val_thresh: float) -> None:
    """Trims the time and data vectors to only have time values over the latest 'val_thresh' window"""

    ind = bisect.bisect_left(time, time[-1] - val_thresh)
    if ind > 0:
        del data[0:ind]
        del time[0:ind]

def time_to_seconds(time: TimeMsg) -> float:
    """Converts the time to seconds
    """
    seconds = time.sec + time.nanosec*1.e-9
    return seconds

class StateStorage:
    """Stores state data created from UavState"""

    def __init__(self, t_horizon) -> None:
        """ Initialize the state storage

        Inputs:
            t_horizon: Time horizon for storage
        """

        # Store variables
        self.t_horizon = t_horizon      # Time horizon

        # Create storage
        self.time: list[float] = []     # Time values (seconds)
        self.pn: list[float] = []       # North positions
        self.pe: list[float] = []       # East positions
        self.p_alt: list[float] = []    # Altitude
        self.phi: list[float] = []      # Attitude euler angles
        self.theta: list[float] = []
        self.psi: list[float] = []
        self.u: list[float] = []        # x,y,z body frame velocities
        self.v: list[float] = []
        self.w: list[float] = []
        self.p: list[float] = []        # x,y,z body frame rotational velocities
        self.q: list[float] = []
        self.r: list[float] = []
        self.gyro_bx: list[float] = []  # Gyro biases in body frame
        self.gyro_by: list[float] = []
        self.gyro_bz: list[float] = []
        self.w_n: list[float] = []      # North wind
        self.w_e: list[float] = []      # East wind
        self.v_a: list[float] = []      # Airspeed
        self.v_g: list[float] = []      # Ground speed
        self.alpha: list[float] = []    # Angle of attack
        self.beta: list[float] = []     # sideslip angle
        self.gamma: list[float] = []    # Flight angle
        self.chi: list[float] = []      # course angle

    def append(self, state: UavState):
        """ Stores the state data and trims the vectors
        """
        # Store the data
        self.time.append(time_to_seconds(state.pose.header.stamp))
        self.pn.append(state.pose.pose.position.x)
        self.pe.append(state.pose.pose.position.y)
        self.p_alt.append(-state.pose.pose.position.z) # -1 to get the altitude instead of down
        self.phi.append(state.phi)
        self.theta.append(state.theta)
        self.psi.append(state.psi)
        self.u.append(state.twist.twist.linear.x)
        self.v.append(state.twist.twist.linear.y)
        self.w.append(state.twist.twist.linear.z)
        self.p.append(state.twist.twist.angular.x)
        self.q.append(state.twist.twist.angular.y)
        self.r.append(state.twist.twist.angular.z)
        self.gyro_bx.append(state.gyro_bias.vector.x)
        self.gyro_by.append(state.gyro_bias.vector.y)
        self.gyro_bz.append(state.gyro_bias.vector.z)
        self.w_n.append(state.w_n)
        self.w_e.append(state.w_e)
        self.v_a.append(state.v_a)
        self.v_g.append(state.v_g)
        self.alpha.append(state.alpha)
        self.beta.append(state.beta)
        self.gamma.append(state.gamma)
        self.chi.append(state.chi)

        # Trim the data
        ind = bisect.bisect_left(self.time, self.time[-1] - self.t_horizon)
        if ind > 0:
            del self.time[0:ind]
            del self.pn[0:ind]
            del self.pe[0:ind]
            del self.p_alt[0:ind]
            del self.phi[0:ind]
            del self.theta[0:ind]
            del self.psi[0:ind]
            del self.u[0:ind]
            del self.v[0:ind]
            del self.w[0:ind]
            del self.p[0:ind]
            del self.q[0:ind]
            del self.r[0:ind]
            del self.gyro_bx[0:ind]
            del self.gyro_by[0:ind]
            del self.gyro_bz[0:ind]
            del self.w_n[0:ind]
            del self.w_e[0:ind]
            del self.v_a[0:ind]
            del self.v_g[0:ind]
            del self.alpha[0:ind]
            del self.beta[0:ind]
            del self.gamma[0:ind]
            del self.chi[0:ind]

class CommandStorage:
    """Stores the vehicle commands
    """
    def __init__(self, t_horizon) -> None:
        """ Initialize the command storage
        """
        # Store the horizon variable
        self.t_horizon = t_horizon

        # Create storage
        self.time: list[float] = []     # Time values (seconds)
        self.elevator: list[float] = []
        self.aileron: list[float] = []
        self.rudder: list[float] = []
        self.throttle: list[float] = []

    def append(self, cmd: ControlSurfaceCommands, time: Time) -> None:
        """ Stores the command data and trims the vectors
        """
        # Append data
        self.time.append(time.nanoseconds*1.e-9)
        self.elevator.append(cmd.elevator)
        self.aileron.append(cmd.aileron)
        self.rudder.append(cmd.rudder)
        self.throttle.append(cmd.throttle)

        # Trim the data
        ind = bisect.bisect_left(self.time, self.time[-1] - self.t_horizon)
        if ind > 0:
            del self.time[0:ind]
            del self.elevator[0:ind]
            del self.aileron[0:ind]
            del self.rudder[0:ind]
            del self.throttle[0:ind]

class CompassStorage:
    """Stores the compass readings
    """
    def __init__(self, t_horizon) -> None:
        """ Initialize the compass storage
        """
        # Store the horizon variable
        self.t_horizon = t_horizon

        # Create storage
        self.time: list[float] = []     # Time values (seconds)
        self.angle: list[float] = []

    def append(self, msg: Compass) -> None:
        """ Stores the compass data and trims the vectors
        """
        # Append data
        self.time.append(time_to_seconds(msg.header.stamp))
        self.angle.append(msg.angle)

        # Trim the data
        ind = bisect.bisect_left(self.time, self.time[-1] - self.t_horizon)
        if ind > 0:
            del self.time[0:ind]
            del self.angle[0:ind]

class GpsStorage:
    """Stores GPS readings"""

    def __init__(self, t_horizon) -> None:
        """ Initialize the GPS storage

        Inputs:
            t_horizon: Time horizon for storage
        """

        # Store variables
        self.t_horizon = t_horizon      # Time horizon

        # Create storage
        self.time: list[float] = []     # Time values (seconds)
        self.pn: list[float] = []       # North positions
        self.pe: list[float] = []       # East positions
        self.p_alt: list[float] = []    # Altitude
        self.v_g: list[float] = []      # Ground velocity
        self.course: list[float] = []   # Course angle

    def append(self, msg: Gps):
        """ Stores the gps data and trims the vectors
        """
        # Store the data
        self.time.append(time_to_seconds(msg.position.header.stamp))
        self.pn.append(msg.position.point.x)
        self.pe.append(msg.position.point.y)
        self.p_alt.append(-msg.position.point.z) # -1 to get the altitude instead of down
        self.v_g.append(msg.ground_velocity)
        self.course.append(msg.course_ang)

        # Trim the data
        ind = bisect.bisect_left(self.time, self.time[-1] - self.t_horizon)
        if ind > 0:
            del self.time[0:ind]
            del self.pn[0:ind]
            del self.pe[0:ind]
            del self.p_alt[0:ind]
            del self.v_g[0:ind]
            del self.course[0:ind]

class ImuStorage:
    """Stores IMU readings"""

    def __init__(self, t_horizon) -> None:
        """ Initialize the IMU storage

        Inputs:
            t_horizon: Time horizon for storage
        """

        # Store variables
        self.t_horizon = t_horizon      # Time horizon

        # Create storage
        self.time: list[float] = []     # Time values (seconds)
        self.ax: list[float] = []       # x,y,z body frame accelerations
        self.ay: list[float] = []
        self.az: list[float] = []
        self.p: list[float] = []        # x,y,z body frame rotational velocities
        self.q: list[float] = []
        self.r: list[float] = []

    def append(self, msg: Imu):
        """ Stores the imu data and trims the vectors
        """
        # Store the data
        self.time.append(time_to_seconds(msg.header.stamp))
        self.ax.append(msg.linear_acceleration.x)
        self.ay.append(msg.linear_acceleration.y)
        self.az.append(msg.linear_acceleration.z)
        self.p.append(msg.angular_velocity.x)
        self.q.append(msg.angular_velocity.y)
        self.r.append(msg.angular_velocity.z)

        # Trim the data
        ind = bisect.bisect_left(self.time, self.time[-1] - self.t_horizon)
        if ind > 0:
            del self.time[0:ind]
            del self.ax[0:ind]
            del self.ay[0:ind]
            del self.az[0:ind]
            del self.p[0:ind]
            del self.q[0:ind]
            del self.r[0:ind]

class PressureStorage:
    """Stores Pressure readings"""

    def __init__(self, t_horizon) -> None:
        """ Initialize the Pressure storage

        Inputs:
            t_horizon: Time horizon for storage
        """

        # Store variables
        self.t_horizon = t_horizon      # Time horizon

        # Create storage
        self.time: list[float] = []     # Time values (seconds)
        self.abs_pressure: list[float] = []
        self.diff_pressure: list[float] = []

    def append(self, msg: Pressure):
        """ Stores the pressure data and trims the vectors
        """
        # Store the data
        self.time.append(time_to_seconds(msg.stamp))
        self.abs_pressure.append(msg.abs_pressure)
        self.diff_pressure.append(msg.diff_pressure)

        # Trim the data
        ind = bisect.bisect_left(self.time, self.time[-1] - self.t_horizon)
        if ind > 0:
            del self.time[0:ind]
            del self.abs_pressure[0:ind]
            del self.diff_pressure[0:ind]

class RosStorageInterface:
    """ Uses a given node to subscribe to state, command, and sensory information
    """

    def __init__(self, node: Node, t_horizon: float) -> None:
        """ Create the ros interfaces required for storage

        Inputs:
            node: Node to use for subscriptions
            t_horizon: Time horizon of data to keep
        """

        # Store inputs
        self.node = node
        self.t_horizon = t_horizon

        # Initialize the state parameters
        self.t_horizon = 100. # TODO Read in instead of hard code
        self.lock = threading.Lock() # The lock is used to allow data to not be received / updated
                                     # while being accessed


        # Initialize the ros variables
        self._sub_state = self.node.create_subscription(UavState, "/uav_state", self.state_callback, 1)
        self._sub_est = self.node.create_subscription(UavState, "/uav_state_estimate", self.state_estimate_callback, 1)
        self._sub_cmd = self.node.create_subscription(ControlSurfaceCommands, "/command", self.command_callback, 1)
        self._sub_comp = self.node.create_subscription(Compass, "/compass", self.compass_callback, 1)
        self._sub_gps = self.node.create_subscription(Gps, "/gps", self.gps_callback, 1)
        self._sub_imu = self.node.create_subscription(Imu, "/imu", self.imu_callback, 1)
        self._sub_pre = self.node.create_subscription(Pressure, "/pressure", self.pressure_callback, 1)

        # Initailize the storage
        self.true = StateStorage(t_horizon=self.t_horizon)
        self.est = StateStorage(t_horizon=self.t_horizon)
        self.cmd = CommandStorage(t_horizon=self.t_horizon)
        self.comp = CompassStorage(t_horizon=self.t_horizon)
        self.gps = GpsStorage(t_horizon=self.t_horizon)
        self.imu = ImuStorage(t_horizon=self.t_horizon)
        self.press = PressureStorage(t_horizon=self.t_horizon)

    def state_callback(self, msg: UavState) -> None:
        """Stores the latest state data
        """
        with self.lock:
            self.true.append(state=msg)

    def state_estimate_callback(self, msg: UavState) -> None:
        """Stores the latest state estimate data
        """
        #self.node.get_logger().info("Enter state estimate callback")
        with self.lock:
            self.est.append(state=msg)

    def command_callback(self, msg: ControlSurfaceCommands) -> None:
        """Stores the latest command data
        """
        with self.lock:
            self.cmd.append(cmd=msg, time=self.node.get_clock().now())

    def compass_callback(self, msg: Compass) -> None:
        """Stores the latest compass data
        """
        with self.lock:
            self.comp.append(msg=msg)

    def gps_callback(self, msg: Gps) -> None:
        """Stores the latest gps data
        """
        with self.lock:
            self.gps.append(msg=msg)

    def imu_callback(self, msg: Imu) -> None:
        """Stores the latest imu data
        """
        with self.lock:
            self.imu.append(msg=msg)

    def pressure_callback(self, msg: Pressure) -> None:
        """Stores the latest pressure data
        """
        with self.lock:
            self.press.append(msg=msg)



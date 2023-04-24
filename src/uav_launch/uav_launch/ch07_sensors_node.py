from builtin_interfaces.msg._time import Time as TimeMsg
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray
import mav_sim.parameters.sensor_parameters as SENSOR
import math
from mav_sim.chap3.mav_dynamics import DynamicState, IND
from mav_sim.chap7.mav_dynamics import gyro, accelerometer, magnetometer, pressure, gps_error_trans_update, GpsTransient, gps
from mav_sim.tools.rotations import Quaternion2Euler
from mav_sim.tools import types
import numpy as np
from uav_interfaces.msg import UavStateWrench, Magnetometer, Pressure, Gps, Compass
import rclpy
from rclpy.node import Node, Subscription, Timer
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from typing import Optional, cast
from uav_launch.conversions import state_ros_to_vector

class ImuMeasurements():
    """Struct for storing all measurements performed at the imu rate"""
    def __init__(self) -> None:
        """Define each of the imu measurements"""
        # Gyro: body frame x-y-z measurements
        self.gyro_x: float
        self.gyro_y: float
        self.gyro_z: float

        # Accelerometer: body frame x-y-z acceleration measurements
        self.accel_x: float
        self.accel_y: float
        self.accel_z: float

        # Magnetometer: body frame x-y-z magnetometer measurements
        self.mag_x: float
        self.mag_y: float
        self.mag_z: float

        # Compass: Meaured rotation about the intertial down axis (ned frame)
        self.compass_angle: float

        # Pressure
        self.abs_pressure: float    # Absolute pressure measurement
        self.diff_pressure: float   # Differential pressure measurement

class SensorParameters():
    """Struct for storing the parameters for determining each of the sensors"""

    def __init__(self) -> None:
        """Define each of the sensor parameters"""
        # General parameters
        self.noise_scale: float = 1.0        # Scaling of the noise (ability to turn noise off)
        self.imu_period: float = 0.02        # Period at which imu will be published
        self.gps_period: float = SENSOR.ts_gps                  # Period at which gps sensor will be published

        # Accelerometer
        self.accel_sigma: float = SENSOR.accel_sigma    # standard deviation of accelerometers in m/s^2

        # Magetometer parameters
        self.mag_sigma = SENSOR.mag_sigma                       # Standard deviation of the magnetic sensor measurement
        self.mag_dec = SENSOR.mag_dec                           # Declination of the magnetic to inertial frame
        self.mag_inc = SENSOR.mag_inc                           # Inclination of the magnetic to inertial frame

        # Compass parameters
        self.compass_bias: float = 0.0                          # Compass bias
        self.compass_sigma: float = 0.1                         # Standard deviation of the compass sensor reading

        # Gyro parameters
        self.gyro_sigma: float = SENSOR.gyro_sigma              # Standard deviation of gyros in rad/sec
        self.gyro_x_bias: float = SENSOR.gyro_x_bias            # bias on x axis in body frame
        self.gyro_y_bias: float = SENSOR.gyro_y_bias            # bias on y axis in body frame
        self.gyro_z_bias: float = SENSOR.gyro_z_bias            # bias on z axis in body frame

        # Initialize parameter for pressure sensors
        # Altitude (absolute) parameters
        self.abs_pres_bias: float = SENSOR.abs_pres_bias        # Bias on the absolute pressure measurement
        self.abs_pres_sigma: float = SENSOR.abs_pres_sigma      # standard deviation of absolute pressure sensors in Pascals

        # Airspeed (differential) parameters
        self.diff_pres_bias: float = SENSOR.diff_pres_bias      # Bias on the differential pressure
        self.diff_pres_sigma: float = SENSOR.diff_pres_sigma    # Standard deviation of diff pressure sensor in Pascals

        # GPS position parameters
        self.gps_k: float = SENSOR.gps_k                        # 1 / s - time constant of the process
        self.gps_n_sigma: float = SENSOR.gps_n_sigma            # Standard deviation of the north gps measurement
        self.gps_e_sigma: float = SENSOR.gps_e_sigma            # Standard deviation of the east gps measurement
        self.gps_h_sigma: float = SENSOR.gps_h_sigma            # Standard deviation of the altitude gps measurement

        # GPS ground velocity parameters
        self.gps_Vg_sigma: float = SENSOR.gps_Vg_sigma          # Standard deviation of the ground velocity measurement

        # GPS course angle parameters
        self.gps_course_sigma: float = SENSOR.gps_course_sigma  # Standard deviation of the course angle measurement"))

def create_imu_measurement(sp: SensorParameters, state: types.DynamicState, psi: float,
        v_a: float, forces: types.NP_MAT) -> ImuMeasurements:
        """Create all of the imu measurements

        Inputs:
            state: 13x1 numpy array of the state
            psi: Orientation about the down axis in ned frame
            v_a: airspeed
            forces: 3x1 numpy array of the forces

        Outputs:
            sm: Sensor measurements calculated
        """
        # Define variables for sensor calculation
        st = DynamicState(state)
        continuous_noise_mult = 1./sp.imu_period
        sm = ImuMeasurements()

        # Extract needed orientation values
        phi, theta, _ = Quaternion2Euler(state[IND.QUAT])

        # simulate rate gyros(units are rad / sec)
        sm.gyro_x, sm.gyro_y, sm.gyro_z = gyro(st.p, st.q, st.r, sp.noise_scale,
                                      gyro_sigma=math.sqrt((sp.gyro_sigma**2)*continuous_noise_mult),
                                      gyro_x_bias=sp.gyro_x_bias, gyro_y_bias=sp.gyro_y_bias,
                                      gyro_z_bias=sp.gyro_z_bias)

        # simulate accelerometers(units of g)
        sm.accel_x, sm.accel_y, sm.accel_z = accelerometer(phi, theta, forces, sp.noise_scale,
                                                  accel_sigma=math.sqrt((sp.accel_sigma**2)*continuous_noise_mult))

        # simulate magnetometers (vector in body frame)
        quat = cast(types.Quaternion, state[IND.QUAT])
        sm.mag_x, sm.mag_y, sm.mag_z = magnetometer(quat, sp.noise_scale, mag_inc=sp.mag_inc,
                                    mag_dec=sp.mag_dec, mag_sigma=sp.mag_sigma)

        # Simulate the compass measurement
        sm.compass_angle = psi + sp.compass_bias + \
                            (sp.noise_scale * np.random.normal(0., sp.compass_sigma))

        # simulate pressure sensors
        sm.abs_pressure, sm.diff_pressure = pressure(st.down, v_a, sp.noise_scale,
                                        abs_pres_bias=sp.abs_pres_bias, abs_pres_sigma=sp.abs_pres_sigma,
                                        diff_pres_bias=sp.diff_pres_bias, diff_pres_sigma=sp.diff_pres_sigma)

        return sm

class SensorsInterface():
    """ Calculates the sensor measurements based upon the latest state and forces
        and publishes them into three messages.

    Subscriptions:
        /uav_state: (uav_interfaces/msg/UavState) state of the mav
        /forces_moments: (geometry_msgs/msg/WrenchStamped)
        /gps_denial: (std_msgs/msg/Bool) - True if in gps denied region, false otherwise

    Publications
        /imu: (sensor_msgs/msg/imu) IMU measurements - accelerometer, and gyro
        /magnetometer: (uav_interface/msg/magnetometer) - magnetometer measurement vector
        /pressure: (uav_interface/msg/pressure) Absolute and differential pressure measurements
        /gps: (uav_interface/msg/gps) Gps position, velocity, and course angle measurements
        /diagnostics: (diagnostic_msgs/DiagnosticArray) Diagnostics for the dynamics
    """

    def __init__(self, node: Node, sensor_parameters: SensorParameters) -> None:
        """ Initializes the parameters, publications, and subscriptions
        """
        # Store input parameters
        self._node = node
        self._sp = sensor_parameters
        self.declare_parameters()

        # Create the IMU publishers
        self._pub_imu = self._node.create_publisher(Imu, "imu", 1)
        self._pub_mag = self._node.create_publisher(Magnetometer, "magnetometer", 1)
        self._pub_compass = self._node.create_publisher(Compass, "compass", 1)
        self._pub_diff = self._node.create_publisher(Pressure, "pressure", 1)

        # Create the gps values and publishers
        self._gps_nu = GpsTransient(nu_n = 0., nu_e=0., nu_h = 0.) #TODO, these parameter should be pulled from distribution
        self._pub_gps = self._node.create_publisher(Gps, "gps", 1)

        # Create monitor for gps denial
        self._gps_denial = Bool()
        self._gps_denial.data = False
        self._sub_gps_denial = self._node.create_subscription(Bool, "gps_denied", self._gps_denial_callback, 1)

        # Initialize subscriber for the state / forces and moments
        self._state_wrench: Optional[UavStateWrench] = None
        self._sub_state_wrench: Subscription

        # Create diagnostics variables
        self._pub_diag = self._node.create_publisher(DiagnosticArray, "/diagnostics", 1)
        self._status = DiagnosticStatus() # Stores the current status of the path follower
        self._status.message = "Initialized"
        self._status.level = DiagnosticStatus.OK
        self._status.name = "Sensors"
        self._status.hardware_id = "Sensors"
        self._diagnostic_timer = self._node.create_timer(1., self._diagnostic_publishing)

        # Create the sensor loops
        self._imu_timer: Optional[Timer] = None
        self._gps_timer = self._node.create_timer(self._sp.gps_period, self._gps_loop)

    def declare_parameters(self) -> None:
        """Declare and read in parameters"""
        # Create a noise scaling parameter
        self._node.declare_parameter(name="noise_scale", value=self._sp.noise_scale,
            descriptor=ParameterDescriptor(description="Scaling of the noise (ability to turn noise off)"))
        self._sp.noise_scale = self._node.get_parameter(name="noise_scale").value

        # Initialize parameters for the imu
        self._node.declare_parameter(name="imu_period", value=self._sp.imu_period,
            descriptor=ParameterDescriptor(description="Period at which imu will be published"))
        self._sp.imu_period = self._node.get_parameter(name="imu_period").value

        # Accelerometer parameters
        self._node.declare_parameter(name="accel_sigma", value=self._sp.accel_sigma,
            descriptor=ParameterDescriptor(description="standard deviation of accelerometers in m/s^2"))
        self._sp.accel_sigma = self._node.get_parameter(name="accel_sigma").value

        # Magetometer parameters
        self._node.declare_parameter(name="mag_sigma", value=self._sp.mag_sigma,
            descriptor=ParameterDescriptor(description="Standard deviation of the magnetic sensor measurement"))
        self._sp.mag_sigma = self._node.get_parameter(name="mag_sigma").value
        self._node.declare_parameter(name="mag_dec", value=self._sp.mag_dec,
            descriptor=ParameterDescriptor(description="Declination of the magnetic to inertial frame"))
        self._sp.mag_dec = self._node.get_parameter(name="mag_dec").value
        self._node.declare_parameter(name="mag_inc", value=self._sp.mag_inc,
            descriptor=ParameterDescriptor(description="Inclination of the magnetic to inertial frame"))
        self._sp.mag_inc = self._node.get_parameter(name="mag_inc").value

        # Compass parameters
        self._node.declare_parameter(name="compass_bias", value=self._sp.compass_bias,
            descriptor=ParameterDescriptor(description="Compass bias"))
        self._sp.compass_bias = self._node.get_parameter(name="compass_bias").value
        self._node.declare_parameter(name="compass_sigma", value=self._sp.compass_sigma,
            descriptor=ParameterDescriptor(description="Standard deviation of the compass sensor reading"))
        self._sp.compass_sigma = self._node.get_parameter(name="compass_sigma").value

        # Gyro parameters
        self._node.declare_parameter(name="gyro_sigma", value=self._sp.gyro_sigma,
            descriptor=ParameterDescriptor(description="standard deviation of gyros in rad/sec"))
        self._sp.gyro_sigma = self._node.get_parameter(name="gyro_sigma").value
        self._node.declare_parameter(name="gyro_x_bias", value=self._sp.gyro_x_bias,
            descriptor=ParameterDescriptor(description="bias on x_gyro"))
        self._sp.gyro_x_bias = self._node.get_parameter(name="gyro_x_bias").value
        self._node.declare_parameter(name="gyro_y_bias", value=self._sp.gyro_y_bias,
            descriptor=ParameterDescriptor(description="bias on y_gyro"))
        self._sp.gyro_y_bias = self._node.get_parameter(name="gyro_y_bias").value
        self._node.declare_parameter(name="gyro_z_bias", value=self._sp.gyro_z_bias,
            descriptor=ParameterDescriptor(description="bias on z_gyro"))
        self._sp.gyro_z_bias = self._node.get_parameter(name="gyro_z_bias").value

        # Initialize parameter for pressure sensors
        # Altitude (absolute) parameters
        self._node.declare_parameter(name="abs_pres_bias", value=self._sp.abs_pres_bias,
            descriptor=ParameterDescriptor(description="Bias on the absolute pressure measurement"))
        self._sp.abs_pres_bias = self._node.get_parameter(name="abs_pres_bias").value
        self._node.declare_parameter(name="abs_pres_sigma", value=self._sp.abs_pres_sigma,
            descriptor=ParameterDescriptor(description="standard deviation of absolute pressure sensors in Pascals"))
        self._sp.abs_pres_sigma = self._node.get_parameter(name="abs_pres_sigma").value

        # Airspeed (differential) parameters
        self._node.declare_parameter(name="diff_pres_bias", value=self._sp.diff_pres_bias,
            descriptor=ParameterDescriptor(description="Bias on the differential pressure"))
        self._sp.diff_pres_bias = self._node.get_parameter(name="diff_pres_bias").value
        self._node.declare_parameter(name="diff_pres_sigma", value=self._sp.diff_pres_sigma,
            descriptor=ParameterDescriptor(description="standard deviation of diff pressure sensor in Pascals"))
        self._sp.diff_pres_sigma = self._node.get_parameter(name="diff_pres_sigma").value

        # Initialize the gps
        self._node.declare_parameter(name="gps_period", value=self._sp.gps_period,
            descriptor=ParameterDescriptor(description="Period at which gps sensor will be published"))
        self._sp.gps_period = self._node.get_parameter(name="gps_period").value

        # GPS position parameters
        self._node.declare_parameter(name="gps_k", value=self._sp.gps_k,
            descriptor=ParameterDescriptor(description="1 / s - time constant of the process"))
        self._sp.gps_k = self._node.get_parameter(name="gps_k").value
        self._node.declare_parameter(name="gps_n_sigma", value=self._sp.gps_n_sigma,
            descriptor=ParameterDescriptor(description="Standard deviation of the north gps measurement"))
        self._sp.gps_n_sigma = self._node.get_parameter(name="gps_n_sigma").value
        self._node.declare_parameter(name="gps_e_sigma", value=self._sp.gps_e_sigma,
            descriptor=ParameterDescriptor(description="Standard deviation of the east gps measurement"))
        self._sp.gps_e_sigma = self._node.get_parameter(name="gps_e_sigma").value
        self._node.declare_parameter(name="gps_h_sigma", value=self._sp.gps_h_sigma,
            descriptor=ParameterDescriptor(description="Standard deviation of the altitude gps measurement"))
        self._sp.gps_h_sigma = self._node.get_parameter(name="gps_h_sigma").value

        # GPS ground velocity parameters
        self._node.declare_parameter(name="gps_Vg_sigma", value=self._sp.gps_Vg_sigma,
            descriptor=ParameterDescriptor(description="Standard deviation of the ground velocity measurement"))
        self._sp.gps_Vg_sigma = self._node.get_parameter(name="gps_Vg_sigma").value

        # GPS position parameters
        self._node.declare_parameter(name="gps_course_sigma", value=self._sp.gps_course_sigma,
            descriptor=ParameterDescriptor(description="Standard deviation of the course angle measurement"))
        self._sp.gps_course_sigma = self._node.get_parameter(name="gps_course_sigma").value


    def _gps_denial_callback(self, msg: Bool) -> None:
        """Stores the gps denial message"""
        self._gps_denial = msg

    def initialize_callbacks(self) -> None:
        """Initailizes the callbacks for the state and parameters that are needed when running the node solo, but
           may not be needed when running node within a node that provides the true state data
        """
        # Subscribe to the state
        self._sub_state_wrench = self._node.create_subscription(UavStateWrench, "state_wrench", self.state_callback,  1)

        # Create the parameter callback
        self._node.add_on_set_parameters_callback(self.param_callback)

        # Create the imu timer
        self._imu_timer = self._node.create_timer(self._sp.imu_period, self._imu_loop)

    def param_callback(self, params: list[rclpy.parameter.Parameter]) -> SetParametersResult:
        """ Set all of the parameters that are passed in
        """
        # Loop through any parameters and set them accordingly
        successful = True # Assume success unless otherwise found not to be true

        for param in params:
            successful = successful and self.process_param(param)

        return SetParametersResult(successful=successful)

    def process_param(self, param: rclpy.parameter.Parameter) -> bool:
        """Processes a single parameter. Return true if successfully processed"""
        successful = True # Default to success return
        try:
            # Check for the imu or gps period variables to update timers
            if param.name in ["imu_period", "gps_period"]:
                if param.value > 0:
                    if param.name == "imu_period":
                        if self._imu_timer is not None:
                            # Destory and recreate the imu_timer
                            self._node.destroy_timer(self._imu_timer)
                            self._imu_timer = self._node.create_timer(param.value, self._imu_loop)

                    elif param.name == "gps_period":
                        # Destroy and recreate the gps_timer
                        self._node.destroy_timer(self._gps_timer)
                        self._gps_timer = self._node.create_timer(param.value, self._gps_loop)

                else:
                    successful = False
                    self._node.get_logger().error("period not positive for setting parameter " + param.name )

            # Set the value
            setattr(self, "_sp."+param.name, param.value)

        except Exception as e:
            print(e)
            successful = False
            self._node.get_logger().error("exception caught while setting parameter " + param.name + ": " + str(e) )

        return successful

    def state_callback(self, msg: UavStateWrench) -> None:
        """ Stores the state and wrench
        """
        self._state_wrench = msg

    def _diagnostic_publishing(self) -> None:
        """Updates the diagnostic message
        """
        msg = DiagnosticArray()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "sensors"
        msg.status.append(self._status)
        self._pub_diag.publish(msg)

    def _imu_loop(self) -> None:
        """ Creates and publishes the IMU information with the latest position and force updates
        """
        # Extract the state
        if self._state_wrench is None:
            return
        state = state_ros_to_vector(self._state_wrench.state)

        # Extract the forces
        forces = np.array([ [self._state_wrench.wrench.wrench.force.x],
                            [self._state_wrench.wrench.wrench.force.y],
                            [self._state_wrench.wrench.wrench.force.z]    ])

        # Create the imu measurement
        sensor_measurements = create_imu_measurement(sp=self._sp, state=state, psi=self._state_wrench.state.psi,
            v_a= self._state_wrench.state.v_a, forces=forces)

        # Publish the imu measurement
        self.publish_imu(sm=sensor_measurements, time=self._state_wrench.wrench.header.stamp)

    def publish_imu(self, sm: ImuMeasurements, time: TimeMsg) -> None:
        """ Publish the imu data
        """
        # Create and publish the IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = time
        imu_msg.header.frame_id = "body"
        imu_msg.angular_velocity.x = sm.gyro_x # Gyro
        imu_msg.angular_velocity.y = sm.gyro_y
        imu_msg.angular_velocity.z = sm.gyro_z
        gyro_variance = self._sp.gyro_sigma**2
        imu_msg.angular_velocity_covariance = [gyro_variance, 0.,            0.,
                                                0.,           gyro_variance, 0.,
                                                0.,           0.,            gyro_variance]
        imu_msg.linear_acceleration.x = sm.accel_x # Accelerometer
        imu_msg.linear_acceleration.y = sm.accel_y
        imu_msg.linear_acceleration.z = sm.accel_z
        accel_variance = self._sp.accel_sigma**2
        imu_msg.linear_acceleration_covariance = [accel_variance, 0.,             0.,
                                                  0.,             accel_variance, 0.,
                                                  0.,             0.,             accel_variance]
        imu_msg.orientation_covariance = [-1., 0., 0., # Mark orientation not used
                                          0.,  0., 0.,
                                          0.,  0., 0.]
        self._pub_imu.publish(imu_msg)

        # Create and publish the Magnetometer message
        mag_msg = Magnetometer()
        mag_msg.magnetometer.header = imu_msg.header
        mag_msg.magnetometer.vector.x = sm.mag_x
        mag_msg.magnetometer.vector.y = sm.mag_y
        mag_msg.magnetometer.vector.z = sm.mag_z
        mag_variance = self._sp.mag_sigma**2
        mag_msg.covariance = [mag_variance,  0.,           0.,
                              0.,            mag_variance, 0.,
                              0.,            0.,           mag_variance]
        self._pub_mag.publish(mag_msg)

        # Create and publish the compass measurement
        compass_msg = Compass()
        compass_msg.header.stamp = time
        compass_msg.header.frame_id = "ned"
        compass_msg.angle = sm.compass_angle
        compass_msg.variance = self._sp.compass_sigma**2
        self._pub_compass.publish(compass_msg)

        # Create and publish the pressure sensor message
        press_msg = Pressure()
        press_msg.stamp = time
        press_msg.abs_pressure = sm.abs_pressure
        press_msg.abs_variance = self._sp.abs_pres_sigma**2
        press_msg.diff_pressure = sm.diff_pressure
        press_msg.diff_variance = self._sp.diff_pres_sigma**2
        self._pub_diff.publish(press_msg)

    def _gps_loop(self) -> None:
        """ Creates and publishes the gps sensor information
        """
        # Extract the state
        if self._state_wrench is None:
            return
        state = state_ros_to_vector(self._state_wrench.state)
        quat = cast(types.Quaternion, state[IND.QUAT])

        # Update the gps transient bias
        self._gps_nu = gps_error_trans_update(self._gps_nu, self._sp.noise_scale,
                        gps_n_sigma=self._sp.gps_n_sigma, gps_e_sigma=self._sp.gps_e_sigma,
                        gps_h_sigma=self._sp.gps_h_sigma, gps_k=self._sp.gps_k,
                        ts_gps=self._sp.gps_period)

        # Check to see if uav in the gps denied region, exit if in gps denied region
        if self._gps_denial.data:
            return

        # Calculate the gps
        gps_n, gps_e, gps_h, gps_Vg, gps_course = \
            gps(state[0:3], state[IND.VEL], quat, self._gps_nu, self._sp.noise_scale,
                gps_Vg_sigma=self._sp.gps_Vg_sigma, gps_course_sigma=self._sp.gps_course_sigma)

        # Create and publish the gps message
        gps_msg = Gps()
        gps_msg.position.header.stamp = self._state_wrench.wrench.header.stamp # Position
        gps_msg.position.header.frame_id = "ned"
        gps_msg.position.point.x = gps_n
        gps_msg.position.point.y = gps_e
        gps_msg.position.point.z = -gps_h
        gps_msg.position_transient_covariance = \
            [self._sp.gps_n_sigma**2, 0.,                    0.,
            0.,                    self._sp.gps_e_sigma**2,  0.,
            0.,                    0.,                    self._sp.gps_h_sigma**2]
        gps_msg.ground_velocity = gps_Vg                                        # Ground velocity
        gps_msg.ground_velocity_variance = self._sp.gps_Vg_sigma**2
        gps_msg.course_ang = gps_course                                         # Course angle
        gps_msg.course_angle_variance = self._sp.gps_course_sigma**2

        self._pub_gps.publish(gps_msg)

def main(args=None):
    rclpy.init(args=args)

    # Create the autopilot node
    sensors_node = rclpy.create_node(node_name="sensors_node")
    sensor_params = SensorParameters()
    sensors_interface = SensorsInterface(node=sensors_node, sensor_parameters=sensor_params)
    sensors_interface.initialize_callbacks()
    rclpy.spin(sensors_node)

    # Shutdown the node
    sensors_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


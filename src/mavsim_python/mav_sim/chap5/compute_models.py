"""
compute_ss_model
    - Chapter 5 assignment for Beard & McLain, PUP, 2012
    - Update history:
        2/4/2019 - RWB
        12/21 - GND
"""
from typing import Any, cast

# pylint: disable=protected-access
import mav_sim.parameters.aerosonde_parameters as MAV
import numpy as np
import numpy.typing as npt
from mav_sim.chap3.mav_dynamics import IND, derivatives
from mav_sim.chap4.mav_dynamics import (
    forces_moments,
    motor_thrust_torque,
    update_velocity_data,
)
from mav_sim.message_types.msg_delta import MsgDelta
from mav_sim.parameters.simulation_parameters import ts_simulation as Ts
from mav_sim.tools import types
from mav_sim.tools.rotations import Euler2Quaternion, Quaternion2Euler
from mav_sim.tools.types import Quaternion  # Type for a numpy matrix


def compute_model(trim_state: types.DynamicState, trim_input: MsgDelta) -> None:
    """TODO Document

    Args:
        trim_state: state of the trim trajectory
        trim_input: inputs for the trim trajectory
    """
    A_lon, B_lon, A_lat, B_lat = compute_ss_model(trim_state, trim_input)
    (
        Va_trim,
        alpha_trim,
        theta_trim,
        a_phi1,
        a_phi2,
        a_theta1,
        a_theta2,
        a_theta3,
        a_V1,
        a_V2,
        a_V3,
    ) = compute_tf_model(trim_state, trim_input)

    # write transfer function gains to file
    with open("model_coef.py", "w") as file:
        file.write("import numpy as np\n")
        file.write(
            "x_trim = np.array([[%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f]]).T\n"
            % (
                trim_state.item(0),
                trim_state.item(1),
                trim_state.item(2),
                trim_state.item(3),
                trim_state.item(4),
                trim_state.item(5),
                trim_state.item(6),
                trim_state.item(7),
                trim_state.item(8),
                trim_state.item(9),
                trim_state.item(10),
                trim_state.item(11),
                trim_state.item(12),
            )
        )
        file.write(
            "u_trim = np.array([[%f, %f, %f, %f]]).T\n"
            % (
                trim_input.elevator,
                trim_input.aileron,
                trim_input.rudder,
                trim_input.throttle,
            )
        )
        file.write("Va_trim = %f\n" % Va_trim)
        file.write("alpha_trim = %f\n" % alpha_trim)
        file.write("theta_trim = %f\n" % theta_trim)
        file.write("a_phi1 = %f\n" % a_phi1)
        file.write("a_phi2 = %f\n" % a_phi2)
        file.write("a_theta1 = %f\n" % a_theta1)
        file.write("a_theta2 = %f\n" % a_theta2)
        file.write("a_theta3 = %f\n" % a_theta3)
        file.write("a_V1 = %f\n" % a_V1)
        file.write("a_V2 = %f\n" % a_V2)
        file.write("a_V3 = %f\n" % a_V3)
        file.write(
            "A_lon = np.array([\n    [%f, %f, %f, %f, %f],\n    "
            "[%f, %f, %f, %f, %f],\n    "
            "[%f, %f, %f, %f, %f],\n    "
            "[%f, %f, %f, %f, %f],\n    "
            "[%f, %f, %f, %f, %f]])\n"
            % (
                A_lon[0][0],
                A_lon[0][1],
                A_lon[0][2],
                A_lon[0][3],
                A_lon[0][4],
                A_lon[1][0],
                A_lon[1][1],
                A_lon[1][2],
                A_lon[1][3],
                A_lon[1][4],
                A_lon[2][0],
                A_lon[2][1],
                A_lon[2][2],
                A_lon[2][3],
                A_lon[2][4],
                A_lon[3][0],
                A_lon[3][1],
                A_lon[3][2],
                A_lon[3][3],
                A_lon[3][4],
                A_lon[4][0],
                A_lon[4][1],
                A_lon[4][2],
                A_lon[4][3],
                A_lon[4][4],
            )
        )
        file.write(
            "B_lon = np.array([\n    [%f, %f],\n    "
            "[%f, %f],\n    "
            "[%f, %f],\n    "
            "[%f, %f],\n    "
            "[%f, %f]])\n"
            % (
                B_lon[0][0],
                B_lon[0][1],
                B_lon[1][0],
                B_lon[1][1],
                B_lon[2][0],
                B_lon[2][1],
                B_lon[3][0],
                B_lon[3][1],
                B_lon[4][0],
                B_lon[4][1],
            )
        )
        file.write(
            "A_lat = np.array([\n    [%f, %f, %f, %f, %f],\n    "
            "[%f, %f, %f, %f, %f],\n    "
            "[%f, %f, %f, %f, %f],\n    "
            "[%f, %f, %f, %f, %f],\n    "
            "[%f, %f, %f, %f, %f]])\n"
            % (
                A_lat[0][0],
                A_lat[0][1],
                A_lat[0][2],
                A_lat[0][3],
                A_lat[0][4],
                A_lat[1][0],
                A_lat[1][1],
                A_lat[1][2],
                A_lat[1][3],
                A_lat[1][4],
                A_lat[2][0],
                A_lat[2][1],
                A_lat[2][2],
                A_lat[2][3],
                A_lat[2][4],
                A_lat[3][0],
                A_lat[3][1],
                A_lat[3][2],
                A_lat[3][3],
                A_lat[3][4],
                A_lat[4][0],
                A_lat[4][1],
                A_lat[4][2],
                A_lat[4][3],
                A_lat[4][4],
            )
        )
        file.write(
            "B_lat = np.array([\n    [%f, %f],\n    "
            "[%f, %f],\n    "
            "[%f, %f],\n    "
            "[%f, %f],\n    "
            "[%f, %f]])\n"
            % (
                B_lat[0][0],
                B_lat[0][1],
                B_lat[1][0],
                B_lat[1][1],
                B_lat[2][0],
                B_lat[2][1],
                B_lat[3][0],
                B_lat[3][1],
                B_lat[4][0],
                B_lat[4][1],
            )
        )
        file.write("Ts = %f\n" % Ts)


def compute_tf_model(
    trim_state: types.DynamicState, trim_input: MsgDelta
) -> tuple[float, float, float, float, float, float, float, float, float, float, float]:
    """Compute the transfer function coefficients for the models in chapter 5.4

    Args:
        mav: dynamics of the mav
        trim_state: state of the trim trajectory
        trim_input: inputs for the trim trajectory

    Returns:
        Va_trim: Airspeed from the trim trajectory (assuming no wind)
        alpha_trim: Angle of attack as calculated from trim trajectory (assuming no wind)
        theta_trim: Pitch angle as calculated from the trim state

        a_phi1, a_phi2: Coefficients for phi transfer function (defined in 5.23 -5.24)

        a_theta1, a_theta2, a_theta3: Coefficients for the theta transfer function (defined prior to 5.29)

        a_V1, a_V2, a_V3: Coefficients for the airspeed transfer function (defined prior to 5.37)
    """
    # trim values
    (Va_trim, alpha_trim, _, _) = update_velocity_data(trim_state)
    quat = cast(Quaternion, trim_state[IND.QUAT])
    _, theta_trim, _ = Quaternion2Euler(quat)

    # define transfer function constants

    # equation (5.23) and (5.24)
    a_phi1 = (
        -0.5
        * MAV.rho
        * Va_trim ** 2
        * MAV.S_wing
        * MAV.b
        * MAV.C_p_p
        * MAV.b
        / 2.0
        / Va_trim
    )
    a_phi2 = 0.5 * MAV.rho * Va_trim ** 2 * MAV.S_wing * MAV.b * MAV.C_p_delta_a

    # defined prior to (5.29)
    a_theta1 = (
        -MAV.rho
        * Va_trim ** 2
        * MAV.c
        * MAV.S_wing
        / 2.0
        / MAV.Jy
        * MAV.C_m_q
        * MAV.c
        / 2.0
        / Va_trim
    )
    a_theta2 = (
        -MAV.rho * Va_trim ** 2 * MAV.c * MAV.S_wing / 2.0 / MAV.Jy * MAV.C_m_alpha
    )
    a_theta3 = (
        MAV.rho * Va_trim ** 2 * MAV.c * MAV.S_wing / 2.0 / MAV.Jy * MAV.C_m_delta_e
    )

    # Compute transfer function coefficients using new propulsion model  -- defined in equation before (5.37)
    a_V1 = (
        MAV.rho
        * Va_trim
        * MAV.S_wing
        / MAV.mass
        * (
            MAV.C_D_0
            + MAV.C_D_alpha * alpha_trim
            + MAV.C_D_delta_e * trim_input.elevator
        )
        - dT_dVa(Va_trim, trim_input.throttle) / MAV.mass
    )
    a_V2 = dT_ddelta_t(Va_trim, trim_input.throttle) / MAV.mass
    a_V3 = MAV.gravity * np.cos(theta_trim - alpha_trim)

    return (
        Va_trim,
        alpha_trim,
        theta_trim,
        a_phi1,
        a_phi2,
        a_theta1,
        a_theta2,
        a_theta3,
        a_V1,
        a_V2,
        a_V3,
    )


def compute_ss_model(
    trim_state: types.DynamicState, trim_input: MsgDelta
) -> tuple[npt.NDArray[Any], ...]:
    """Computes the state space model resulting from linearizing about the
    trim trajectory and input

    Args:
        trim_state: state of the trim trajectory
        trim_input: inputs for the trim trajectory

    Returns:
        (A_lon, B_lon): Longitudinal model defined in (5.51)
        (A_lat, B_lat): Lateral model defined in (5.44)
    """
    x_euler = euler_state(trim_state)
    A = df_dx(x_euler, trim_input)
    B = df_du(x_euler, trim_input)
    # extract longitudinal states (u, w, q, theta, pd)
    A_lon = A[np.ix_([3, 5, 10, 7, 2], [3, 5, 10, 7, 2])]
    B_lon = B[np.ix_([3, 5, 10, 7, 2], [0, 3])]
    # change pd to h
    for i in range(0, 5):
        A_lon[i, 4] = -A_lon[i, 4]
        A_lon[4, i] = -A_lon[4, i]
    for i in range(0, 2):
        B_lon[4, i] = -B_lon[4, i]
    # extract lateral states (v, p, r, phi, psi)
    A_lat = A[np.ix_([4, 9, 11, 6, 8], [4, 9, 11, 6, 8])]
    B_lat = B[np.ix_([4, 9, 11, 6, 8], [1, 2])]
    return A_lon, B_lon, A_lat, B_lat


def euler_state(x_quat: types.DynamicState) -> types.DynamicStateEuler:
    """
    convert state x with attitude represented by quaternion
    to x_euler with attitude represented by Euler angles
    """
    x_euler = np.zeros((12, 1))
    x_euler[0:6] = np.copy(x_quat[0:6])  # copy position, velocity
    phi, theta, psi = Quaternion2Euler(x_quat[IND.QUAT])
    x_euler[6] = phi
    x_euler[7] = theta
    x_euler[8] = psi
    x_euler[9:12] = np.copy(x_quat[10:13])  # copy angular rate
    return x_euler


def quaternion_state(x_euler: types.DynamicStateEuler) -> types.DynamicState:
    """
    convert state x_euler with attitude represented by Euler angles
    to x_quat with attitude represented by quaternions
    """
    x_quat = np.zeros((13, 1))
    x_quat[0:6] = np.copy(x_euler[0:6])  # copy position, velocity
    phi = x_euler.item(6)
    theta = x_euler.item(7)
    psi = x_euler.item(8)
    quat = Euler2Quaternion(phi, theta, psi)
    x_quat[6:10] = quat
    x_quat[10:13] = np.copy(x_euler[9:12])  # copy angular rate
    return x_quat


def f_euler(
    x_euler: types.DynamicStateEuler, delta: MsgDelta
) -> types.DynamicStateEuler:
    """
    return 12x1 dynamics (as if state were Euler state)
    compute f at euler_state
    """
    x_quat = quaternion_state(x_euler)
    x_quat_np = x_quat
    (Va, alpha, beta, _) = update_velocity_data(x_quat)
    f = derivatives(x_quat, forces_moments(x_quat, delta, Va, beta, alpha))
    # f_euler will be f, except for the attitude states
    f_euler_ = euler_state(f)

    # need to correct attitude states by multiplying f by
    # partial of Quaternion2Euler(quat) with respect to quat
    # compute partial Quaternion2Euler(quat) with respect to quat
    # dEuler/dt = dEuler/dquat * dquat/dt
    eps = 0.001  # epsilon used to approximate derivatives
    e = x_quat_np[IND.QUAT]  # pylint: disable=unsubscriptable-object
    phi = x_euler.item(6)
    theta = x_euler.item(7)
    psi = x_euler.item(8)
    dTheta_dquat = np.zeros((3, 4))
    for j in range(0, 4):
        tmp = np.zeros((4, 1))
        tmp[j][0] = eps
        # e_eps = (e + tmp)
        e_eps = (e + tmp) / np.linalg.norm(e + tmp)
        phi_eps, theta_eps, psi_eps = Quaternion2Euler(e_eps)
        dTheta_dquat[0][j] = (phi_eps - phi) / eps
        dTheta_dquat[1][j] = (theta_eps - theta) / eps
        dTheta_dquat[2][j] = (psi_eps - psi) / eps
    # Multiply f by
    # partial of Quaternion2Euler(quat) with respect to quat
    f_np = f
    f_euler_[6:9] = np.copy(
        dTheta_dquat @ f_np[6:10]
    )  # pylint: disable=unsubscriptable-object,unsupported-assignment-operation
    return f_euler_


def df_dx(x_euler: types.DynamicStateEuler, delta: MsgDelta) -> npt.NDArray[Any]:
    """
    take numeric partial of f_euler with respect to x_euler evaluated at (x_euler, delta)
    """

    eps = 0.01  # deviation
    A = np.zeros((12, 12))  # Jacobian of f wrt x
    f = f_euler(x_euler, delta)
    for i in range(0, 12):
        x_eps = np.copy(x_euler)
        x_eps[i][0] += eps
        f_eps = f_euler(x_eps, delta)
        df = (f_eps - f) / eps
        df_np = cast(npt.NDArray[Any], df)  # numpy version for type checking
        A[:, i] = df_np[:, 0]
    return A


def df_du(x_euler: types.DynamicStateEuler, delta: MsgDelta) -> npt.NDArray[Any]:
    """
    take numeric partial of f_euler with respect to input
    """
    eps = 0.01  # deviation
    B = np.zeros((12, 4))  # Jacobian of f wrt u
    f = f_euler(x_euler, delta)
    delta_eps = MsgDelta(
        elevator=delta.elevator,
        aileron=delta.aileron,
        rudder=delta.rudder,
        throttle=delta.throttle,
    )
    for i in range(0, 4):
        if i == 0:
            delta_eps.elevator += eps
            f_eps = f_euler(x_euler, delta_eps)
            delta_eps.elevator -= eps
        elif i == 1:
            delta_eps.aileron += eps
            f_eps = f_euler(x_euler, delta_eps)
            delta_eps.aileron -= eps
        elif i == 2:
            delta_eps.rudder += eps
            f_eps = f_euler(x_euler, delta_eps)
            delta_eps.rudder -= eps
        else:
            delta_eps.throttle += eps
            f_eps = f_euler(x_euler, delta_eps)
            delta_eps.throttle -= eps
        df = (f_eps - f) / eps
        df_np = cast(npt.NDArray[Any], df)  # numpy version for type checking
        B[:, i] = df_np[:, 0]
    return B


def dT_dVa(Va: float, delta_t: float) -> float:
    """
    returns the numeric derivative of motor thrust with respect to Va

    Args:
        Va: Airspeed
        delta_t: Throttle command

    Returns:
        dT_dVa: numeric partial of motor thrust wrt Va
    """
    eps = 0.01
    T_eps, _ = motor_thrust_torque(Va + eps, delta_t)
    T, _ = motor_thrust_torque(Va, delta_t)
    return (T_eps - T) / eps


def dT_ddelta_t(Va: float, delta_t: float) -> float:
    """
    returns the numeric derivative of motor thrust with respect to delta_t

    Args:
        Va: Airspeed
        delta_t: Throttle command

    Returns:
        dT_ddelta_t: numeric partial of motor thrust wrt delta_t
    """
    eps = 0.01
    T_eps, _ = motor_thrust_torque(Va, delta_t + eps)
    T, _ = motor_thrust_torque(Va, delta_t)
    return (T_eps - T) / eps

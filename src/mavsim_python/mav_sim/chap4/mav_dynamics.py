"""
mavDynamics
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state

part of mavPySim
    - Beard & McLain, PUP, 2012
    - Update history:
        12/20/2018 - RWB
"""
from typing import Optional, cast

import mav_sim.parameters.aerosonde_parameters as MAV
import numpy as np
import numpy.typing as npt

# load mav dynamics from previous chapter
from mav_sim.chap3.mav_dynamics import IND, DynamicState, ForceMoments, derivatives
from mav_sim.message_types.msg_delta import MsgDelta

# load message types
from mav_sim.message_types.msg_state import MsgState
from mav_sim.tools import types
from mav_sim.tools.rotations import Quaternion2Euler, Quaternion2Rotation


class MavDynamics:
    """Implements the dynamics of the MAV using vehicle inputs and wind"""

    def __init__(self, Ts: float, state: Optional[DynamicState] = None):
        self._ts_simulation = Ts
        # set initial states based on parameter file
        # _state is the 13x1 internal state of the aircraft that is being propagated:
        # _state = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]
        # We will also need a variety of other elements that are functions
        #   of the _state and the wind.
        # self.true_state is a 19x1 vector that is estimated and used by the
        #   autopilot to control the aircraft:
        # true_state = [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r,
        #   Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        if state is None:
            self._state = DynamicState().convert_to_numpy()
        else:
            self._state = state.convert_to_numpy()

        # store wind data for fast recall since it is used at various points in simulation
        self._wind = np.array([[0.0], [0.0], [0.0]])  # wind in NED frame in meters/sec

        # update velocity data
        (self._Va, self._alpha, self._beta, self._wind) = update_velocity_data(
            self._state
        )

        # Update forces and moments data
        self._forces = np.array(
            [[0.0], [0.0], [0.0]]
        )  # store forces to avoid recalculation in the sensors function (ch 7)
        self._moments = np.array(
            [[0.0], [0.0], [0.0]]
        )  # store moments to avoid recalculation
        forces_moments_vec = forces_moments(
            self._state, MsgDelta(), self._Va, self._beta, self._alpha
        )
        self._forces[0] = forces_moments_vec.item(0)
        self._forces[1] = forces_moments_vec.item(1)
        self._forces[2] = forces_moments_vec.item(2)
        self._moments[0] = forces_moments_vec.item(3)
        self._moments[1] = forces_moments_vec.item(4)
        self._moments[2] = forces_moments_vec.item(5)

        # initialize true_state message
        self.true_state = MsgState()
        self._update_true_state()

    ###################################
    # public functions
    def update(
        self, delta: MsgDelta, wind: types.WindVector, time_step: Optional[float] = None
    ) -> None:
        """
        Integrate the differential equations defining dynamics, update sensors

        Args:
            delta : (delta_a, delta_e, delta_r, delta_t) are the control inputs
            wind: the wind vector in inertial coordinates
        """
        # get forces and moments acting on rigid bod
        forces_moments_vec = forces_moments(
            self._state, delta, self._Va, self._beta, self._alpha
        )
        self._forces[0] = forces_moments_vec.item(0)
        self._forces[1] = forces_moments_vec.item(1)
        self._forces[2] = forces_moments_vec.item(2)
        self._moments[0] = forces_moments_vec.item(3)
        self._moments[1] = forces_moments_vec.item(4)
        self._moments[2] = forces_moments_vec.item(5)

        # Get the timestep
        if time_step is None:
            time_step = self._ts_simulation

        # Integrate ODE using Runge-Kutta RK4 algorithm
        k1 = derivatives(self._state, forces_moments_vec)
        k2 = derivatives(self._state + time_step / 2.0 * k1, forces_moments_vec)
        k3 = derivatives(self._state + time_step / 2.0 * k2, forces_moments_vec)
        k4 = derivatives(self._state + time_step * k3, forces_moments_vec)
        self._state += time_step / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        # normalize the quaternion
        e0 = self._state.item(IND.E0)
        e1 = self._state.item(IND.E1)
        e2 = self._state.item(IND.E2)
        e3 = self._state.item(IND.E3)
        norm_e = np.sqrt(e0 ** 2 + e1 ** 2 + e2 ** 2 + e3 ** 2)
        self._state[IND.E0][0] = self._state.item(IND.E0) / norm_e
        self._state[IND.E1][0] = self._state.item(IND.E1) / norm_e
        self._state[IND.E2][0] = self._state.item(IND.E2) / norm_e
        self._state[IND.E3][0] = self._state.item(IND.E3) / norm_e

        # update the airspeed, angle of attack, and side slip angles using new state
        (self._Va, self._alpha, self._beta, self._wind) = update_velocity_data(
            self._state, wind
        )

        # update the message class for the true state
        self._update_true_state()

    def external_set_state(self, new_state: types.DynamicState) -> None:
        """Loads a new state"""
        self._state = new_state

    def get_state(self) -> types.DynamicState:
        """Returns the state"""
        return self._state

    def get_struct_state(self) -> DynamicState:
        """Returns the current state in a struct format

        Outputs:
            DynamicState: The latest state of the mav
        """
        return DynamicState(self._state)

    def get_fm_struct(self) -> ForceMoments:
        """Returns the latest forces and moments calculated in dynamic update"""
        force_moment = np.zeros((6, 1))
        force_moment[0:3] = self._forces
        force_moment[3:6] = self._moments
        return ForceMoments(force_moment=cast(types.ForceMoment, force_moment))

    def get_euler(self) -> tuple[float, float, float]:
        """Returns the roll, pitch, and yaw Euler angles based upon the state"""
        # Get Euler angles
        phi, theta, psi = Quaternion2Euler(self._state[IND.QUAT])

        # Return angles
        return (phi, theta, psi)

    ###################################
    # private functions
    def _update_true_state(self) -> None:
        """update the class structure for the true state:

        [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi,
            gyro_bx, gyro_by, gyro_bz]
        """
        quat = cast(types.Quaternion, self._state[IND.QUAT])
        phi, theta, psi = Quaternion2Euler(quat)
        pdot = Quaternion2Rotation(quat) @ self._state[IND.VEL]
        self.true_state.north = self._state.item(IND.NORTH)
        self.true_state.east = self._state.item(IND.EAST)
        self.true_state.altitude = -self._state.item(IND.DOWN)
        self.true_state.Va = self._Va
        self.true_state.alpha = self._alpha
        self.true_state.beta = self._beta
        self.true_state.phi = phi
        self.true_state.theta = theta
        self.true_state.psi = psi
        self.true_state.Vg = cast(float, np.linalg.norm(pdot))
        if self.true_state.Vg != 0.0:
            self.true_state.gamma = np.arcsin(pdot.item(2) / self.true_state.Vg)
        else:
            self.true_state.gamma = 0.0
        self.true_state.chi = np.arctan2(pdot.item(1), pdot.item(0))
        self.true_state.p = self._state.item(IND.P)
        self.true_state.q = self._state.item(IND.Q)
        self.true_state.r = self._state.item(IND.R)
        self.true_state.wn = self._wind.item(0)
        self.true_state.we = self._wind.item(1)


def _sigma_blending_lift(alpha: float) -> float:
    """Sigma blending function for lift"""
    top = (
        1 + np.exp(-MAV.M * (alpha - MAV.alpha0)) + np.exp(MAV.M * (alpha + MAV.alpha0))
    )
    bottom = (1 + np.exp(-MAV.M * (alpha - MAV.alpha0))) * (
        1 + np.exp(MAV.M * (alpha + MAV.alpha0))
    )
    frac: float = top / bottom
    return frac


def cla(alpha: float) -> float:
    """Definition of cla term."""
    sigma = _sigma_blending_lift(alpha)
    coeff = 1 - sigma
    first = MAV.C_L_0 + MAV.C_L_alpha * alpha
    second = sigma * (2 * np.sign(alpha) * (np.sin(alpha)) ** 2 * np.cos(alpha))
    cl: float = coeff * first + second
    return cl


def cda(alpha: float) -> float:
    """Definition of cda term."""
    c_d_a: float = MAV.C_D_p + (MAV.C_L_0 + MAV.C_L_alpha * alpha) ** 2 / (
        np.pi * MAV.e * MAV.AR
    )
    return c_d_a


def forces_moments(
    state: types.DynamicState, delta: MsgDelta, Va: float, beta: float, alpha: float
) -> types.ForceMoment:
    """
    Return the forces on the UAV based on the state, wind, and control surfaces

    Args:
        state: current state of the aircraft
        delta: flap and thrust commands
        Va: Airspeed
        beta: Side slip angle
        alpha: Angle of attack


    Returns:
        Forces and Moments on the UAV (in body frame) np.matrix(fx, fy, fz, Mx,
            My, Mz)
    """
    # Unpacking variables  -----------------------------
    d_a, d_e, d_r = delta.aileron, delta.elevator, delta.rudder
    cos, sin = np.cos, np.sin
    st = DynamicState(state)
    quant = state[IND.QUAT]
    m, g = MAV.gravity, MAV.mass
    b, c = MAV.b, MAV.c
    # --------------------------------------------------

    # Forces #############################################################

    # Force of Gravity ---------------------------------
    e0, ex, ey, ez = quant.item(0), quant.item(1), quant.item(2), quant.item(3)
    fbg: npt.NDArray[np.float64] = np.array(
        [
            [2 * (ex * ez - ey * e0)],
            [2 * (ey * ez + ex * e0)],
            [ez ** 2 + e0 ** 2 - ex ** 2 - ey ** 2],
        ]
    )
    mg = m * g
    fbg_2 = fbg * mg
    # --------------------------------------------------

    # Aerodynamic Force Coefficents --------------------
    if Va != 0:
        C_L_A = cla(alpha)
        C_D_A = cda(alpha)
        coeff = 0.5 * MAV.rho * Va ** 2 * MAV.S_wing
        c_x_a = -C_D_A * cos(alpha) + C_L_A * sin(alpha)
        c_x_q = -MAV.C_D_q * cos(alpha) + MAV.C_L_q * sin(alpha)
        c_x_d_e = -MAV.C_D_delta_e * cos(alpha) + MAV.C_L_delta_e * sin(alpha)
        c_z_a = -C_D_A * sin(alpha) - C_L_A * cos(alpha)
        c_z_d = -MAV.C_D_delta_e * sin(alpha) - MAV.C_L_delta_e * cos(alpha)
        c_z_q = -MAV.C_D_q * sin(alpha) - MAV.C_L_q * cos(alpha)
        c_y_0, c_y_b, c_y_p, c_y_r = MAV.C_Y_0, MAV.C_Y_beta, MAV.C_Y_p, MAV.C_Y_r
        c_y_d_r, c_y_d_a = MAV.C_Y_delta_r, MAV.C_Y_delta_a
        # --------------------------------------------------
        # Aerodynamic Forces --------------------------------
        long_forces: npt.NDArray[np.float64] = np.array(
            [
                [c_x_a + c_x_q * (c / (2 * Va)) * st.q],
                [
                    c_y_0
                    + c_y_b * beta
                    + c_y_p * (b / (2 * Va) * st.p + c_y_r * (b / (2 * Va) * st.r))
                ],
                [c_z_a + c_z_q * (c / (2 * Va)) * st.q],
            ]
        )
        lat_forces = np.array(
            [[c_x_d_e * d_e], [c_y_d_r * d_r + c_y_d_a * d_a], [c_z_d * d_e]]
        )
    # --------------------------------------------------

    # Motor  -------------------------------------------
    thrust, tourqe = motor_thrust_torque(Va, delta.throttle)
    # --------------------------------------------------

    # Motor Forces -------------------------------------
    tourqe_array = np.array([[thrust], [0], [0]])
    # --------------------------------------------------

    # Total Forces -------------------------------------
    if Va == 0:
        forces = fbg_2 + tourqe_array
    else:
        forces = fbg_2 + coeff * long_forces + coeff * lat_forces + tourqe_array
    fx = forces.item(0)
    fy = forces.item(1)
    fz = forces.item(2)
    # --------------------------------------------------

    # Moments ############################################################

    # Aerodynamic Moment Coefficents -------------------
    C_l_0, C_l_b, C_l_p, C_l_r = MAV.C_ell_0, MAV.C_ell_beta, MAV.C_ell_p, MAV.C_ell_r
    C_l_d_a, C_l_d_r = MAV.C_ell_delta_a, MAV.C_ell_delta_r
    C_m_0, C_m_a, C_m_q, C_m_d_e = MAV.C_m_0, MAV.C_m_alpha, MAV.C_m_q, MAV.C_m_delta_e
    C_n_0, C_n_b, C_n_p, C_n_r = MAV.C_n_0, MAV.C_n_beta, MAV.C_n_p, MAV.C_n_r
    C_n_d_a, C_n_d_r = MAV.C_n_delta_a, MAV.C_n_delta_r
    # --------------------------------------------------

    # Longitudinal Aerodynamic Moments ----------------------
    thrust_vec = np.array([[-tourqe], [0], [0]])
    if Va != 0:
        top_mom = b * (
            C_l_0
            + C_l_b * beta
            + C_l_p * (b / (2 * Va)) * st.p
            + C_l_r * (b / (2 * Va)) * st.r
        )
        mid_mom = c * (C_m_0 + C_m_a * alpha + C_m_q * (c / (2 * Va)) * st.q)
        bot_mom = b * (
            C_n_0
            + C_n_b * beta
            + C_n_p * (b / (2 * Va)) * st.p
            + C_n_r * (b / (2 * Va)) * st.r
        )
        long_moments = np.array([[top_mom], [mid_mom], [bot_mom]])
        lat_mom = np.array(
            [
                [b * (C_l_d_a * d_a + C_l_d_r * d_r)],
                [c * (C_m_d_e * d_e)],
                [b * (C_n_d_a * d_a + C_n_d_r * d_r)],
            ]
        )
        moment_vec = coeff * (long_moments + lat_mom) + thrust_vec
    else:
        moment_vec = thrust_vec
    Mx = moment_vec.item(0)
    My = moment_vec.item(1)
    Mz = moment_vec.item(2)
    # forcem = np.array([[fx], [fy], [fz], [Mx], [My], [Mz]])

    return types.ForceMoment(np.array([[fx, fy, fz, Mx, My, Mz]]).T)


def motor_thrust_torque(Va: float, delta_t: float) -> tuple[float, float]:
    """compute thrust and torque due to propeller  (See addendum by McLain)

    Args:
        Va: Airspeed
        delta_t: Throttle command

    Returns:
        T_p: Propeller thrust
        Q_p: Propeller torque
    """
    # thrust and torque due to propeller
    V_i = MAV.V_max * delta_t
    R = MAV.R_motor
    rho = MAV.rho
    D = MAV.D_prop
    a = (rho * (D ** 5)) / ((2 * np.pi) ** 2) * MAV.C_Q0
    b = ((rho * (D ** 4)) / (2 * np.pi)) * (MAV.C_Q1 * Va) + (MAV.KQ * MAV.KQ) / R
    c = rho * D ** 3 * MAV.C_Q2 * (Va ** 2) - MAV.KQ / R * V_i + MAV.KQ * MAV.i0
    prop_speed = (-b + np.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
    J_op = (2 * np.pi * Va) / (prop_speed * D)
    C_q = MAV.C_Q0 + MAV.C_Q1 * J_op + MAV.C_Q2 * J_op ** 2
    C_t = MAV.C_T0 + MAV.C_T1 * J_op + MAV.C_T2 * J_op ** 2
    thrust_prop = (rho * D ** 4) / (4 * np.pi ** 2) * prop_speed ** 2 * C_t
    torque_prop = (rho * D ** 5) / (4 * np.pi ** 2) * prop_speed ** 2 * C_q
    return thrust_prop, torque_prop


def update_velocity_data(
    state: types.DynamicState,
    wind: types.WindVector = types.WindVector(np.zeros((6, 1))),
) -> tuple[float, float, float, types.NP_MAT]:
    """Calculates airspeed, angle of attack, sideslip, and velocity wrt wind.

    Args:
        state: current state of the aircraft

    Returns:
        Va: Airspeed
        alpha: Angle of attack
        beta: Side slip angle
        wind_inertial_frame: Wind vector in inertial frame
    """
    st = DynamicState(state)
    # alt = -st.down
    steady_state = wind[0:3]
    gust = wind[3:6]

    # convert wind vector from world to body frame
    R = Quaternion2Rotation(state[IND.QUAT])  # rotation from body to world frame
    wind_body_frame = R.T @ steady_state  # rotate steady state wind to body frame
    wind_body_frame += gust  # add the gust
    wind_inertial_frame = R @ wind_body_frame  # Wind in the world frame

    speed = np.array([[st.u], [st.v], [st.w]])
    relative_speed = speed - wind_body_frame

    # compute airspeed
    Va = np.linalg.norm(relative_speed)

    # compute angle of attack
    alpha = np.arctan2(relative_speed.item(2), relative_speed.item(0))

    # compute sideslip angle
    # numerator = float(relative_speed.item(1))
    beta = np.arctan2(
        relative_speed.item(1),
        np.sqrt(relative_speed.item(0) ** 2 + relative_speed.item(2) ** 2),
    )

    # Return computed values
    return (Va, alpha, beta, wind_inertial_frame)  # type: ignore

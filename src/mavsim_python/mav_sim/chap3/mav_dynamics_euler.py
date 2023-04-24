"""
mav_dynamics
    - this file implements the dynamic equations of motion for MAV using Euler coordinates
    - use Euler angles for the attitude state

part of mavsimPy
    - Beard & McLain, PUP, 2012
    - Update history:
        12/17/2018 - RWB
        1/14/2019 - RWB
        12/21 - GND
        12/22 - GND
"""
import mav_sim.parameters.aerosonde_parameters as MAV
import numpy as np
import numpy.typing as npt
from mav_sim.chap3.mav_dynamics import IND, ForceMoments
from mav_sim.tools import types
from mav_sim.tools.rotations import Euler2Quaternion, Quaternion2Euler


# Indexing constants for state using Euler representation
class StateIndicesEuler:
    """Constant class for easy access of state indices"""

    NORTH: int = 0  # North position
    EAST: int = 1  # East position
    DOWN: int = 2  # Down position
    U: int = 3  # body-x velocity
    V: int = 4  # body-y velocity
    W: int = 5  # body-z velocity
    PHI: int = 6  # Roll angle (about x-axis)
    THETA: int = 7  # Pitch angle (about y-axis)
    PSI: int = 8  # Yaw angle (about z-axis)
    P: int = 9  # roll rate - body frame - i
    Q: int = 10  # pitch rate - body frame - j
    R: int = 11  # yaw rate - body frame - k
    VEL: list[int] = [U, V, W]  # Body velocity indices
    ANG_VEL: list[int] = [P, Q, R]  # Body rotational velocities
    NUM_STATES: int = 12  # Number of states


IND_EULER = StateIndicesEuler()

# Conversion functions
def euler_state_to_quat_state(
    state_euler: types.DynamicStateEuler,
) -> types.DynamicState:
    """Converts an Euler state representation to a quaternion state representation

    Args:
        state_euler: The state vector to be converted to a quaternion representation

    Returns:
        state_quat: The converted state
    """
    # Create the quaternion from the euler coordinates
    e = Euler2Quaternion(
        phi=state_euler[IND_EULER.PHI],
        theta=state_euler[IND_EULER.THETA],
        psi=state_euler[IND_EULER.PSI],
    )

    # Copy over data
    state_quat = np.zeros((IND.NUM_STATES, 1))
    state_quat[IND.NORTH] = state_euler.item(IND_EULER.NORTH)
    state_quat[IND.EAST] = state_euler.item(IND_EULER.EAST)
    state_quat[IND.DOWN] = state_euler.item(IND_EULER.DOWN)
    state_quat[IND.U] = state_euler.item(IND_EULER.U)
    state_quat[IND.V] = state_euler.item(IND_EULER.V)
    state_quat[IND.W] = state_euler.item(IND_EULER.W)
    state_quat[IND.E0] = e.item(0)
    state_quat[IND.E1] = e.item(1)
    state_quat[IND.E2] = e.item(2)
    state_quat[IND.E3] = e.item(3)
    state_quat[IND.P] = state_euler.item(IND_EULER.P)
    state_quat[IND.Q] = state_euler.item(IND_EULER.Q)
    state_quat[IND.R] = state_euler.item(IND_EULER.R)

    return state_quat


def quat_state_to_euler_state(
    state_quat: types.DynamicState,
) -> types.DynamicStateEuler:
    """Converts a quaternion state representation to an Euler state representation

    Args:
        state_quat: The state vector to be converted

    Returns
        state_euler: The converted state
    """
    # Create the quaternion from the euler coordinates
    phi, theta, psi = Quaternion2Euler(state_quat[IND.QUAT])

    # Copy over data
    state_euler = np.zeros((IND_EULER.NUM_STATES, 1))
    state_euler[IND_EULER.NORTH] = state_quat.item(IND.NORTH)
    state_euler[IND_EULER.EAST] = state_quat.item(IND.EAST)
    state_euler[IND_EULER.DOWN] = state_quat.item(IND.DOWN)
    state_euler[IND_EULER.U] = state_quat.item(IND.U)
    state_euler[IND_EULER.V] = state_quat.item(IND.V)
    state_euler[IND_EULER.W] = state_quat.item(IND.W)
    state_euler[IND_EULER.PHI] = phi
    state_euler[IND_EULER.THETA] = theta
    state_euler[IND_EULER.PSI] = psi
    state_euler[IND_EULER.P] = state_quat.item(IND.P)
    state_euler[IND_EULER.Q] = state_quat.item(IND.Q)
    state_euler[IND_EULER.R] = state_quat.item(IND.R)

    return state_euler


class DynamicStateEuler:
    """Struct for the dynamic state"""

    def __init__(self, state: types.DynamicStateEuler) -> None:
        self.north: float  # North position
        self.east: float  # East position
        self.down: float  # Down position
        self.u: float  # body-x velocity
        self.v: float  # body-y velocity
        self.w: float  # body-z velocity
        self.phi: float  # roll angle (about x-axis)
        self.theta: float  # pitch angle (about y-axis)
        self.psi: float  # yaw angle (about z-axis)
        self.p: float  # roll rate - body frame - i
        self.q: float  # pitch rate - body frame - j
        self.r: float  # yaw rate - body frame - k

        self.extract_state(state)

    def extract_state(self, state: types.DynamicStateEuler) -> None:
        """Initializes the state variables

        Args:
            state: State from which to extract the state values

        """
        self.north = state.item(IND_EULER.NORTH)
        self.east = state.item(IND_EULER.EAST)
        self.down = state.item(IND_EULER.DOWN)
        self.u = state.item(IND_EULER.U)
        self.v = state.item(IND_EULER.V)
        self.w = state.item(IND_EULER.W)
        self.phi = state.item(IND_EULER.PHI)
        self.theta = state.item(IND_EULER.THETA)
        self.psi = state.item(IND_EULER.PSI)
        self.p = state.item(IND_EULER.P)
        self.q = state.item(IND_EULER.Q)
        self.r = state.item(IND_EULER.R)

    def convert_to_numpy(self) -> types.DynamicStateEuler:
        """Converts the state to a numpy object"""
        output = np.empty((IND_EULER.NUM_STATES, 1))
        output[IND_EULER.NORTH, 0] = self.north
        output[IND_EULER.EAST, 0] = self.east
        output[IND_EULER.DOWN, 0] = self.down
        output[IND_EULER.U, 0] = self.u
        output[IND_EULER.V, 0] = self.v
        output[IND_EULER.W, 0] = self.w
        output[IND_EULER.PHI, 0] = self.phi
        output[IND_EULER.THETA, 0] = self.theta
        output[IND_EULER.PSI, 0] = self.psi
        output[IND_EULER.P, 0] = self.p
        output[IND_EULER.Q, 0] = self.q
        output[IND_EULER.R, 0] = self.r

        return types.DynamicState(output)


def derivatives_euler(
    state: types.DynamicStateEuler, forces_moments: types.ForceMoment
) -> types.DynamicStateEuler:
    """Implements the dynamics xdot = f(x, u) where u is the force/moment vector

    Args:
        state: Current state of the vehicle
        forces_moments: 6x1 array containing [fx, fy, fz, Mx, My, Mz]^T

    Returns:
        Time derivative of the state ( f(x,u), where u is the force/moment vector )
    """
    # collect the derivative of the states
    st = DynamicStateEuler(state)
    fm = ForceMoments(forces_moments)
    x_dot = np.empty((IND_EULER.NUM_STATES, 1))
    phi, theta, psi = st.phi, st.theta, st.psi
    c, s, t = np.cos, np.sin, np.tan

    translation_angle: npt.NDArray[np.float64] = np.array(
        [
            [
                c(theta) * c(psi),
                s(phi) * s(theta) * c(psi) - c(phi) * s(psi),
                c(phi) * s(theta) * c(psi) + s(phi) * s(psi),
            ],
            [
                c(theta) * s(psi),
                s(phi) * s(theta) * s(psi) + c(phi) * c(psi),
                c(phi) * s(theta) * s(psi) - s(phi) * c(psi),
            ],
            [-s(theta), s(phi) * c(theta), c(phi) * c(theta)],
        ]
    )
    u, v, w = st.u, st.v, st.w
    pn_pe_pd = translation_angle @ np.array([u, v, w])
    x_dot[IND_EULER.NORTH] = pn_pe_pd.item(0)
    x_dot[IND_EULER.EAST] = pn_pe_pd.item(1)
    x_dot[IND_EULER.DOWN] = pn_pe_pd.item(2)
    p, q, r = st.p, st.q, st.r
    diff_vec = np.array([r * v - q * w, p * w - r * u, q * u - p * v])
    mass = MAV.mass
    f1, f2, f3 = fm.fx, fm.fy, fm.fz
    force_vec = np.array([f1, f2, f3])
    u_v_w = diff_vec + (1 / mass) * force_vec
    # print(f"u_v_w: {u_v_w.shape}")
    x_dot[IND_EULER.U] = u_v_w.item(0)
    x_dot[IND_EULER.V] = u_v_w.item(1)
    x_dot[IND_EULER.W] = u_v_w.item(2)
    rot_mat = np.array(
        [
            [1, s(phi) * t(theta), c(phi) * t(theta)],
            [0, c(phi), -s(phi)],
            [0, s(phi) / c(theta), c(phi) / c(theta)],
        ]
    )
    ptp_dot = rot_mat @ np.array([[p], [q], [r]])

    x_dot[IND_EULER.PHI] = ptp_dot.item(0)
    x_dot[IND_EULER.THETA] = ptp_dot.item(1)
    x_dot[IND_EULER.PSI] = ptp_dot.item(2)

    pqr1 = np.array(
        [
            [MAV.gamma1 * p * q - MAV.gamma2 * q * r],
            [MAV.gamma5 * p * r - MAV.gamma6 * (p ** 2 - r ** 2)],
            [MAV.gamma7 * p * q - MAV.gamma1 * q * r],
        ]
    )
    l, m, n = fm.l, fm.m, fm.n
    pqr2 = np.array(
        [
            [MAV.gamma3 * l + MAV.gamma4 * n],
            [1 / MAV.Jy * m],
            [MAV.gamma4 * l + MAV.gamma8 * n],
        ]
    )
    pqr_dot = pqr1 + pqr2

    x_dot[IND_EULER.P] = pqr_dot.item(0)
    x_dot[IND_EULER.Q] = pqr_dot.item(1)
    x_dot[IND_EULER.R] = pqr_dot.item(2)

    return x_dot

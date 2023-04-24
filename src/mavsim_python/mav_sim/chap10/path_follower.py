"""path_follower.py implements a class for following a path with a mav
"""

import numpy as np
import numpy.typing as npt
from mav_sim.message_types.msg_autopilot import MsgAutopilot
from mav_sim.message_types.msg_path import MsgPath
from mav_sim.message_types.msg_state import MsgState
from mav_sim.tools.wrap import wrap


class PathFollower:
    """Class for path following"""

    def __init__(self) -> None:
        """Initialize path following class"""
        # approach angle for large distance from straight-line path
        self.chi_inf = np.radians(50)
        # path gain for straight-line path following
        self.k_path = 0.01  # 0.05
        # path gain for orbit following
        self.k_orbit = 1.0  # 10.0
        self.gravity = 9.8
        self.autopilot_commands = MsgAutopilot()  # message sent to autopilot

    def update(self, path: MsgPath, state: MsgState) -> MsgAutopilot:
        """Update the control for following the path

        Args:
            path: path to be followed
            state: current state of the mav

        Returns:
            autopilot_commands: Commands to autopilot for following the path
        """
        if path.type == "line":
            self.autopilot_commands = follow_straight_line(
                path=path, state=state, k_path=self.k_path, chi_inf=self.chi_inf
            )
        elif path.type == "orbit":
            self.autopilot_commands = follow_orbit(
                path=path, state=state, k_orbit=self.k_orbit, gravity=self.gravity
            )
        return self.autopilot_commands


def follow_straight_line(
    path: MsgPath, state: MsgState, k_path: float, chi_inf: float
) -> MsgAutopilot:
    """Calculate the autopilot commands for following a straight line

    Args:
        path: straight-line path to be followed
        state: current state of the mav
        k_path: convergence gain for converging to the path
        chi_inf: Angle to take towards path when at an infinite distance from
            the path

    Returns:
        autopilot_commands: the commands required for executing the desired line
    """
    # Initialize the output
    autopilot_commands = MsgAutopilot()

    # Create autopilot commands here

    # unit vector in the direction of the path
    direction = path.line_direction
    # point on the line
    direction = np.array([direction.item(0), direction.item(1), direction.item(2)])
    r = path.line_origin
    r = np.array([r.item(0), r.item(1), r.item(2)])
    # current position
    p = np.array([state.north, state.east, state.altitude])
    # calculate deviance from path
    chi_q = wrap(np.arctan2(direction.item(1), direction.item(0)), state.chi)

    # --- Tracking

    # ------ Lateral
    # Rotation from veicle frame to path frame
    # error in path frame
    e_p: npt.NDArray[np.float64] = p - r
    # Desired course angle
    e_p_y = -np.sin(chi_q) * (state.north - r.item(0)) + np.cos(chi_q) * (
        state.east - r.item(1)
    )
    chi_c = chi_q - chi_inf * 2 / np.pi * np.arctan2(k_path * e_p_y, 1)

    # ------ Longitudinal

    # get normal unit vector of path q and k plane
    n_k_q = np.array([direction.item(1), -direction.item(0), 0])
    n = n_k_q / np.linalg.norm(n_k_q)
    dot_ep_n = sum(x_i * y_i for x_i, y_i in zip(e_p, n))
    s = e_p - (dot_ep_n * n)
    # similar triangles between r and q to s and r
    tri = direction.item(2) / np.sqrt(direction.item(0) ** 2 + direction.item(1) ** 2)
    # desired altitude
    h_d: float = -r.item(2) - np.linalg.norm(s[0:1]) * tri
    # altitude error

    # implement closed loop altitude control to find appropriate height command
    autopilot_commands.airspeed_command = path.airspeed
    autopilot_commands.course_command = chi_c
    autopilot_commands.altitude_command = h_d
    return autopilot_commands


def follow_orbit(
    path: MsgPath, state: MsgState, k_orbit: float, gravity: float
) -> MsgAutopilot:
    """Calculate the autopilot commands for following a circular path

    Args:
        path: circular orbit to be followed
        state: current state of the mav
        k_orbit: Convergence gain for reducing error to orbit
        gravity: Gravity constant

    Returns:
        autopilot_commands: the commands required for executing the desired orbit
    """

    # Initialize the output
    autopilot_commands = MsgAutopilot()
    d = np.sqrt(
        (state.east - path.orbit_center.item(1)) ** 2
        + (state.north - path.orbit_center.item(0)) ** 2
    )
    angle_from_aircraft = np.arctan2(
        state.east - path.orbit_center.item(1), state.north - path.orbit_center.item(0)
    )
    angle_from_aircraft = wrap(angle_from_aircraft, state.chi)
    h_c = -path.orbit_center.item(2)
    lambda_ = 1 if path.orbit_direction == "CW" else -1
    chi_c = angle_from_aircraft + lambda_ * (
        np.pi / 2 + np.arctan2(k_orbit * (d - path.orbit_radius), path.orbit_radius)
    )
    feed_forward = lambda_ * np.arctan2(state.Va ** 2, gravity * path.orbit_radius)
    autopilot_commands.course_command = chi_c
    autopilot_commands.altitude_command = h_c
    autopilot_commands.airspeed_command = path.airspeed
    autopilot_commands.phi_feedforward = (
        0 if (d - path.orbit_radius) / path.orbit_radius > 10 else feed_forward
    )

    return autopilot_commands

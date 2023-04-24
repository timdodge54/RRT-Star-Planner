"""Provides an implementation of the fillet path manager for waypoint following as described in
   Chapter 11 Algorithm 8
"""

import numpy as np
from mav_sim.chap11.path_manager_utilities import (
    EPSILON,
    HalfSpaceParams,
    WaypointIndices,
    extract_waypoints,
    get_airspeed,
    inHalfSpace,
)
from mav_sim.message_types.msg_path import MsgPath
from mav_sim.message_types.msg_state import MsgState
from mav_sim.message_types.msg_waypoints import MsgWaypoints
from mav_sim.tools.types import NP_MAT


def fillet_manager(
    state: MsgState,
    waypoints: MsgWaypoints,
    ptr_prv: WaypointIndices,
    path_prv: MsgPath,
    hs_prv: HalfSpaceParams,
    radius: float,
    manager_state: int,
) -> tuple[MsgPath, HalfSpaceParams, WaypointIndices, int]:

    """Update for the fillet manager.
       Updates state machine if the MAV enters into the next halfspace.

    Args:
        state: current state of the vehicle
        waypoints: The waypoints to be followed
        ptr_prv: The indices that were being used on the previous iteration
                (i.e., current waypoint inidices being followed when manager
                called)
        hs_prv: The previous halfspace being looked for (i.e., the current
                halfspace when manager called)
        radius: minimum radius circle for the mav
        manager_state: Integer state of the manager
                Value of 1 corresponds to following the straight line path
                Value of 2 corresponds to following the arc between straight
                lines

    Returns:
        path (MsgPath): Path to be followed
        hs (HalfSpaceParams): Half space parameters corresponding to the next
                change in state
        ptr (WaypointIndices): Indices of the current waypoint being followed
        manager_state (int): The current state of the manager

    """
    # Default the outputs to be the inputs
    path = path_prv
    hs = hs_prv
    ptr = ptr_prv
    pos = np.array([[state.north], [state.east], [-state.altitude]])
    # Insert code here
    if waypoints.flag_waypoints_changed and waypoints.num_waypoints >= 3:
        ptr = WaypointIndices()
        waypoints.flag_waypoints_changed = False
        manager_state = 1
    if manager_state == 1:
        path_line, hs = construct_fillet_line(
            waypoints=waypoints, ptr=ptr, radius=radius
        )
        path = path_line
        if inHalfSpace(pos, hs):
            manager_state = 2
    if manager_state == 2:
        path_cir, hs = construct_fillet_circle(
            waypoints=waypoints, ptr=ptr, radius=radius
        )
        path = path_cir
        if inHalfSpace(pos, hs):
            ptr.increment_pointers(waypoints.num_waypoints)
            manager_state = 1
    return (path, hs, ptr, manager_state)


def construct_fillet_line(
    waypoints: MsgWaypoints, ptr: WaypointIndices, radius: float
) -> tuple[MsgPath, HalfSpaceParams]:
    """Define the line on a fillet and a halfspace for switching to the next fillet curve.

    The line is created from the previous and current waypoints with halfspace defined for
    switching once a circle of the specified radius can be used to transition to the next line segment.

    Args:
        waypoints: The waypoints to be followed
        ptr: The indices of the waypoints being used for the path
        radius: minimum radius circle for the mav

    Returns:
        path: The straight-line path to be followed
        hs: The halfspace for switching to the next waypoint
    """
    # Extract the waypoints (w_{i-1}, w_i, w_{i+1})
    previous, current, next_wp = extract_waypoints(waypoints=waypoints, ptr=ptr)
    current_line: NP_MAT = current - previous
    current_line_mag = np.linalg.norm(current - previous)
    current_line = current_line / current_line_mag
    next_line = (next_wp - current) / np.linalg.norm(next_wp - current)
    var_rho = np.arccos(-current_line.T @ next_line)
    if var_rho % (2 * np.pi) == 0:
        Z = current
    else:
        Z = current - radius / (np.tan(var_rho / 2)) * current_line

    # Construct the path
    path = MsgPath()
    path.plot_updated = False
    path.line_direction = current_line
    path.line_origin = previous
    path.airspeed = get_airspeed(waypoints=waypoints, ptr=ptr)

    # Construct the halfspace
    hs = HalfSpaceParams(point=Z, normal=current_line)

    return (path, hs)


def construct_fillet_circle(
    waypoints: MsgWaypoints, ptr: WaypointIndices, radius: float
) -> tuple[MsgPath, HalfSpaceParams]:
    """Define the circle on a fillet

    Args:
        waypoints: The waypoints to be followed
        ptr: The indices of the waypoints being used for the path
        radius: minimum radius circle for the mav

    Returns:
        path: The straight-line path to be followed
        hs: The halfspace for switching to the next waypoint
    """
    # Extract the waypoints (w_{i-1}, w_i, w_{i+1})
    previous, current, next_wp = extract_waypoints(waypoints=waypoints, ptr=ptr)
    # Define lines ---
    current_line: NP_MAT = current - previous
    current_line_mag = np.linalg.norm(current - previous)
    current_line = current_line / current_line_mag
    next_line: NP_MAT = next_wp - current
    next_line_mag = np.linalg.norm(next_wp - current)

    next_line = next_line / next_line_mag
    var_rho = np.arccos(-current_line.T @ next_line)
    vec_center: NP_MAT = current_line - next_line
    vec_center_mag = np.linalg.norm(vec_center)
    lambda_ = np.sign(
        current_line.item(0) * next_line.item(1)
        - current_line.item(1) * next_line.item(0)
    )
    if var_rho % (2 * np.pi) == 0 or vec_center_mag < EPSILON:
        hs = HalfSpaceParams(normal=current_line, point=current)
        # J = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        J = np.array([[0, -1, 0], [1, 0, 0], [0, 0, -1]])
        C = current + radius * J @ current_line
    else:
        vec_center = vec_center / vec_center_mag
        C = current - (radius / np.sin(var_rho / 2)) * vec_center
        Z = current + radius / (np.tan(var_rho / 2)) * next_line
        # Define the switching halfspace
        hs = HalfSpaceParams(normal=next_line, point=Z)

    # Construct the path
    path = MsgPath()
    path.plot_updated = False
    path.type = "orbit"
    path.orbit_center = C
    path.orbit_direction = "CW" if lambda_ > 0 else "CCW"
    path.orbit_radius = radius

    path.airspeed = get_airspeed(waypoints=waypoints, ptr=ptr)

    return (path, hs)

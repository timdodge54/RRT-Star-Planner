"""Provides an implementation of the straight line path manager for waypoint following as described in
   Chapter 11 Algorithm 7
"""

import numpy as np
from mav_sim.chap11.path_manager_utilities import (
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


def line_manager(
    state: MsgState,
    waypoints: MsgWaypoints,
    ptr_prv: WaypointIndices,
    path_prv: MsgPath,
    hs_prv: HalfSpaceParams,
) -> tuple[MsgPath, HalfSpaceParams, WaypointIndices]:
    """Update for the line manager. Only updates the path and next halfspace under two conditions:
        1) The waypoints are new
        2) In a new halfspace

    Args:
        state: current state of the vehicle
        waypoints: The waypoints to be followed
        ptr_prv: The indices of the waypoints being used for the previous path
        path_prv: The previously commanded path
        hs_prv: The currently active halfspace for switching

    Returns:
        path: The updated path to follow
        hs: The updated halfspace for the next switch
        ptr: The updated index pointer
    """
    # Default the outputs to be the inputs
    path = path_prv
    hs = hs_prv
    ptr = ptr_prv

    if waypoints.flag_waypoints_changed and waypoints.num_waypoints >= 3:
        ptr = WaypointIndices()
        waypoints.flag_waypoints_changed = False
    path, hs = construct_line(waypoints=waypoints, ptr=ptr)
    pne: NP_MAT = np.array([[state.north], [state.east], [-state.altitude]])
    # path, hs = construct_line(waypoints=waypoints, ptr=ptr)
    if inHalfSpace(pne, hs):
        ptr.increment_pointers(waypoints.num_waypoints)
    # Output the updated path, halfspace, and index pointer
    return (path, hs, ptr)


def construct_line(
    waypoints: MsgWaypoints, ptr: WaypointIndices
) -> tuple[MsgPath, HalfSpaceParams]:
    """Creates a line and switching halfspace. The halfspace assumes that the aggregate
       path will consist of a series of straight lines.

    The line is created from the previous and current waypoints with halfspace defined for
    switching once the current waypoint is reached.

    Args:
        waypoints: The waypoints from which to construct the path
        ptr: The indices of the waypoints being used for the path

    Returns:
        path: The straight-line path to be followed
        hs: The halfspace for switching to the next waypoint
    """

    # Extract the waypoints (w_{i-1}, w_i, w_{i+1})
    previous, current, next_wp = extract_waypoints(waypoints=waypoints, ptr=ptr)
    # define the lines
    current_non: NP_MAT = current - previous
    current_mag = np.linalg.norm(current - previous)
    current_line = current_non / current_mag
    next_line_non: NP_MAT = next_wp - current
    next_line_mag = np.linalg.norm(next_wp - current)
    next_line = next_line_non / next_line_mag
    # define the halfspace
    norm = current_line + next_line
    mag_norm = np.linalg.norm(current_line + next_line)
    if mag_norm == 0:
        norm = current_line
    else:
        norm = norm / mag_norm
    # Construct the halfspace
    hs = HalfSpaceParams(normal=norm, point=current)

    # Construct the path
    path = MsgPath()
    path.plot_updated = False
    path.line_direction = current_line
    path.line_origin = previous
    path.airspeed = get_airspeed(waypoints=waypoints, ptr=ptr)
    return (path, hs)

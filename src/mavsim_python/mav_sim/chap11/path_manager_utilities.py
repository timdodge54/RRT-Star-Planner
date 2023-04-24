"""Provides utility functions for working with the waypoints
"""

import numpy as np
from mav_sim.message_types.msg_waypoints import MsgWaypoints
from mav_sim.tools.types import NP_MAT

IND_OOB = (
    9999  # Out of bounds index - used when waypoint index goes beyond number of indices
)
EPSILON = 0.0000001  # A very small number


class WaypointIndices:
    """Stores pointers to the indices to the previous, current, and next waypoints"""

    def __init__(self) -> None:
        self.previous = 0  # Previous waypoint
        self.current = 1  # Waypoint currently navigating towards
        self.next = 2  # The waypoint that will come after the current

    def increment_pointers(self, num_waypoints: int) -> None:
        """Updates the pointers to point at the next index. A value of IND_OOB is assigned
           to pointers when the number of waypoints has been exceeded

        Args:
            num_waypoints: Maximum index allowed. A value of IND_OOB is assigned when index >= this value
        """
        self.previous = self.current
        self.current = self.next
        self.next = self.next + 1
        if self.next > num_waypoints - 1:
            self.next = IND_OOB
        if self.current > num_waypoints - 1:
            self.current = IND_OOB


class HalfSpaceParams:
    """Class defining a halfspace"""

    def __init__(
        self,
        normal: NP_MAT = np.array([[1.0], [0.0], [0.0]]),
        point: NP_MAT = np.array([[1.0], [0.0], [0.0]]),
    ) -> None:
        """Initializes the normal vector and point on the halfspace to trivial values"""
        self.normal: NP_MAT = normal  # Defines vector normal to the halfspace
        self.point: NP_MAT = (
            point  # Defines a point on the halfspace (i.e., a point on the boundary)
        )

    def set(self, normal: NP_MAT, point: NP_MAT) -> None:
        """Sets the normal vector and the point on the halfspace. The passed in normal
           vector is normalized to the unit vector

        Args:
            normal: New normal vector. Must not have a zero magnitude
            point: New point on halfspace
        """
        # Check to ensure that the normal magnitude is non-zero
        mag_normal = np.linalg.norm(normal)
        if abs(mag_normal) < EPSILON:
            raise ValueError("Normal vector must have a non-zero magnitude")

        # Store the updated values
        self.point = point
        self.normal = normal / mag_normal

    def print(self) -> None:
        """Print the commands to the console."""
        print(
            "\nnormal=",
            self.normal,
            "\npoint=",
            self.point,
        )


def get_airspeed(waypoints: MsgWaypoints, ptr: WaypointIndices) -> float:
    """Extracts the airspeed from the waypoints. Current waypoint airspeed is used
       unless the current waypoint is out of bounds

    Args:
        waypoints: The waypoints to be followed
        ptr: The indices of the waypoints being used for the path

    Returns:
        airspeed: The speed at which the UAV should proceed to the current waypoint
    """
    airspeed = 25  # Default airspeed
    if ptr.current == IND_OOB:  # Use previous waypoint airspeed
        if ptr.previous != IND_OOB:
            airspeed = waypoints.airspeed.item(ptr.previous)
    else:  # Use the current waypoint airspeed
        airspeed = waypoints.airspeed.item(ptr.current)
    return airspeed


def extract_waypoints(
    waypoints: MsgWaypoints, ptr: WaypointIndices
) -> tuple[NP_MAT, NP_MAT, NP_MAT]:
    """Extracts the previous, current, and next waypoints

    Args:
        waypoints: The waypoints from which to construct the path
        ptr: The indices of the waypoints being used for the path

    Returns:
        previous: waypoint from the previous index (w_{i-1})
        current: current waypoint or 100 meters beyond the previous if the current is out of bounds (w_i)
        next_wp: next waypoint or 200 meters beyond previous if index is out of bounds (w_{i+1})
    """
    previous = waypoints.get_ned(ptr.previous)
    if ptr.current == IND_OOB:
        # Set the current and next to be in the distant direction of the final waypoints
        current = previous + 1000000.0 * waypoints.terminal_direction()
    else:
        current = waypoints.get_ned(ptr.current)
    if ptr.next == IND_OOB:
        next_wp = previous + 2000000.0 * waypoints.terminal_direction()
    else:
        next_wp = waypoints.get_ned(ptr.next)
    return (previous, current, next_wp)


def inHalfSpace(pos: NP_MAT, hs: HalfSpaceParams) -> bool:
    """Determine whether the position is in the next half-space

    Args:
        pos: Position being evaluated
        hs: halfspace parameters

    Returns:
        True if pos is in the halfpace, False otherwise
    """
    pos.reshape(hs.point.shape)
    plane_direction: NP_MAT = pos - hs.point
    check = plane_direction.T @ hs.normal >= 0
    if check:
        return True
    return False

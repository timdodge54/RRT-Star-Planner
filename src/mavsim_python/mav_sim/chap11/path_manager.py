"""path_manager.py is a module for managing the path for the mav
"""
from mav_sim.chap11.dubins_manager import dubins_manager
from mav_sim.chap11.dubins_parameters import DubinsParameters
from mav_sim.chap11.fillet_manager import fillet_manager
from mav_sim.chap11.line_manager import line_manager
from mav_sim.chap11.path_manager_utilities import (
    IND_OOB,
    HalfSpaceParams,
    WaypointIndices,
)
from mav_sim.message_types.msg_path import MsgPath
from mav_sim.message_types.msg_state import MsgState
from mav_sim.message_types.msg_waypoints import MsgWaypoints


class PathManager:
    """Class for managing the path for following waypoints"""

    def __init__(self) -> None:
        """Initialize class variables"""
        # message sent to path follower
        self.path = MsgPath()

        # pointers to previous, current, and next waypoints
        self.ptr = WaypointIndices()

        # Halfspace determines when to switch to the next waypoint
        self.halfspace = HalfSpaceParams()

        # state of the manager state machine, used for fillet and dubin's paths
        self.manager_state = 1

        # Flag indicating that new waypoints are needed, but have not been received
        self._waypoints = MsgWaypoints()

        # Storage for dubins parameters
        self.dubins_path = DubinsParameters()

    def set_waypoints(self, waypoints: MsgWaypoints) -> None:
        """Sets a new set of waypoints to be followed"""
        # Reset variables
        self._waypoints = waypoints
        self.ptr = WaypointIndices()
        self.manager_state = 1

        # Ensure that there are at least three waypoints for following
        if waypoints.num_waypoints < 3:
            raise ValueError("Path Manager: need at least three waypoints")

    def manager_requests_waypoints(self) -> bool:
        """Indicates that the manager needs waypoints (i.e., reached the end of waypoints list)

        Returns:
            Returns True if manager needs waypoints
        """
        return bool(self._waypoints.num_waypoints == 0 or self.ptr.current == IND_OOB)

    def update(self, radius: float, state: MsgState) -> MsgPath:
        """Provide update to the waypoing manager

        Args:
            radius: minimum radius circle for the mav
            state: current state of the vehicle

        Returns:
            path: The path to be followed
        """
        if self._waypoints.type == "straight_line":
            (self.path, self.halfspace, self.ptr) = line_manager(
                state=state,
                waypoints=self._waypoints,
                ptr_prv=self.ptr,
                path_prv=self.path,
                hs_prv=self.halfspace,
            )

        elif self._waypoints.type == "fillet":
            (self.path, self.halfspace, self.ptr, self.manager_state) = fillet_manager(
                state=state,
                waypoints=self._waypoints,
                ptr_prv=self.ptr,
                path_prv=self.path,
                hs_prv=self.halfspace,
                radius=radius,
                manager_state=self.manager_state,
            )

        elif self._waypoints.type == "dubins":
            (
                self.path,
                self.halfspace,
                self.ptr,
                self.manager_state,
                self.dubins_path,
            ) = dubins_manager(
                state=state,
                waypoints=self._waypoints,
                ptr_prv=self.ptr,
                path_prv=self.path,
                hs_prv=self.halfspace,
                radius=radius,
                manager_state=self.manager_state,
                dubins_path_prv=self.dubins_path,
            )

        else:  # Provide safety if a new type is added without updating the path manager
            raise ValueError("Error in Path Manager: Undefined waypoint type.")
        return self.path

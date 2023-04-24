"""
path planner for mavsim_python

mavsim_python
    - Beard & McLain, PUP, 2012
    - Last updated:
        4/3/2019 - BGM
"""
from typing import Optional

import mav_sim.parameters.planner_parameters as PLAN
import numpy as np
from mav_sim.chap12.planner_utilities import height_above_ground
from mav_sim.chap12.rrt_straight_line import RRTStraightLine
from mav_sim.final_project.rrt_star import RRTStarStraightLine
from mav_sim.message_types.msg_state import MsgState
from mav_sim.message_types.msg_waypoints import MsgWaypoints
from mav_sim.message_types.msg_world_map import MsgWorldMap
from mav_sim.tools.types import NP_MAT


class PlannerType:
    """Defines the planner to be used"""

    simple_fillet = 1  # Fixed waypoint path connected by fillet motion primitives
    simple_dubins = 2  # Fixed dubins waypoint path
    rrt_straight = 3  # Straight-line rrt. Waypoints defined as fillets, but fillet constraints not considered
    rrt_star = 4  # RRT* planner. Waypoints defined as fillets, but fillet constraints not considered


class PathPlanner:
    """Path planning object"""

    def __init__(self) -> None:
        """Initilize path planner"""
        # waypoints definition
        self.waypoints = MsgWaypoints()
        self.rrt_star = RRTStarStraightLine()
        self.radius = PLAN.R_min

    def update(
        self,
        world_map: MsgWorldMap,
        state: MsgState,
        end_pose_in: Optional[NP_MAT] = None,
        planner_type: int = PlannerType.rrt_straight,
        desired_airspeed: float = 25.0,
        desired_altitude: float = 100.0,
    ) -> MsgWaypoints:
        """Creates a plan based upon the current map, mav state, and minimum radius

           Raises a ValueError if the starting and ending points are invalid (i.e., below ground level, in city limits)

        Args:
            world_map: definition of the world for planning
            state: latest mav state
            end_pose: 3x1 or 4x1 vector giving the desired end location.
                        3x1 for straight line
                        4x1 for dubins where final element is the end orientation
            desired_airspeed: Airspeed to assign to the waypoints
            desired_altitude: The alitude that the aircraft should maintain

        Returns:
            waypoints: resulting plan
        """
        # Create the start pose
        start_pose = np.array(
            [[state.north], [state.east], [-desired_altitude], [state.chi]]
        )

        # Create the default end position
        if end_pose_in is None:
            # Move to top right corder
            if np.linalg.norm(start_pose[0:2]) < world_map.city_width / 2:
                end_pose = np.array(
                    [
                        [world_map.city_width],
                        [world_map.city_width],
                        [-desired_altitude],
                    ]
                )
            else:  # or to the bottom-left corner of world_map
                end_pose = np.array([[0], [0], [-desired_altitude]])
        else:
            end_pose = end_pose_in

        # Check the goal and end pose to ensure they are collision free
        h_start = height_above_ground(world_map=world_map, point=start_pose)
        if h_start <= 0.0:
            raise ValueError("Starting pose must be above ground")
        h_end = height_above_ground(world_map=world_map, point=end_pose)
        if h_end <= 0.0:
            raise ValueError("Ending pose must be above ground")

        # Check to ensure that the goal location is inside the city
        if (
            end_pose.item(0) < 0.0
            or end_pose.item(0) > world_map.city_width
            or end_pose.item(1) < 0.0
            or end_pose.item(1) > world_map.city_width
        ):
            raise ValueError("Ending pose must be in city limits")

        # Create plan
        print("planning...")
        self.waypoints = self.rrt_star.update(
            start_pose[0:3], end_pose, desired_airspeed, world_map, 100
        )

        self.waypoints.plot_updated = False
        print("...done planning.")
        return self.waypoints

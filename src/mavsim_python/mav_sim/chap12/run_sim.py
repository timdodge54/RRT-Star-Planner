""" Run the chapter 12 simulation
mavsim_python
    - Chapter 12 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        2/2/2019 - RWB
        4/22 - GND
"""

from typing import Optional

import mav_sim.parameters.planner_parameters as PLAN
from mav_sim.chap2.video_writer import VideoWriter
from mav_sim.chap3.data_viewer import DataViewer
from mav_sim.chap3.mav_dynamics import DynamicState
from mav_sim.chap4.wind_simulation import WindSimulation
from mav_sim.chap6.autopilot import Autopilot
from mav_sim.chap7.mav_dynamics import MavDynamics
from mav_sim.chap10.path_follower import PathFollower
from mav_sim.chap11.path_manager import PathManager
from mav_sim.chap12.path_planner import PathPlanner, PlannerType
from mav_sim.chap12.world_viewer import WorldViewer
from mav_sim.message_types.msg_sim_params import MsgSimParams
from mav_sim.message_types.msg_world_map import MsgWorldMap
from mav_sim.tools.types import NP_MAT

# pylint: disable=too-many-arguments


def run_sim(
    sim: MsgSimParams,
    end_pose: NP_MAT,
    init_state: Optional[DynamicState] = None,
    world_view: Optional[WorldViewer] = None,
    data_view: Optional[DataViewer] = None,
) -> tuple[WorldViewer, DataViewer]:
    """Runs the chapter 12 simulation

    Args:
        sim: Timing and file parameters for the simulation
        end_pose: The desired final set of waypoints
        init_state: Initial state of the MAV
        waypoint_view: Viewing window to be used for the mav and its associated path
        data_view: Viewing window to be used for the states

    Returns:
        waypoint_view: Viewing window to be used for the mav and its associated path
        data_view: Viewing window to be used for the states
    """

    # initialize the visualization
    if world_view is None:
        world_view = WorldViewer()  # initialize the mav viewer
    if data_view is None:
        data_view = DataViewer()  # initialize view of data plots
    if sim.write_video is True:
        video = VideoWriter(
            video_name=sim.video_name,
            bounding_box=(0, 0, 1000, 1000),
            output_rate=sim.ts_video,
        )

    # initialize elements of the architecture
    wind = WindSimulation(sim.ts_simulation)
    mav = MavDynamics(sim.ts_simulation, init_state)
    autopilot = Autopilot(sim.ts_simulation)
    path_follower = PathFollower()
    path_manager = PathManager()
    path_planner = PathPlanner()
    world_map = MsgWorldMap()

    # initialize the simulation time
    sim_time = sim.start_time
    next_plot_time = sim.ts_plotting

    # Initialize the waypoint planning with the desired end pose
    waypoints = path_planner.update(
        world_map=world_map,
        state=mav.true_state,
        planner_type=PlannerType.rrt_straight,
        end_pose_in=end_pose,
    )
    path_manager.set_waypoints(waypoints)

    # main simulation loop
    while sim_time < sim.end_time:
        # -------path planner - use default end-point when end received - ----
        if path_manager.manager_requests_waypoints() is True:

            waypoints = path_planner.update(
                world_map=world_map,
                state=mav.true_state,
                planner_type=PlannerType.rrt_straight,
            )
            path_manager.set_waypoints(waypoints)

        # -------path manager-------------
        path = path_manager.update(PLAN.R_min, mav.true_state)

        # -------path follower-------------
        autopilot_commands = path_follower.update(path, mav.true_state)

        # -------autopilot-------------
        delta, commanded_state = autopilot.update(autopilot_commands, mav.true_state)

        # -------physical system-------------
        mav.update(delta, wind.update())  # propagate the MAV dynamics

        # -------update viewer-------------
        if next_plot_time <= sim_time:
            next_plot_time += sim.ts_plotting
            world_view.update(
                mav.true_state, path, waypoints, world_map
            )  # plot path and MAV
            data_view.update(
                mav.true_state,  # true states
                mav.true_state,  # estimated states
                commanded_state,  # commanded states
                delta,  # input to aircraft
                sim.ts_plotting,
            )

        if sim.write_video is True:
            video.update(sim_time)

        # -------increment time-------------
        sim_time += sim.ts_simulation

    if sim.write_video is True:
        video.close()

    return (world_view, data_view)

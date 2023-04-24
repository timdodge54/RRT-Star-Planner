""" Run the chapter 10 simulation
mavsim_python
    - Chapter 10 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        2/2/2019 - RWB
        2/22 - GND
"""

from typing import Callable, Optional

from mav_sim.chap2.video_writer import VideoWriter
from mav_sim.chap3.data_viewer import DataViewer
from mav_sim.chap3.mav_dynamics import DynamicState
from mav_sim.chap4.wind_simulation import WindSimulation
from mav_sim.chap6.autopilot import Autopilot
from mav_sim.chap7.mav_dynamics import MavDynamics
from mav_sim.chap10.path_follower import PathFollower
from mav_sim.chap10.path_viewer import PathViewer
from mav_sim.message_types.msg_path import MsgPath
from mav_sim.message_types.msg_sim_params import MsgSimParams
from mav_sim.message_types.msg_state import MsgState

# pylint: disable=too-many-arguments

# Function that will take in a time value and state message and return a path message
PathFunction = Callable[[float, MsgState], MsgPath]


def run_sim(
    sim: MsgSimParams,
    path_fnc: PathFunction,
    init_state: Optional[DynamicState] = None,
    path_view: Optional[PathViewer] = None,
    data_view: Optional[DataViewer] = None,
) -> tuple[PathViewer, DataViewer]:
    """Runs the chapter 10 simulation

    Args:
        sim: Timing and file parameters for the simulation
        path_fnc: Function that takes in time and state and returns the path to be followed by uav
        init_state: Initial state of the MAV
        path_view: Viewing window to be used for the mav and its associated path
        data_view: Viewing window to be used for the states

    Returns:
        path_view: Viewing window to be used for the mav and its associated path
        data_view: Viewing window to be used for the states
    """

    # initialize the visualization
    if path_view is None:
        path_view = PathViewer()  # initialize the mav viewer
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

    # initialize the simulation time
    sim_time = sim.start_time
    next_plot_time = sim.ts_plotting

    # main simulation loop
    while sim_time < sim.end_time:
        # -------path follower-------------
        current_path = path_fnc(sim_time, mav.true_state)
        autopilot_commands = path_follower.update(current_path, mav.true_state)

        # -------autopilot-------------
        delta, commanded_state = autopilot.update(autopilot_commands, mav.true_state)

        # -------physical system-------------
        mav.update(delta, wind.update())  # propagate the MAV dynamics

        # -------update viewer-------------
        if next_plot_time <= sim_time:
            next_plot_time += sim.ts_plotting
            path_view.update(mav.true_state, current_path)  # plot path and MAV
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

    return (path_view, data_view)

""" Run the chapter 4 simulation
run_sim.py
    - Chapter 4 assignment for Beard & McLain, PUP, 2012
    - Update history:
        12/18/2018 - RWB
        1/14/2019 - RWB
        12/21 - GND
"""

from typing import Callable, Optional, cast

import mav_sim.parameters.simulation_parameters as SIM
import numpy as np
from mav_sim.chap2.mav_viewer import MavViewer
from mav_sim.chap2.video_writer import VideoWriter
from mav_sim.chap3.data_viewer import DataViewer
from mav_sim.chap3.mav_dynamics import DynamicState
from mav_sim.chap4.mav_dynamics import MavDynamics
from mav_sim.chap4.wind_simulation import WindSimulation
from mav_sim.message_types.msg_delta import MsgDelta
from mav_sim.message_types.msg_gust_params import MsgGustParams
from mav_sim.message_types.msg_sim_params import MsgSimParams
from mav_sim.tools import types

DeltaTimeFunction = Callable[[float], MsgDelta]


def run_sim(
    sim: MsgSimParams,
    delta_fnc: DeltaTimeFunction,
    init_state: Optional[DynamicState] = None,
    mav_view: Optional[MavViewer] = None,
    data_view: Optional[DataViewer] = None,
    use_wind: bool = False,
    gust_params: Optional[MsgGustParams] = None,
) -> tuple[MavViewer, DataViewer]:
    """Runs the chapter 4 simulation

    Args:
        sim: Timing and file parameters for the simulation
        delta: Fixed control surfaces
        init_state: Initial state of the MAV
        mav_view: Viewing window to be used for the mav
        data_view: Viewing window to be used for the states
        use_wind: True => wind will be used, False => no wind

    Returns:
        mav_view: Viewing window to be used for the mav
        data_view: Viewing window to be used for the states
    """

    # initialize the visualization
    if mav_view is None:
        mav_view = MavViewer()  # initialize the mav viewer
    if data_view is None:
        data_view = DataViewer()  # initialize view of data plots
    if sim.write_video is True:
        video = VideoWriter(
            video_name=sim.video_name,
            bounding_box=(0, 0, 1000, 1000),
            output_rate=sim.ts_video,
        )

    # initialize elements of the architecture
    wind = WindSimulation(SIM.ts_simulation, gust_params)
    mav = MavDynamics(sim.ts_simulation, init_state)

    # initialize the simulation time
    sim_time = sim.start_time
    next_plot_time = sim.ts_plotting

    # Create the default wind
    def current_wind() -> types.WindVector:
        zero_wind_vec = np.zeros([6, 1])
        return cast(types.WindVector, zero_wind_vec)

    if use_wind:
        current_wind = wind.update

    # main simulation loop
    while sim_time < sim.end_time:
        # -------Get inputs -------------
        delta = delta_fnc(sim_time)

        # -------physical system-------------
        mav.update(delta, current_wind())  # propagate the MAV dynamics

        # -------update viewer-------------
        if next_plot_time <= sim_time:
            next_plot_time += sim.ts_plotting
            mav_view.update(mav.true_state)  # plot body of MAV
            data_view.update(
                mav.true_state,  # true states
                mav.true_state,  # estimated states
                mav.true_state,  # commanded states
                delta,  # inputs to the aircraft
                sim.ts_plotting,
            )

        if sim.write_video is True:
            video.update(sim_time)

        # -------increment time-------------
        sim_time += sim.ts_simulation

    if sim.write_video is True:
        video.close()

    return (mav_view, data_view)

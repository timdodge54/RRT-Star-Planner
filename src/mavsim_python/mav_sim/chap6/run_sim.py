""" Run the chapter 6 simulation
mavsim_python
    - Chapter 6 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        2/2/2019 - RWB
        1/22 - GND
"""

from typing import Optional, cast

import mav_sim.parameters.aerosonde_parameters as MAV
import mav_sim.parameters.simulation_parameters as SIM
import numpy as np
from mav_sim.chap2.mav_viewer import MavViewer
from mav_sim.chap2.video_writer import VideoWriter
from mav_sim.chap3.data_viewer import DataViewer
from mav_sim.chap3.mav_dynamics import DynamicState
from mav_sim.chap4.mav_dynamics import MavDynamics
from mav_sim.chap4.wind_simulation import WindSimulation
from mav_sim.chap6.autopilot import Autopilot
from mav_sim.message_types.msg_autopilot import MsgAutopilot
from mav_sim.message_types.msg_gust_params import MsgGustParams
from mav_sim.message_types.msg_sim_params import MsgSimParams
from mav_sim.tools import types
from mav_sim.tools.signals import Signals

# pylint: disable=too-many-arguments

# Define nominal commands
Va_command_nom = Signals(
    dc_offset=MAV.Va0, amplitude=0.0, start_time=1000.0, frequency=0.01
)
altitude_command_nom = Signals(
    dc_offset=-MAV.down0, amplitude=0.0, start_time=1000.0, frequency=0.02
)
course_command_nom = Signals(
    dc_offset=MAV.psi0, amplitude=0.0, start_time=1000.0, frequency=0.015
)


def run_sim(
    sim: MsgSimParams,
    init_state: Optional[DynamicState] = None,
    mav_view: Optional[MavViewer] = None,
    data_view: Optional[DataViewer] = None,
    use_wind: bool = False,
    gust_params: Optional[MsgGustParams] = None,
    Va_command: Signals = Va_command_nom,
    altitude_command: Signals = altitude_command_nom,
    course_command: Signals = course_command_nom,
) -> tuple[MavViewer, DataViewer]:
    """Runs the chapter 6 simulation

    Args:
        sim: Timing and file parameters for the simulation
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
    autopilot = Autopilot(SIM.ts_simulation)

    # initialize the simulation time
    sim_time = sim.start_time
    next_plot_time = sim.ts_plotting

    # Create the default wind
    def current_wind() -> types.WindVector:
        zero_wind_vec = np.zeros([6, 1])
        return cast(types.WindVector, zero_wind_vec)

    if use_wind:
        current_wind = wind.update

    # autopilot commands
    commands = MsgAutopilot()

    # main simulation loop
    while sim_time < sim.end_time:
        # -------autopilot commands-------------
        commands.airspeed_command = Va_command.square(sim_time)
        commands.course_command = course_command.square(sim_time)
        commands.altitude_command = altitude_command.square(sim_time)

        # -------autopilot-------------
        estimated_state = mav.true_state  # uses true states in the control
        delta, commanded_state = autopilot.update(commands, estimated_state)

        # -------physical system-------------
        mav.update(delta, current_wind())  # propagate the MAV dynamics

        # -------update viewer-------------
        if next_plot_time <= sim_time:
            next_plot_time += sim.ts_plotting
            mav_view.update(mav.true_state)  # plot body of MAV
            data_view.update(
                mav.true_state,  # true states
                estimated_state,  # estimated states
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

    return (mav_view, data_view)

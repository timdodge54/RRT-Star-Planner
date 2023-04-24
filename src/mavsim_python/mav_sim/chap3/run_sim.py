""" Run the chapter 3 simulation
run_sim.py
    - Chapter 3 assignment for Beard & McLain, PUP, 2012
    - Update history:
        12/18/2018 - RWB
        1/14/2019 - RWB
        12/21 - GND
"""

from typing import Optional

from mav_sim.chap2.mav_viewer import MavViewer
from mav_sim.chap2.video_writer import VideoWriter
from mav_sim.chap3.data_viewer import DataViewer
from mav_sim.chap3.mav_dynamics import DynamicState, ForceMoments, MavDynamics
from mav_sim.message_types.msg_delta import MsgDelta
from mav_sim.message_types.msg_sim_params import MsgSimParams


def run_sim(
    sim: MsgSimParams,
    init_state: DynamicState,
    fm: ForceMoments,
    mav_view: Optional[MavViewer] = None,
    data_view: Optional[DataViewer] = None,
) -> tuple[MavViewer, DataViewer]:
    """Runs the chapter 3 simulation

    Args:
        sim: Timing and file parameters for the simulation
        init_state: Initial state of the MAV
        fm: forces and moments (constant) being input to the system
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
    mav = MavDynamics(sim.ts_simulation, init_state)
    delta = MsgDelta()

    # initialize the simulation time
    sim_time = sim.start_time
    next_plot_time = sim.ts_plotting

    # main simulation loop
    print("Press Command-Q to exit...")
    while sim_time < sim.end_time:
        # -------vary forces and moments to check dynamics-------------
        forces_moments = fm.to_array()

        # -------physical system-------------
        mav.update(forces_moments)  # propagate the MAV dynamics

        # -------update viewer-------------
        if next_plot_time <= sim_time:
            mav_view.update(mav.true_state)  # plot body of MAV
            data_view.update(
                mav.true_state,  # true states
                mav.true_state,  # estimated states
                mav.true_state,  # commanded states
                delta,  # inputs to the aircraft
                sim.ts_plotting,
            )
            next_plot_time += sim.ts_plotting

        if sim.write_video is True:
            video.update(sim_time)

        # -------increment time-------------
        sim_time += sim.ts_simulation

    if sim.write_video is True:
        video.close()

    return (mav_view, data_view)

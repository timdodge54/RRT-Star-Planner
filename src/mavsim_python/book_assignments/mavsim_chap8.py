"""
mavsim_python
    - Chapter 8 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        2/21/2019 - RWB
        2/24/2020 - RWB
"""
import copy

import mav_sim.parameters.simulation_parameters as SIM
import numpy as np
from mav_sim.chap2.mav_viewer import MavViewer
from mav_sim.chap2.video_writer import VideoWriter
from mav_sim.chap3.data_viewer import DataViewer
from mav_sim.chap4.wind_simulation import WindSimulation
from mav_sim.chap6.autopilot import Autopilot
from mav_sim.chap7.mav_dynamics import MavDynamics
from mav_sim.chap8.observer import Observer
from mav_sim.message_types.msg_autopilot import MsgAutopilot

# from mav_sim.chap8.observer_analyt import Observer
# from mav_sim.chap8.observer_simple import Observer
from mav_sim.tools.signals import Signals

# initialize the visualization
VIDEO = False  # True==write video, False==don't write video
mav_view = MavViewer()  # initialize the mav viewer
data_view = DataViewer()  # initialize view of data plots
if VIDEO is True:
    video = VideoWriter(
        video_name="chap8_video.avi",
        bounding_box=(0, 0, 1000, 1000),
        output_rate=SIM.ts_video,
    )

# initialize elements of the architecture
wind = WindSimulation(SIM.ts_simulation)
mav = MavDynamics(SIM.ts_simulation)
autopilot = Autopilot(SIM.ts_simulation)
initial_state = copy.deepcopy(mav.true_state)
initial_measurements = copy.deepcopy(mav.sensors())
observer = Observer(SIM.ts_simulation, initial_state, initial_measurements)
# observer = Observer(SIM.ts_simulation) #, initial_state,initial_measurements)


# autopilot commands
commands = MsgAutopilot()
Va_command = Signals(dc_offset=25.0, amplitude=3.0, start_time=2.0, frequency=0.01)
h_command = Signals(dc_offset=100.0, amplitude=20.0, start_time=0.0, frequency=0.02)
chi_command = Signals(
    dc_offset=np.radians(0.0),
    amplitude=np.radians(45.0),
    start_time=10.0,
    frequency=0.015,
)

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:

    # -------autopilot commands-------------
    commands.airspeed_command = Va_command.polynomial(sim_time)
    commands.course_command = chi_command.polynomial(sim_time)
    commands.altitude_command = h_command.polynomial(sim_time)

    # -------autopilot-------------
    measurements = mav.sensors()  # get sensor measurements
    estimated_state = observer.update(measurements)  # estimate states from measurements
    delta, commanded_state = autopilot.update(commands, estimated_state)

    # -------physical system-------------
    current_wind = wind.update()  # get the new wind vector
    mav.update(delta, current_wind)  # propagate the MAV dynamics

    # -------update viewer-------------
    mav_view.update(mav.true_state)  # plot body of MAV
    data_view.update(
        mav.true_state,  # true states
        estimated_state,  # estimated states
        commanded_state,  # commanded states
        delta,  # input to aircraft
        SIM.ts_simulation,
    )
    if VIDEO is True:
        video.update(sim_time)

    # -------increment time-------------
    sim_time += SIM.ts_simulation

if VIDEO is True:
    video.close()

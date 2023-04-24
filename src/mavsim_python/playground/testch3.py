# Note that this cell can be run separately to initialize for other cell blocks
from mav_sim.chap3.mav_dynamics import DynamicState, ForceMoments
from mav_sim.chap3.run_sim import run_sim
from mav_sim.message_types.msg_sim_params import MsgSimParams
from mav_sim.chap2.mav_viewer import MavViewer
from mav_sim.chap2.video_writer import VideoWriter
from mav_sim.chap3.data_viewer import DataViewer
from mav_sim.tools.display_figures import display_data_view, display_mav_view
import numpy as np


def run_sim_and_display(sim_params: MsgSimParams, state, fm, mav_view, data_view):
    data_view.reset(sim_params.start_time)
    (mav_view, data_view) = run_sim(sim_params, state, fm, mav_view, data_view)

    # mav_view.app.closeAllWindows() #window.close()

    # data_view.plotter.app.closeAllWindows()
    print("gets here")
    # display_data_view(data_view)
    # display_mav_view(mav_view)


# Initialize the simulation parameters
mav_view = MavViewer()  # initialize the mav viewer
data_view = DataViewer()  # initialize view of data plots
sim_params = MsgSimParams(end_time=20.0)  # Sim ending in 10 seconds
state = DynamicState(state=np.zeros([13, 1]))  # State initialized to zeros
state.down = -5.0
state.set_attitude_euler(0.0, 0.0, np.pi / 4)
fm = ForceMoments(force_moment=np.zeros([6, 1]))  # Set all forces equal to zero
fm.l = 0.1

# Run the simulation and display the data
run_sim_and_display(sim_params, state, fm, mav_view, data_view)


# Initialize the simulation parameters
sim_params = MsgSimParams(end_time=10.0)  # Sim ending in 10 seconds
state = DynamicState(state=np.zeros([13, 1]))  # State initialized to zeros
state.down = -5.0
state.set_attitude_euler(0.0, 0.0, np.pi / 4)
fm = ForceMoments(force_moment=np.zeros([6, 1]))  # Set all forces equal to zero

# Run the simulation and display the data
run_sim_and_display(sim_params, state, fm, mav_view, data_view)

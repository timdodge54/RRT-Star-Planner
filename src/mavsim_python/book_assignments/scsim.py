"""
scSimPy
    - Appendix C example for Beard & McLain, PUP, 2012
    - Update history:
        1/13/2021 - TWM
"""

# import parameters
import mav_sim.parameters.simulation_parameters as SIM

# import viewers and video writer
from mav_sim.appC.sc_viewer import ScViewer
from mav_sim.appC.video_writer import VideoWriter

# import message types
from mav_sim.message_types.msg_state import MsgState

# initialize messages
state = MsgState()  # instantiate state message

# initialize viewers and video
VIDEO = False  # True==write video, False==don't write video
sc_view = ScViewer()
if VIDEO is True:
    video = VideoWriter(
        video_name="appC_video.avi",
        bounding_box=(0, 0, 1000, 1000),
        output_rate=SIM.ts_video,
    )

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
while sim_time < SIM.end_time:
    # -------vary states to check viewer-------------
    if sim_time < SIM.end_time / 6:
        state.north += 10 * SIM.ts_simulation
    elif sim_time < 2 * SIM.end_time / 6:
        state.east += 10 * SIM.ts_simulation
    elif sim_time < 3 * SIM.end_time / 6:
        state.altitude += 10 * SIM.ts_simulation
    elif sim_time < 4 * SIM.end_time / 6:
        state.psi += 0.1 * SIM.ts_simulation
    elif sim_time < 5 * SIM.end_time / 6:
        state.theta += 0.1 * SIM.ts_simulation
    else:
        state.phi += 0.1 * SIM.ts_simulation

    # -------update viewer and video-------------
    sc_view.update(state)
    if VIDEO is True:
        video.update(sim_time)

    # -------increment time-------------
    sim_time += SIM.ts_simulation

print("Press Ctrl-Q to exit...")
if VIDEO is True:
    video.close()

"""
mavsimPy
    - Chapter 3 assignment for Beard & McLain, PUP, 2012
    - Update history:
        12/18/2018 - RWB
        1/14/2019 - RWB
        1/22 - GND
"""

import numpy as np
from mav_sim.chap3.mav_dynamics import DynamicState, ForceMoments
from mav_sim.chap3.run_sim import run_sim
from mav_sim.message_types.msg_sim_params import MsgSimParams


def main() -> None:
    """Provide a test scenario"""
    # Initialize the simulation parameters
    # sim_params = MsgSimParams(end_time=100., video_name="chap3.avi") # Sim ending in 10 seconds
    # state = DynamicState(state=np.zeros([13,1]))
    # state.down = -5.
    # state.u = 1
    # state.set_attitude_euler(0., 0., np.pi/4)
    # fm = ForceMoments(force_moment=np.zeros([6,1])) # type: ignore
    # fm.l = 0.01
    # Initialize the simulation parameters
    sim_params = MsgSimParams(end_time=100.0)  # Sim ending in 10 seconds
    state = DynamicState(state=np.zeros([13, 1]))  # State initialized to zeros
    state.down = -5.0
    state.set_attitude_euler(0.0, 0.0, np.pi / 4)
    fm = ForceMoments(force_moment=np.zeros([6, 1]))  # type: ignore
    fm.fx = 0.1
    fm.fz = 0.5

    # Run the simulation
    run_sim(sim_params, state, fm)


if __name__ == "__main__":
    main()

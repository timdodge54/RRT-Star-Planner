"""
mavsimPy
    - Chapter 4 assignment for Beard & McLain, PUP, 2012
    - Update history:
        12/18/2018 - RWB
        1/14/2019 - RWB
        1/22 - GND
"""

import numpy as np
from mav_sim.chap3.mav_dynamics import DynamicState
from mav_sim.chap4.run_sim import run_sim
from mav_sim.message_types.msg_delta import MsgDelta
from mav_sim.message_types.msg_sim_params import MsgSimParams


def trim(time: float) -> MsgDelta:  # pylint: disable=unused-argument
    """Passes out the constant trim command"""
    # Set control surfaces
    delta = MsgDelta()
    delta.elevator = -0.1248
    delta.aileron = 0.001836
    delta.rudder = -0.0003026
    delta.throttle = 0.6768
    return delta


def perturb_elevator(time: float) -> MsgDelta:
    """Perturb the trim trajectory"""
    delta = trim(time)
    if time > 30.0:
        delta.elevator += 5 * np.pi / 180.0  # Perturb up by 2 degrees
    elif time > 10:
        delta.elevator += -5 * np.pi / 180.0  # Perturb down by 2 degrees

    return delta


def main() -> None:
    """Provide a test scenario"""
    # Initialize the simulation parameters
    sim_params = MsgSimParams(
        end_time=100.0, video_name="chap4.avi"
    )  # Sim ending in 10 seconds
    state = DynamicState()

    # Run the simulation - Note that while not used, the viewer objects
    # need to remain active to keep the windows open
    (_, _) = run_sim(
        sim_params, perturb_elevator, state, use_wind=True
    )  # pylint: disable=unused-variable

    # Wait until user is finished
    print("Press any key to close")
    input()


if __name__ == "__main__":
    main()

"""
mavsim_python
    - Chapter 10 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        3/11/2019 - RWB
        2/27/2020 - RWB
        3/2022 - GND
"""
import numpy as np
from mav_sim.chap3.mav_dynamics import DynamicState
from mav_sim.chap10.run_sim import run_sim
from mav_sim.message_types.msg_path import MsgPath
from mav_sim.message_types.msg_sim_params import MsgSimParams
from mav_sim.message_types.msg_state import MsgState


def main() -> None:
    """Provide a test scenario for chapter 10"""
    # Initialize the simulation parameters
    sim_params = MsgSimParams(
        end_time=200.0, video_name="cha10.avi"
    )  # Sim ending in 10 seconds
    state = DynamicState()

    # path definition
    path = MsgPath()
    # path.type = 'line'
    # path.line_origin = np.array([[0.0, 0.0, -100.0]]).T
    # path.line_direction = np.array([[0.5, 1.0, 0.0]]).T
    # path.line_direction = path.line_direction / np.linalg.norm(path.line_direction)

    path.type = "orbit"
    path.orbit_center = np.array([[0.0, 0.0, -100.0]]).T  # center of the orbit
    path.orbit_radius = 300.0  # radius of the orbit
    path.orbit_direction = (
        "CW"  # orbit direction: 'CW'==clockwise, 'CCW'==counter clockwise
    )

    def constant_path(_: float, __: MsgState) -> MsgPath:
        return path

    # Run the simulation - Note that while not used, the viewer objects
    # need to remain active to keep the windows open
    (_, _) = run_sim(
        sim=sim_params, path_fnc=constant_path, init_state=state
    )  # pylint: disable=unused-variable

    # Wait until user is finished
    print("Press any key to close")
    input()


if __name__ == "__main__":
    main()

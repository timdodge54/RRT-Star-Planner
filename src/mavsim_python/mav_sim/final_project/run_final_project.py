"""
mavsim_python
    - Chapter 12 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        4/3/2019 - BGM
        2/27/2020 - RWB
        4/2022 - GND
"""
import numpy as np
from mav_sim.chap3.mav_dynamics import DynamicState
from mav_sim.final_project.run_sim import run_sim
from mav_sim.message_types.msg_sim_params import MsgSimParams
from mav_sim.message_types.msg_world_map import MsgWorldMap


def main() -> None:
    """Provide a test scenario for chapter 12"""
    # Initialize the simulation parameters
    sim_params = MsgSimParams(
        end_time=1000.0, video_name="cha12.avi"
    )  # Sim ending in 10 seconds
    state = DynamicState()

    # Final point definition
    world_map = MsgWorldMap()
    end_pose = np.array([[world_map.city_width], [world_map.city_width], [-100]])

    # Run the simulation - Note that while not used, the viewer objects
    # need to remain active to keep the windows open
    (_, _) = run_sim(
        sim=sim_params, end_pose=end_pose, init_state=state
    )  # pylint: disable=unused-variable

    # Wait until user is finished
    print("Press any key to close")
    input()


if __name__ == "__main__":
    main()

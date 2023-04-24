"""
mavsim_python
    - Chapter 11 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        3/11/2019 - RWB
        2/27/2020 - RWB
        3/2022 - GND
"""
import mav_sim.parameters.planner_parameters as PLAN
import numpy as np
from mav_sim.chap3.mav_dynamics import DynamicState
from mav_sim.chap11.run_sim import run_sim
from mav_sim.message_types.msg_sim_params import MsgSimParams
from mav_sim.message_types.msg_waypoints import MsgWaypoints


def main() -> None:
    """Provide a test scenario for chapter 11"""
    # Initialize the simulation parameters
    sim_params = MsgSimParams(
        end_time=130.0, video_name="cha11.avi"
    )  # Sim ending in 10 seconds
    state = DynamicState()

    # Waypoint definition
    waypoints = MsgWaypoints()
    # waypoints.type = 'straight_line'
    # waypoints.type = 'fillet'
    waypoints.type = "dubins"
    Va = PLAN.Va0
    waypoints.add(np.array([[0, 0, -100]]).T, Va, np.radians(0), np.inf, 0, 0)
    waypoints.add(np.array([[1000, 0, -100]]).T, Va, np.radians(45), np.inf, 0, 0)
    waypoints.add(np.array([[0, 1000, -100]]).T, Va, np.radians(45), np.inf, 0, 0)
    waypoints.add(np.array([[1000, 1000, -100]]).T, Va, np.radians(-135), np.inf, 0, 0)

    # waypoints.add(np.array([[0, 0, -100]]).T, Va, np.radians(0), np.inf, 0, 0)
    # waypoints.add(np.array([[1000, 0, -100]]).T, Va, np.radians(45), np.inf, 0, 0)
    # waypoints.add(np.array([[0, 0, -100]]).T, Va, np.radians(45), np.inf, 0, 0)

    # Run the simulation - Note that while not used, the viewer objects
    # need to remain active to keep the windows open
    (_, _) = run_sim(
        sim=sim_params, waypoints=waypoints, init_state=state
    )  # pylint: disable=unused-variable

    # Wait until user is finished
    print("Press any key to close")
    input()


if __name__ == "__main__":
    main()

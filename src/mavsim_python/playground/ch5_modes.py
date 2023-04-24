"""
mavsim_python
    - Chapter 5 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        2/2/2019 - RWB
        1/22 - GND
"""

from mav_sim.chap3.mav_dynamics import DynamicState
from mav_sim.chap4.run_sim import run_sim
from mav_sim.message_types.msg_sim_params import MsgSimParams
from mav_sim.message_types.msg_delta import MsgDelta
from mav_sim.tools.signals import Signals
from mav_sim.chap5.trim import compute_trim

# Function for perturbing inputs
input_signal = Signals(amplitude=0.3, duration=0.3, start_time=5.0)


def perturb_delta(sim_time: float, delta: MsgDelta):
    """Takes in the delta command and perturbs the output"""
    # copy the input command
    delta_cmd = MsgDelta()
    delta_cmd.copy(delta)

    # this input excites the phugoid mode by adding an elevator impulse at t = 5.0 s
    # delta_cmd.elevator += input_signal.impulse(sim_time)
    delta_cmd.elevator += input_signal.doublet(sim_time)

    # this input excites the roll and spiral divergence modes by adding an aileron doublet at t = 5.0 s
    # delta_cmd.aileron += input_signal.doublet(sim_time)

    # this input excites the dutch roll mode by adding a rudder doublet at t = 5.0 s
    # delta_cmd.rudder += input_signal.doublet(sim_time)

    return delta_cmd


def main() -> None:
    """Provide a test scenario"""
    # Initialize the simulation parameters
    sim_params = MsgSimParams(
        end_time=100.0, video_name="chap4.avi"
    )  # Sim ending in 10 seconds
    state = DynamicState()

    # use compute_trim function to compute trim state and trim input
    Va_trim = 35.0
    gamma_trim = 0.0
    trim_state, trim_input = compute_trim(state.convert_to_numpy(), Va_trim, gamma_trim)

    # Create a function for perturbing the trim input
    delta_fnc = lambda sim_time: perturb_delta(sim_time, trim_input)

    # Run the simulation - Note that while not used, the viewer objects
    # need to remain active to keep the windows open
    (mav_view, data_view) = run_sim(
        sim=sim_params, init_state=DynamicState(trim_state), delta_fnc=delta_fnc
    )  # pylint: disable=unused-variable

    # Wait until user is finished
    print("Press any key to close")
    input()


if __name__ == "__main__":
    main()

"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
        12/21 - GND
"""
# from typing import Optional

# import mav_sim.parameters.aerosonde_parameters as MAV
import mav_sim.parameters.control_parameters as AP
import numpy as np
from mav_sim.chap6.pd_control_with_rate import PDControlWithRate
from mav_sim.chap6.pi_control import PIControl
from mav_sim.chap6.tf_control import TFControl
from mav_sim.message_types.msg_autopilot import MsgAutopilot
from mav_sim.message_types.msg_delta import MsgDelta
from mav_sim.message_types.msg_state import MsgState

# from mav_sim.tools.transfer_function import TransferFunction
from mav_sim.tools.wrap import saturate, wrap


class Autopilot:
    """Creates an autopilot for controlling the mav to desired values"""

    def __init__(self, ts_control: float) -> None:
        """Initialize the lateral and longitudinal controllers

        Args:
            ts_control: time step for the control
        """

        # instantiate lateral-directional controllers (note, these should be objects, not numbers)
        self.roll_limit = 45 * np.pi / 180
        self.roll_from_aileron = PDControlWithRate(
            kp=AP.roll_kp, kd=AP.roll_kd, limit=self.roll_limit
        )
        self.course_limit = 30 * np.pi / 180
        self.yaw_limit = 30 * np.pi / 180
        self.course_from_roll = PIControl(
            kp=AP.course_kp, ki=AP.course_ki, Ts=ts_control, limit=self.course_limit
        )
        self.yaw_damper = TFControl(
            k=AP.yaw_damper_kr,
            n0=0,
            n1=1,
            d1=1,
            d0=AP.yaw_damper_p_wo,
            Ts=ts_control,
            # limit=self.yaw_limit,
        )

        # instantiate longitudinal controllers (note, these should be objects, not numbers)
        self.elv_limit = 45 * np.pi / 180
        self.pitch_from_elevator = PDControlWithRate(
            kp=AP.pitch_kp, kd=AP.pitch_kd, limit=np.pi / 4
        )
        self.alt_limit = 30 * np.pi / 180
        self.altitude_from_pitch = PIControl(
            kp=AP.altitude_kp, ki=AP.altitude_ki, Ts=ts_control, limit=np.pi / 6
        )
        self.throttle_limit = 1.0
        self.airspeed_from_throttle = PIControl(
            kp=AP.airspeed_throttle_kp,
            ki=AP.airspeed_throttle_ki,
            Ts=ts_control,
        )
        self.commanded_state = MsgState()

    def update(self, cmd: MsgAutopilot, state: MsgState) -> tuple[MsgDelta, MsgState]:
        """Given a state and autopilot command, compute the control to the mav

        Args:
            cmd: command to the autopilot
            state: current state of the mav

        Returns:
            delta: low-level flap commands
            commanded_state: the state being commanded
        """
        limit_phi = 30 * np.pi / 180
        # --------------roll loop-----------
        altitude_command = saturate(
            cmd.altitude_command,
            state.altitude - AP.altitude_zone,
            state.altitude + AP.altitude_zone,
        )
        course_command = wrap(cmd.course_command, state.chi)
        phi_c = (
            self.course_from_roll.update(y_ref=course_command, y=state.chi)
            + cmd.phi_feedforward
        )
        # print(f"phi_c sat: {phi_c}")
        phi_c = saturate(phi_c, -limit_phi, limit_phi)
        delta_a = self.roll_from_aileron.update(y_ref=phi_c, y=state.phi, ydot=state.p)
        # print(f"delta_a: {delta_a}")
        # -----------pitch loop
        theta_c = self.altitude_from_pitch.update(
            y_ref=altitude_command, y=state.altitude
        )
        # print(f"thetac: {theta_c}, state.theta: {state.theta}")
        delta_e = self.pitch_from_elevator.update(
            y_ref=theta_c, y=state.theta, ydot=state.q
        )
        # print(f"delta_e: {delta_e}")
        # -----------Throttle
        delta_t = self.airspeed_from_throttle.update(
            y_ref=cmd.airspeed_command, y=state.Va
        )
        # print(f"delta_t: {delta_t}")
        # -----------Rudder
        delta_r = self.yaw_damper.update(y=state.r)
        # print(f"delta_r: {delta_r}")

        # longitudinal autopilot
        delta_t = saturate(delta_t, 0.0, 1.0)
        # print(f"delta_t sat: {delta_t}")

        # construct control outputs and commanded states
        delta = MsgDelta(
            elevator=delta_e, aileron=delta_a, rudder=delta_r, throttle=delta_t
        )
        self.commanded_state.altitude = cmd.altitude_command
        self.commanded_state.Va = cmd.airspeed_command
        self.commanded_state.phi = phi_c
        self.commanded_state.theta = theta_c
        self.commanded_state.chi = course_command
        return delta, self.commanded_state.copy()

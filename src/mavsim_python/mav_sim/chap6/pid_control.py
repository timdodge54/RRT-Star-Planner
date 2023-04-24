"""
pid_control
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
        12/21 - GND
"""
import numpy as np
from mav_sim.tools.wrap import saturate


class PIDControl:
    """Proportional integral derivative control"""

    def __init__(
        self,
        kp: float = 0.0,
        ki: float = 0.0,
        kd: float = 0.0,
        Ts: float = 0.01,
        sigma: float = 0.05,
        limit: float = 1.0,
    ) -> None:
        """Store control gains

        Args:
            kp: proportional gain
            ki: integral gain
            kd: derivative gain
            Ts: time step for control
            sigma: Filter value for derivative calculation
            limit: maximum magnitude for the control
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.Ts = Ts
        self.limit = limit
        self.integrator = 0.0
        self.error_delay_1 = 0.0
        self.error_dot_delay_1 = 0.0
        self.y_dot = 0.0
        self.y_delay_1 = 0.0
        self.y_dot_delay_1 = 0.0
        # gains for differentiator
        self.a1 = (2.0 * sigma - Ts) / (2.0 * sigma + Ts)
        self.a2 = 2.0 / (2.0 * sigma + Ts)

    def update(self, y_ref: float, y: float, reset_flag: bool = False) -> float:
        """Calculate the PID control

        Trapazoidal rule used for integrator and a low-pass filter for derivative

        Args:
            y_ref: desired value
            y: actual value
            reset_flag: True => the integrator and derivative calculation will be zeroed out

        Output:
            u_sat: thresholded control output
        """
        if reset_flag is True:
            self.integrator = 0.0
            self.error_delay_1 = 0.0
            self.y_dot = 0.0
            self.y_delay_1 = 0.0
            self.y_dot_delay_1 = 0.0
        # compute the error
        error = y_ref - y
        # update the integrator using trapazoidal rule
        self.integrator = self.integrator + (self.Ts / 2) * (error + self.error_delay_1)
        # update the differentiator
        error_dot = self.a1 * self.error_dot_delay_1 + self.a2 * (
            error - self.error_delay_1
        )
        # PID control
        u = self.kp * error + self.ki * self.integrator + self.kd * error_dot
        # saturate PID control at limit
        u_sat = saturate(u, -self.limit, self.limit)
        # integral anti-windup
        #   adjust integrator to keep u out of saturation
        if np.abs(self.ki) > 0.0001:
            self.integrator = self.integrator + (self.Ts / self.ki) * (u_sat - u)
        # update the delayed variables
        self.error_delay_1 = error
        self.error_dot_delay_1 = error_dot
        return u_sat

    def update_with_rate(
        self, y_ref: float, y: float, ydot: float, reset_flag: bool = False
    ) -> float:
        """Calculate the PID control given the derivative value

        Integral calculated using trapazoidal rule

        Args:
            y_ref: desired value
            y: actual value
            ydot: time derivative
            reset_flag: True => the integrator and derivative calculation will be zeroed out

        Output:
            u_sat: thresholded control output
        """
        if reset_flag is True:
            self.integrator = 0.0
            self.error_delay_1 = 0.0
        # compute the error
        error = y_ref - y
        # update the integrator using trapazoidal rule
        self.integrator = self.integrator + (self.Ts / 2) * (error + self.error_delay_1)
        # PID control
        u = self.kp * error + self.ki * self.integrator - self.kd * ydot
        # saturate PID control at limit
        u_sat = saturate(u, -self.limit, self.limit)
        # integral anti-windup
        #   adjust integrator to keep u out of saturation
        if np.abs(self.ki) > 0.0001:
            self.integrator = self.integrator + (self.Ts / self.ki) * (u_sat - u)
        self.error_delay_1 = error
        return u_sat

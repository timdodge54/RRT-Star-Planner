"""
pi_control
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
        1/2022 - GND
"""
# import numpy as np
from mav_sim.tools.wrap import saturate


class PIControl:
    """Proportional integral controller"""

    def __init__(
        self, kp: float = 0.0, ki: float = 0.0, Ts: float = 0.01, limit: float = 1.0
    ) -> None:
        """Store control gains

        Args:
            kp: proportional gain
            ki: integral gain
            Ts: update period
            limit: maximum magnitude for the control
        """
        self.kp = kp
        self.ki = ki
        self.Ts = Ts
        self.limit = limit
        self.integrator: float = 0.0
        self.error_delay_1: float = 0.0

    def update(self, y_ref: float, y: float) -> float:
        """Compute the proportional integral control given the reference and actual value.
           The control is saturated based upon self.limit and a simple anti-windup procedure
           is performed on the integrator.

        Args:
            y_ref: desired value
            y: actual value

        Returns:
            u_sat: the saturated control input
        """

        # compute the error
        # print(f"Integration: yref: {y_ref}, y:{y}")
        error = y_ref - y

        # Do something with the error
        I_n = self.integrator + (self.Ts / 2.0) * (error + self.error_delay_1)

        u = self.kp * (error) + self.ki * I_n
        u_sat = saturate(u, -self.limit, self.limit)

        # integral anti-windup
        if self.ki != 0:
            delta_I = self.Ts / self.ki * (u_sat - u)
            I_p = I_n + delta_I
            self.integrator = I_p
        else:
            self.integrator = I_n

        # update the delayed variables
        self.error_delay_1 = error
        return u_sat

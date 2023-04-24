"""
pd_control
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
        12/21 - GND
"""
from mav_sim.tools.wrap import saturate


class PDControlWithRate:
    """
    PD control with rate information (i.e., the derivative is given as an input to update function)
    u = kp*(yref-y) - kd*ydot
    """

    def __init__(self, kp: float = 0.0, kd: float = 0.0, limit: float = 1.0) -> None:
        """Store control values

        Args:
            kp: proportional gain
            kd: derivative gain
            limit: max value of the control
        """
        self.kp = kp
        self.kd = kd
        self.limit = limit

    def update(self, y_ref: float, y: float, ydot: float) -> float:
        """Calculates the proportional-derivative control. The output is saturated by self.limit

        Args:
            y_ref: desired value
            y: actual value
            ydot: derivative of actual value

        Output:
            u_sat: a saturated pd controller
        """
        # print(f"y_ref: {y_ref}, y:{y}, ydot: {ydot}")
        u = self.kp * (y_ref - y) - self.kd * ydot
        # print(f"u: {u}")
        u_sat = saturate(u, -self.limit, self.limit)
        return u_sat

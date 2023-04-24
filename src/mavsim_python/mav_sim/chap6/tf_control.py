"""
tf_control
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/10/2021 - TWM

    Difference equation implementation of continous-time first-order transfer function
    using trapezoidal rule approximation. Used in yaw-damper implementation.

    Transfer function form:  H(s) = u(s)/y(s) = k * (b1*s + b0)/(a1*s + a0)
"""
# pylint: disable=too-many-arguments
from mav_sim.tools.wrap import saturate


class TFControl:
    """Class for TF control calculation"""

    def __init__(
        self,
        k: float = 0.0,
        n0: float = 0.0,
        n1: float = 0.0,
        d0: float = 0.0,
        d1: float = 0.0,
        Ts: float = 0.01,
        limit: float = 1.0,
    ) -> None:
        """Initialize control variables

            H(s) = u(s)/y(s) = k * (n1*s + n0)/(d1*s + d0)

        Args:
            k: TF control gain
            n0: numerator constant
            n1: numerator scaling of s
            d0: denominator constant
            d1: denominator scaling of s
            Ts: sampling time
            limit: max amplitude for the control
        """
        self.k = k
        self.n0 = n0
        self.n1 = n1
        self.d0 = d0
        self.d1 = d1
        self.Ts = Ts
        self.limit = limit
        self.y = 0.0
        self.u = 0.0
        self.y_delay_1 = 0.0
        self.u_delay_1 = 0.0

        # coefficients for difference equation
        self.b0 = k * (2.0 * n1 + Ts * n0) / (2.0 * d1 + Ts * d0)
        self.b1 = -k * (2.0 * n1 - Ts * n0) / (2.0 * d1 + Ts * d0)
        self.a1 = -(2.0 * d1 - Ts * d0) / (2.0 * d1 + Ts * d0)

    def update(self, y: float) -> float:
        """Calculate the tf control

        Args:
            y: current value

        Returns:
            u_sat: saturated control
        """
        u_k = -self.a1 * self.u_delay_1 + self.b0 * y + self.b1 * self.y_delay_1
        # saturate transfer function output at limit
        u_sat = saturate(u_k, -self.limit, self.limit)

        # update the delayed variables
        self.y_delay_1 = y
        self.u_delay_1 = u_sat
        return u_sat

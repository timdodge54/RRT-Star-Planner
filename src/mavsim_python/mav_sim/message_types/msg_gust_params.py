"""msg_gust_params.py defines the parameters for Dryden transfer functions
"""


class MsgGustParams:
    """
    Provides parameters for a gust
    """

    def __init__(self) -> None:
        """Initialize parameters to a low altitude, light turbeulence scenario"""

        # Turbulance intensities along the vehicle frame axes
        self.sigma_u: float = 1.06
        self.sigma_v: float = 1.06
        self.sigma_w: float = 0.7

        # Spatial wavelengths
        self.Lu: float = 200
        self.Lv: float = 200
        self.Lw: float = 50

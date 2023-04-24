"""
msg_autopilot
    - messages type for input to the autopilot

part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        2/5/2019 - RWB
"""
from typing import Any

import numpy as np
import numpy.typing as npt


class MsgAutopilot:
    """Message class for commanding the autopilot."""

    __slots__ = [
        "airspeed_command",
        "course_command",
        "altitude_command",
        "phi_feedforward",
    ]

    def __init__(self) -> None:
        """Default parameters to zero"""
        self.airspeed_command: float = 0.0  # commanded airspeed m/s
        self.course_command: float = 0.0  # commanded course angle in rad
        self.altitude_command: float = 0.0  # commanded altitude in m
        self.phi_feedforward: float = 0.0  # feedforward command for roll angle

    def to_array(self) -> npt.NDArray[Any]:
        """Convert the command to a numpy array."""
        return np.array(
            [
                [self.airspeed_command],
                [self.course_command],
                [self.altitude_command],
                [self.phi_feedforward],
            ],
            dtype=float,
        )

    def __str__(self) -> str:
        """Returns a string of the class"""
        out = (
            "airspeed_command: "
            + str(self.airspeed_command)
            + ", course_command: "
            + str(self.course_command)
            + ", altitude_command: "
            + str(self.altitude_command)
            + ", phi_feedforward: "
            + str(self.phi_feedforward)
        )
        return out

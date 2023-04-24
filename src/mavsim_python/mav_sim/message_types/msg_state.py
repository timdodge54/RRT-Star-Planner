"""msgState: messages type for state, that will be passed between blocks in the architecture.

part of mavPySim
    - Beard & McLain, PUP, 2012
    - Update history:
        1/9/2019 - RWB
        12/21 - GND
"""
from typing import Any

import numpy as np
import numpy.typing as npt


class MsgState:
    """Defines the state of the aircraft."""

    __slots__ = [
        "north",
        "east",
        "altitude",
        "phi",
        "theta",
        "psi",
        "Va",
        "alpha",
        "beta",
        "p",
        "q",
        "r",
        "Vg",
        "gamma",
        "chi",
        "wn",
        "we",
        "bx",
        "by",
        "bz",
    ]
    # pylint: disable=too-many-arguments
    def __init__(
        self,
        north: float = 0,
        east: float = 0,
        altitude: float = 0,
        phi: float = 0,
        theta: float = 0,
        psi: float = 0,
        Va: float = 0,
        alpha: float = 0,
        beta: float = 0,
        p: float = 0,
        q: float = 0,
        r: float = 0,
        Vg: float = 0,
        gamma: float = 0,
        chi: float = 0,
        wn: float = 0,
        we: float = 0,
        bx: float = 0,
        by: float = 0,
        bz: float = 0,
    ) -> None:
        self.north: float = north  # inertial north position in meters
        self.east: float = east  # inertial east position in meters
        self.altitude: float = altitude  # inertial altitude in meters
        self.phi: float = phi  # roll angle in radians
        self.theta: float = theta  # pitch angle in radians
        self.psi: float = psi  # yaw angle in radians
        self.Va: float = Va  # airspeed in meters/sec
        self.alpha: float = alpha  # angle of attack in radians
        self.beta: float = beta  # sideslip angle in radians
        self.p: float = p  # roll rate in radians/sec
        self.q: float = q  # pitch rate in radians/sec
        self.r: float = r  # yaw rate in radians/sec
        self.Vg: float = Vg  # groundspeed in meters/sec
        self.gamma: float = gamma  # flight path angle in radians
        self.chi: float = chi  # course angle in radians
        self.wn: float = wn  # inertial windspeed in north direction in meters/sec
        self.we: float = we  # inertial windspeed in east direction in meters/sec
        self.bx: float = bx  # gyro bias along roll axis in radians/sec
        self.by: float = by  # gyro bias along pitch axis in radians/sec
        self.bz: float = bz  # gyro bias along yaw axis in radians/sec

    def to_array(self) -> npt.NDArray[Any]:
        """Convert the command to a numpy array."""
        return np.array(
            [
                [self.north],
                [self.east],
                [self.altitude],
                [self.phi],
                [self.theta],
                [self.psi],
                [self.Va],
                [self.alpha],
                [self.beta],
                [self.p],
                [self.q],
                [self.r],
                [self.Vg],
                [self.gamma],
                [self.chi],
                [self.wn],
                [self.we],
                [self.bx],
                [self.by],
                [self.bz],
            ],
            dtype=float,
        )

    def copy(self) -> "MsgState":
        """Returns a copy of all of the variables"""
        msg_copy = MsgState()
        msg_copy.north = self.north
        msg_copy.east = self.east
        msg_copy.altitude = self.altitude
        msg_copy.phi = self.phi
        msg_copy.theta = self.theta
        msg_copy.psi = self.psi
        msg_copy.Va = self.Va
        msg_copy.alpha = self.alpha
        msg_copy.beta = self.beta
        msg_copy.p = self.p
        msg_copy.q = self.q
        msg_copy.r = self.r
        msg_copy.Vg = self.Vg
        msg_copy.gamma = self.gamma
        msg_copy.chi = self.chi
        msg_copy.wn = self.wn
        msg_copy.we = self.we
        msg_copy.bx = self.bx
        msg_copy.by = self.by
        msg_copy.bz = self.bz
        return msg_copy

    def print(self) -> None:
        """Print the commands to the console."""
        print(self)

    def __str__(self) -> str:
        """Returns a string representation"""
        out = (
            "north="
            + str(self.north)
            + "\n"
            + "east="
            + str(self.east)
            + "\n"
            + "altitude="
            + str(self.altitude)
            + "\n"
            + "phi="
            + str(self.phi)
            + "\n"
            + "theta="
            + str(self.theta)
            + "\n"
            + "psi="
            + str(self.psi)
            + "\n"
            + "Va="
            + str(self.Va)
            + "\n"
            + "alpha="
            + str(self.alpha)
            + "\n"
            + "beta="
            + str(self.beta)
            + "\n"
            + "p="
            + str(self.p)
            + "\n"
            + "q="
            + str(self.q)
            + "\n"
            + "r="
            + str(self.r)
            + "\n"
            + "Vg="
            + str(self.Vg)
            + "\n"
            + "gamma="
            + str(self.gamma)
            + "\n"
            + "chi="
            + str(self.chi)
            + "\n"
            + "wn="
            + str(self.wn)
            + "\n"
            + "we="
            + str(self.we)
            + "\n"
            + "bx="
            + str(self.bx)
            + "\n"
            + "by="
            + str(self.by)
            + "\n"
            + "bz="
            + str(self.bz)
            + "\n"
        )
        return out

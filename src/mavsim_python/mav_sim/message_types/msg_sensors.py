"""msg_sensors: messages type for output of sensors.

part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        2/16/2019 - RWB
        12/21 - GND
"""

from typing import Any

import numpy as np
import numpy.typing as npt


class MsgSensors:
    """Defines the sensor message."""

    # pylint: disable=too-many-arguments
    def __init__(
        self,
        gyro_x: float = 0,
        gyro_y: float = 0,
        gyro_z: float = 0,
        accel_x: float = 0,
        accel_y: float = 0,
        accel_z: float = 0,
        mag_x: float = 0,
        mag_y: float = 0,
        mag_z: float = 0,
        abs_pressure: float = 0,
        diff_pressure: float = 0,
        gps_n: float = 0,
        gps_e: float = 0,
        gps_h: float = 0,
        gps_Vg: float = 0,
        gps_course: float = 0,
    ) -> None:
        self.gyro_x: float = gyro_x  # gyroscope along body x axis
        self.gyro_y: float = gyro_y  # gyroscope along body y axis
        self.gyro_z: float = gyro_z  # gyroscope along body z axis
        self.accel_x: float = accel_x  # specific acceleration along body x axis
        self.accel_y: float = accel_y  # specific acceleration along body y axis
        self.accel_z: float = accel_z  # specific acceleration along body z axis
        self.mag_x: float = mag_x  # magnetic field along body x axis
        self.mag_y: float = mag_y  # magnetic field along body y axis
        self.mag_z: float = mag_z  # magnetic field along body z axis
        self.abs_pressure: float = abs_pressure  # absolute pressure
        self.diff_pressure: float = diff_pressure  # differential pressure
        self.gps_n: float = gps_n  # gps north
        self.gps_e: float = gps_e  # gps east
        self.gps_h: float = gps_h  # gps altitude
        self.gps_Vg: float = gps_Vg  # gps ground speed
        self.gps_course: float = gps_course  # gps course angle

    def to_array(self) -> npt.NDArray[Any]:
        """Convert the command to a numpy array."""
        return np.array(
            [
                [self.gyro_x],
                [self.gyro_y],
                [self.gyro_z],
                [self.accel_x],
                [self.accel_y],
                [self.accel_z],
                [self.mag_x],
                [self.mag_y],
                [self.mag_z],
                [self.abs_pressure],
                [self.diff_pressure],
                [self.gps_n],
                [self.gps_e],
                [self.gps_h],
                [self.gps_Vg],
                [self.gps_course],
            ],
            dtype=float,
        )

    def print(self) -> None:
        """Print the commands to the console."""
        print(
            "gyro_x=",
            self.gyro_x,
            "gyro_y=",
            self.gyro_y,
            "gyro_z=",
            self.gyro_z,
            "accel_x=",
            self.accel_x,
            "accel_y=",
            self.accel_y,
            "accel_z=",
            self.accel_z,
            "mag_x=",
            self.mag_x,
            "mag_y=",
            self.mag_y,
            "mag_z=",
            self.mag_z,
            "abs_pressure=",
            self.abs_pressure,
            "diff_pressure=",
            self.diff_pressure,
            "gps_n=",
            self.gps_n,
            "gps_e=",
            self.gps_e,
            "gps_h=",
            self.gps_h,
            "gps_Vg=",
            self.gps_Vg,
            "gps_course=",
            self.gps_course,
        )

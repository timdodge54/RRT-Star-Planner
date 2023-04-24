"""types.py Defines several types used throughout the code
"""

import typing
from typing import Any, Callable, Union, cast

import numpy as np
import numpy.typing as npt


class Pose(typing.Protocol):
    """Pose information

    Contains the position and orientation of the MAV
    """

    north: float  # north position
    east: float  # east position
    altitude: float  # altitude
    phi: float  # roll angle
    theta: float  # pitch angle
    psi: float  # yaw angle


class AttitudeMeasurement(typing.Protocol):
    """Struct with Attitude measurement data"""

    gyro_x: float  # gyroscope along body x axis
    gyro_y: float  # gyroscope along body y axis
    gyro_z: float  # gyroscope along body z axis
    accel_x: float  # specific acceleration along body x axis
    accel_y: float  # specific acceleration along body y axis
    accel_z: float  # specific acceleration along body z axis
    diff_pressure: float  # differential pressure


class AttitudeState(typing.Protocol):
    """Struct with the gyro biases"""

    phi: float  # roll angle in radians
    theta: float  # pitch angle in radians
    psi: float  # yaw angle in radians
    bx: float  # gyro bias along roll axis in radians/sec
    by: float  # gyro bias along pitch axis in radians/sec
    bz: float  # gyro bias along yaw axis in radians/sec
    Va: float  # airspeed in meters/sec


class PositionMeasurement(typing.Protocol):
    """Struct with the position measurement data"""

    gps_n: float  # gps north
    gps_e: float  # gps east
    gps_h: float  # gps altitude
    gps_Vg: float  # gps ground speed
    gps_course: float  # gps course angle


class PositionState(typing.Protocol):
    """Struct with states needed for position estimate"""

    north: float  # inertial north position in meters
    east: float  # inertial east position in meters
    phi: float  # roll angle in radians
    theta: float  # pitch angle in radians
    psi: float  # yaw angle in radians
    Va: float  # airspeed in meters/sec
    Vg: float  # groundspeed in meters/sec
    chi: float  # course angle in radians
    wn: float  # inertial windspeed in north direction in meters/sec
    we: float  # inertial windspeed in east direction in meters/sec
    p: float  # roll rate in radians/sec
    q: float  # pitch rate in radians/sec
    r: float  # yaw rate in radians/sec


SensorJacobianFnc = Union[
    Callable[[npt.NDArray[Any], AttitudeMeasurement, AttitudeState], npt.NDArray[Any]],
    Callable[[npt.NDArray[Any], PositionMeasurement, PositionState], npt.NDArray[Any]],
]
JacobianMeasurement = Union[AttitudeMeasurement, PositionMeasurement]
JacobianState = Union[AttitudeState, PositionState]


Vector = typing.NewType("Vector", npt.NDArray[Any])  # 3x1 vector
Points = typing.NewType("Points", npt.NDArray[Any])  # 3xn vector
Quaternion = npt.NDArray[Any]  # 4x1 quaternion
RotMat = typing.NewType("RotMat", npt.NDArray[Any])  # 3x3 rotation matrix
SkewSymMat = typing.NewType("SkewSymMat", npt.NDArray[Any])  # 3x3 skew symmetric matrix
DynamicState = npt.NDArray[
    Any
]  # 13x1 array of state elements assuming quaternion attitude
DynamicStateEuler = npt.NDArray[Any]  # 12x1 array \
# of state elements assuming euler coordinates for attitude
ForceMoment = typing.NewType(
    "ForceMoment", npt.NDArray[Any]
)  # 6x1 array of force/moment elements
WindVector = typing.NewType("WindVector", npt.NDArray[Any])  # 6x1 array of wind -
#   The first three elements are the steady state wind in the inertial frame
#   The second three elements are the gust in the body frame
NP_MAT = npt.NDArray[Any]


def check_valid_dimensions(
    mat: npt.NDArray[Any], rows: int = -1, cols: int = -1
) -> None:
    """Checks matrix dimensions.

    Raises a value error if the input matrix has the incorrect dimensions.

    Args:
        mat: Matrix to be checked
        rows: Number of rows that it should have (-1 if the # of rows is not needed)
        cols: Number of columns that the matrix should have (-1 if the # of cols is not needed)
    """
    # Extract the shape
    shape = np.shape(mat)

    # Check rows
    if rows >= 0 and rows != shape[0]:
        raise ValueError("Number of rows should be " + str(rows))

    # Check cols
    if cols >= 0 and cols != shape[0]:
        raise ValueError("Number of columns should be " + str(cols))


def check_vector_size(vec: npt.NDArray[Any], size: int) -> None:
    """Checks vector dimensions

    Raises a value error if the input vector has the incorrect dimensions

    Args:
        vec: Vector to be checked
        size: number of elements that the vector should have
    """

    if size != np.size(vec):
        raise ValueError("Number of elements should be " + str(size))


def check_rotation_matrix(R: RotMat) -> None:
    """Checks that the rotation matrix has the appropriate dimensions"""
    check_valid_dimensions(cast(npt.NDArray[Any], R), 3, 3)


def check_skew_symmetric_matrix(mat: SkewSymMat) -> None:
    """Checks that the skew symmetric matrix has the appropriate dimensions"""
    check_valid_dimensions(cast(npt.NDArray[Any], mat), 3, 3)


def check_vector(vec: Vector) -> None:
    """Checks that vector has three elements"""
    check_vector_size(cast(npt.NDArray[Any], vec), 3)


def check_quaternion(quat: Quaternion) -> None:
    """Checks that quaternion has four elements"""
    check_vector_size(cast(npt.NDArray[Any], quat), 4)


def check_points(points: Points) -> None:
    """Checks that points have three rows and any number of columns"""
    check_valid_dimensions(cast(npt.NDArray[Any], points), rows=3)

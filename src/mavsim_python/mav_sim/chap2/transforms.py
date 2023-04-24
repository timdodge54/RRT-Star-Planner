"""transforms.py implements the basic transforms from chapter 2

There are seven frames defined in chapter two:
    Iniertial frame (i)         - Fixed to earth
    Vehicle frame (v)           - Parellel to (i), but fixed to uav
    Vehicle 1 frame (v1)        - Rotate yaw (psi) about k^v
    Vehicle 2 frame (v2)        - Rotate pitch (theta) about j^{v1}
    Body frame (b)              - Rotate roll (phi) about i^{v2}
    Stability frame (s)         - Rotate left hand alpha about j^b
    Wind frame (w)              - Rotate beta about k^s

Functions:
    This module contains a number of functions for calculating basic rotations
        rot_x: calculate elementary rotation matrix about x-axis
        rot_y: calculate elementary rotation matrix about y-axis
        rot_z: calculate elementary rotation matrix about z-axis

    This module also contains a number of functions for calculating basic rotation matrices
        rot_v_to_v1: calculates the rotation from frame v to v1
        rot_v1_to_v2: calculates the rotation from frame v1 to v2
        rot_v2_to_b: calculates the rotation from v2 to body frame
        rot_b_to_s: calculates the rotation from body to stability frame
        rot_s_to_w: calculates the rotation from stability to wind frame
        rot_v_to_b: calculates the rotation from vehicle to body frame
        rot_b_to_v: calculates the rotation from body frame to vehicle frame

    This module also computes a number of functions for calculating the transforms of points
        trans_i_to_v: transforms a point from inertial frame to the vehicle frame
        trans_v_to_i: transforms a point from vehicle frame to the inertial frame
        trans_i_to_b: transforms a point from inertial frame to the body frame
        trans_b_to_i: transforms a point from the body frame to the inertial frame
"""

from typing import cast

import numpy as np
import numpy.typing as npt
from mav_sim.tools.types import Points, Pose, RotMat


# Elementary rotation matrices
def rot_x(angle: float) -> RotMat:
    """Elementary rotation about x-axis.

    Args:
        angle: angle of rotation

    Returns:
        rot: rotation matrix about x-axis
    """
    # Calculate rotation matrix
    rot: npt.NDArray[np.float64] = np.array(
        [
            [1, 0, 0],
            [0, np.cos(angle), np.sin(angle)],
            [0, -np.sin(angle), np.cos(angle)],
        ]
    )
    return cast(RotMat, rot)


def rot_y(angle: float) -> RotMat:
    """Elementary rotation about y-axis.

    Args:
        angle: angle of rotation

    Returns:
        rot: rotation matrix about y-axis
    """
    # Calculate rotation matrix
    rot: npt.NDArray[np.float64] = np.array(
        [
            [np.cos(angle), 0, -np.sin(angle)],
            [0, 1, 0],
            [np.sin(angle), 0, np.cos(angle)],
        ]
    )
    return cast(RotMat, rot)


def rot_z(angle: float) -> RotMat:
    """Elementary rotation about z-axis.

    Args:
        angle: angle of rotation

    Returns:
        rot: rotation matrix about z-axis
    """
    # Calculate rotation matrix
    rot: npt.NDArray[np.float64] = np.array(
        [
            [np.cos(angle), np.sin(angle), 0],
            [-np.sin(angle), np.cos(angle), 0],
            [0, 0, 1],
        ]
    )
    return cast(RotMat, rot)


# Rotation matrices between concentric frames
def rot_v_to_v1(psi: float) -> RotMat:
    """
    calculates the rotation from frame v to v1

    Args:
        psi: yaw angle about k^v axis

    Returns:
        rot: Rotation matrix from frame v to v1
    """
    # Calculate rotaiton matrix
    rot = rot_z(psi)
    return rot


def rot_v1_to_v2(theta: float) -> RotMat:
    """
    calculates the rotation from frame v1 to v2

    Args:
        theta: pitch angle about j^{v1} axis

    Returns:
        rot: Rotation matrix from frame v1 to v2
    """
    # Calculate rotaiton matrix
    rot = rot_y(theta)
    return rot


def rot_v2_to_b(phi: float) -> RotMat:
    """
    calculates the rotation from frame v2 to body

    Args:
        phi: roll angle about i^{v2} axis

    Returns:
        rot: Rotation matrix from frame v2 to b
    """
    # Calculate rotaiton matrix
    rot = rot_x(phi)
    return rot


def rot_b_to_s(alpha: float) -> RotMat:
    """
    calculates the rotation from body frame to stability frame

    Args:
        alpha: left-hand rotation angle about j^b to align i^s with projection of V_a onto i^b-k^b plane

    Returns:
        rot: Rotation matrix from body frame to stability frame
    """
    # Calculate rotaiton matrix
    rev_alpha = alpha
    rot = np.transpose(rot_y(rev_alpha))
    return cast(RotMat, rot)


def rot_s_to_w(beta: float) -> RotMat:
    """
    calculates the rotation from stability frame to wind frame

    Args:
        beta: rotation about k^s axis to align i^w with V_a

    Returns:
        rot: Rotation matrix from body frame to stability frame
    """
    # Calculate rotaiton matrix
    rot = rot_z(beta)
    return rot


def rot_v_to_b(psi: float, theta: float, phi: float) -> RotMat:
    """
    calculates the rotation matrix from vehicle frame to body frame

    Args:
        psi: yaw angle about k^v axis
        theta: pitch angle about j^{v1} axis
        phi: roll angle about i^{v2} axis

    Returns:
        rot: Rotation matrix from vehicle frame to body frame
    """
    # Calculate rotaiton matrix
    r_v_v1 = rot_z(psi)
    r_v1_v2 = rot_y(theta)
    r_v2_b = rot_x(phi)
    rot = r_v2_b @ r_v1_v2 @ r_v_v1
    return cast(RotMat, rot)


def rot_b_to_v(psi: float, theta: float, phi: float) -> RotMat:
    """
    calculates the rotation matrix from body frame to vehicle frame

    Args:
        psi: yaw angle about k^v axis
        theta: pitch angle about j^{v1} axis
        phi: roll angle about i^{v2} axis

    Returns:
        rot: Rotation matrix from body frame to vehicle frame
    """
    # Calculate rotaiton matrix
    r_b_v2 = np.transpose(rot_x(phi))
    r_v2_v1 = np.transpose(rot_y(theta))
    r_v1_v = np.transpose(rot_z(psi))
    rot = r_v1_v @ r_v2_v1 @ r_b_v2
    return cast(RotMat, rot)


# Calculating the transforms of points
def trans_i_to_v(pose: Pose, p_i: Points) -> Points:
    """
    Transforms a point from inertial frame to the vehicle frame

    Args:
        p_i: Point represented in the inertial frame

    Returns:
        p_v: Point represented in the vehicle frame
    """
    # print(f"pose n: {pose.north}, type: {type(pose.north)}")
    # print(f"p_i: {p_i}, type: {type(p_i)}, p_i[0]: {p_i[0]}, type: {type(p_i[0])}")
    p_i[0][0] -= pose.north
    p_i[1][0] -= pose.east
    p_i[2][0] += pose.altitude
    return p_i


def trans_v_to_i(pose: Pose, p_v: Points) -> Points:
    """
    Transforms a point from vehicle frame to the inertial frame

    Args:
        p_v: Point represented in the vehicle frame

    Returns:
        p_i: Point represented in the inertial frame
    """
    # print(f"P_v: {p_v}, type: {type(p_v)}")
    p_v[0][0] += pose.north
    p_v[1][0] += pose.east
    p_v[2][0] -= pose.altitude
    return p_v


def trans_i_to_b(pose: Pose, p_i: Points) -> Points:
    """
    Transforms a point from inertial frame to the body frame

    Args:
        p_i: Point represented in the inertial frame

    Returns:
        p_b: Point represented in the body frame
    """
    p_i = trans_i_to_v(pose, p_i)
    p_i = rot_v_to_b(pose.psi, pose.theta, pose.phi) @ p_i
    return cast(Points, p_i)


def trans_b_to_i(pose: Pose, p_b: Points) -> Points:
    """
    Transforms a point from body frame to the inertial frame

    Args:
        p_b: Point represented in the body frame

    Returns:
        p_i: Point represented in the inertial frame
    """
    p_i = rot_b_to_v(pose.psi, pose.theta, pose.phi) @ p_b
    p_i = trans_v_to_i(pose, p_i)  # type: ignore
    return p_i

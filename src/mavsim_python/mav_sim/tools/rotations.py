"""
various tools to be used in mavPySim

Conversions between different orientation representations
"""
from typing import cast

import numpy as np
from mav_sim.chap2.transforms import rot_b_to_v
from mav_sim.tools import types
from scipy import linalg


def Quaternion2Euler(
    quaternion: types.Quaternion,
) -> tuple[float, float, float]:  # pylint: disable=invalid-name
    """
    converts a quaternion attitude to an euler angle attitude
    :param quaternion: the quaternion to be converted to euler angles in a np.matrix
    :return: the euler angle equivalent (phi, theta, psi) in a np.array
    """
    # Perform conversion
    e0 = quaternion.item(0)
    e1 = quaternion.item(1)
    e2 = quaternion.item(2)
    e3 = quaternion.item(3)
    # np.seterr("raise")
    # try:
    phi = np.arctan2(
        2.0 * (e0 * e1 + e2 * e3), e0 ** 2.0 + e3 ** 2.0 - e1 ** 2.0 - e2 ** 2.0
    )
    theta = np.arcsin(2.0 * (e0 * e2 - e1 * e3))
    psi = np.arctan2(
        2.0 * (e0 * e3 + e1 * e2), e0 ** 2.0 + e1 ** 2.0 - e2 ** 2.0 - e3 ** 2.0
    )
    # except Exception as e:
    #     print(f"Exception: {e}")
    #     print(f"{2 * (e0 * e2 + e1 * e3)}")
    #     print(f"e0: {e0}, e1: {e1}, e2: {e2}, e3: {e3}")
    #     raise

    return phi, theta, psi


def Euler2Quaternion(
    phi: float, theta: float, psi: float
) -> types.Quaternion:  # pylint: disable=invalid-name
    """
    Converts an euler angle attitude to a quaternian attitude
    :param euler: Euler angle attitude in a np.matrix(phi, theta, psi)
    :return: Quaternian attitude in np.array(e0, e1, e2, e3)
    """
    # Perform conversion
    e0 = np.cos(psi / 2.0) * np.cos(theta / 2.0) * np.cos(phi / 2.0) + np.sin(
        psi / 2.0
    ) * np.sin(theta / 2.0) * np.sin(phi / 2.0)
    e1 = np.cos(psi / 2.0) * np.cos(theta / 2.0) * np.sin(phi / 2.0) - np.sin(
        psi / 2.0
    ) * np.sin(theta / 2.0) * np.cos(phi / 2.0)
    e2 = np.cos(psi / 2.0) * np.sin(theta / 2.0) * np.cos(phi / 2.0) + np.sin(
        psi / 2.0
    ) * np.cos(theta / 2.0) * np.sin(phi / 2.0)
    e3 = np.sin(psi / 2.0) * np.cos(theta / 2.0) * np.cos(phi / 2.0) - np.cos(
        psi / 2.0
    ) * np.sin(theta / 2.0) * np.sin(phi / 2.0)

    return np.array([[e0], [e1], [e2], [e3]])


def Euler2Rotation(
    phi: float, theta: float, psi: float
) -> types.RotMat:  # pylint: disable=invalid-name
    """
    Converts euler angles to rotation matrix (R_b^i)

    Note that R_b^i is the same as R_b^v as the transform from the vehicle frame to the
    inertial frame is purely translation
    """

    return rot_b_to_v(psi, theta, phi)


def Quaternion2Rotation(
    quaternion: types.Quaternion,
) -> types.RotMat:  # pylint: disable=invalid-name
    """
    converts a quaternion attitude to a rotation matrix
    """
    # Check input
    types.check_quaternion(quaternion)

    # Extract quaternion elements
    e0 = quaternion.item(0)
    e1 = quaternion.item(1)
    e2 = quaternion.item(2)
    e3 = quaternion.item(3)

    # Perform conversion
    R = np.array(
        [
            [
                e1 ** 2.0 + e0 ** 2.0 - e2 ** 2.0 - e3 ** 2.0,
                2.0 * (e1 * e2 - e3 * e0),
                2.0 * (e1 * e3 + e2 * e0),
            ],
            [
                2.0 * (e1 * e2 + e3 * e0),
                e2 ** 2.0 + e0 ** 2.0 - e1 ** 2.0 - e3 ** 2.0,
                2.0 * (e2 * e3 - e1 * e0),
            ],
            [
                2.0 * (e1 * e3 - e2 * e0),
                2.0 * (e2 * e3 + e1 * e0),
                e3 ** 2.0 + e0 ** 2.0 - e1 ** 2.0 - e2 ** 2.0,
            ],
        ]
    )
    R = types.RotMat(R / linalg.det(R))

    return R


def Rotation2Quaternion(
    R: types.RotMat,
) -> types.Quaternion:  # pylint: disable=invalid-name
    """
    converts a rotation matrix to a unit quaternion
    """
    # Check input
    types.check_rotation_matrix(R)

    # Extract matrix elements
    r11 = R[0][0]
    r12 = R[0][1]
    r13 = R[0][2]
    r21 = R[1][0]
    r22 = R[1][1]
    r23 = R[1][2]
    r31 = R[2][0]
    r32 = R[2][1]
    r33 = R[2][2]

    # Perform conversion
    tmp = r11 + r22 + r33
    if tmp > 0:
        e0 = 0.5 * np.sqrt(1 + tmp)
    else:
        e0 = 0.5 * np.sqrt(
            ((r12 - r21) ** 2 + (r13 - r31) ** 2 + (r23 - r32) ** 2) / (3 - tmp)
        )

    tmp = r11 - r22 - r33
    if tmp > 0:
        e1 = 0.5 * np.sqrt(1 + tmp)
    else:
        e1 = 0.5 * np.sqrt(
            ((r12 + r21) ** 2 + (r13 + r31) ** 2 + (r23 - r32) ** 2) / (3 - tmp)
        )

    tmp = -r11 + r22 - r33
    if tmp > 0:
        e2 = 0.5 * np.sqrt(1 + tmp)
    else:
        e2 = 0.5 * np.sqrt(
            ((r12 + r21) ** 2 + (r13 + r31) ** 2 + (r23 + r32) ** 2) / (3 - tmp)
        )

    tmp = -r11 + -22 + r33
    if tmp > 0:
        e3 = 0.5 * np.sqrt(1 + tmp)
    else:
        e3 = 0.5 * np.sqrt(
            ((r12 - r21) ** 2 + (r13 + r31) ** 2 + (r23 + r32) ** 2) / (3 - tmp)
        )

    return types.Quaternion(np.array([[e0], [e1], [e2], [e3]]))


def rotz(theta: float) -> types.RotMat:
    """Calculates the rotation matrix about the z axis

    Args:
        theta: Angle of rotation

    Returns:
        Resulting rotation about z-axis
    """

    return cast(
        types.RotMat,
        np.array(
            [
                [np.cos(theta), -np.sin(theta), 0],
                [np.sin(theta), np.cos(theta), 0],
                [0, 0, 1],
            ]
        ),
    )


def hat(omega: types.Vector) -> types.SkewSymMat:
    """
    vector to skew symmetric matrix associated with cross product
    """
    # Check input
    types.check_vector(omega)

    a = omega.item(0)
    b = omega.item(1)
    c = omega.item(2)

    omega_hat = np.array([[0, -c, b], [c, 0, -a], [-b, a, 0]])
    return types.SkewSymMat(omega_hat)

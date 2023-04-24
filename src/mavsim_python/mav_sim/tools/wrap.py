"""
Functions for aiding in thresholding
"""
import numpy as np


def wrap(chi_1: float, chi_2: float) -> float:
    """wrap chi_1, so that it is within +-pi of chi_2."""
    while chi_1 - chi_2 > np.pi:
        chi_1 = chi_1 - 2.0 * np.pi
    while chi_1 - chi_2 < -np.pi:
        chi_1 = chi_1 + 2.0 * np.pi
    return chi_1


def saturate(in_val: float, low_limit: float, up_limit: float) -> float:
    """Threshold the input to be within the lower and upper bounds

    Args:
        in_val: value to be thresholded
        low_limit: the minimum value that can be held
        up_limit: the maximum value that can be held

    Returns:
        output: the thresholded value
    """
    if in_val <= low_limit:
        output = low_limit
    elif in_val >= up_limit:
        output = up_limit
    else:
        output = in_val
    return output

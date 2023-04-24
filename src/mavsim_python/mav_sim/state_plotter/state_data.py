"""state_data.py Stores the data for each state
"""
from typing import Any, Optional, Union, cast

import numpy as np
import numpy.typing as npt


class StateData:
    """Object for storing data for plots"""

    def __init__(
        self,
        sigma_bounds: Optional[list[int]] = None,
        max_length: Optional[int] = None,
        is_angle: bool = False,
        rad2deg: bool = False,
    ) -> None:
        self.data: list[float] = []
        self.time: list[float] = []
        self.max_length: Optional[int] = max_length
        self.is_angle: bool = is_angle
        self.rad2deg: bool = rad2deg
        self.sigma_bounds: Optional[list[int]] = sigma_bounds
        self.sigma_data: dict[int, dict[str, list[float]]] = {}
        self.current_sigma: float = 0.0  # Helps with 2D plot sigma bounds
        self.has_sigma: bool = self.sigma_bounds is not None
        if self.has_sigma and self.sigma_bounds is not None:
            for bound in self.sigma_bounds:
                self.sigma_data[bound] = {"lower": [], "upper": []}

    def add_data(self, data: float, t: float, sigma: float = 0) -> None:
        """Adds data to the storage"""
        if self.is_angle:
            data = cast(float, angle_wrap(data))
        if self.rad2deg:
            data, sigma = np.degrees([data, sigma])
        self.data.append(data)
        self.time.append(t)
        if self.has_sigma and self.sigma_bounds is not None:
            for bound in self.sigma_bounds:
                self.sigma_data[bound]["lower"].append(data - bound * sigma)
                self.sigma_data[bound]["upper"].append(data + bound * sigma)
            self.current_sigma = sigma
        if self.max_length is not None and len(self.data) > self.max_length:
            self.pop(0)

    def set_data(
        self, data: list[float], t: list[float], sigma: Optional[list[float]] = None
    ) -> None:
        """Replaces the current data with the input data"""
        # Make sure the lists are the same size
        if len(data) != len(t):
            raise ValueError(
                "Length of data ({}) does not match length of t ({}).".format(
                    len(data), len(t)
                )
            )
        if sigma is not None and len(sigma) != len(t):
            raise ValueError(
                "Length of sigma ({}) does not match length of t ({}).".format(
                    len(sigma), len(t)
                )
            )
        # Populate empty sigma if not given
        if sigma is None:
            sigma = cast(list[float], np.zeros_like(data))
        if self.is_angle:
            data = cast(list[float], angle_wrap(data))
        if self.rad2deg:
            data = cast(list[float], np.degrees(data))
            sigma = cast(list[float], np.degrees(sigma))

        # Ensure that the data and time were not converted to floats
        if not isinstance(data, list):
            data = [data]  # type: ignore
        if not isinstance(t, list):
            t = [t]  # type: ignore
        self.data = data
        self.time = t
        if self.has_sigma and self.sigma_bounds is not None:
            raise NotImplementedError("This code needs to be fixed")
            # TODO Fix this code - does not pass mypy
            # for bound in self.sigma_bounds:
            #     self.sigma_data[bound]['lower'] = list(data - bound*sigma)
            #     self.sigma_data[bound]['upper'] = list(data + bound*sigma)
            # self.current_sigma = sigma[-1]

    def reset_data(self) -> None:
        """Sets data back to an empy list"""
        self.data = []
        self.time = []

        if self.has_sigma and self.sigma_bounds is not None:
            raise NotImplementedError("This code needs to be fixed")

    def get_data_vec(self) -> list[float]:
        """Returns the stored data"""
        return self.data

    def get_time_vec(self) -> list[float]:
        """Returns the time vector associated with the data"""
        return self.time

    def get_sigma_data(self) -> dict[Any, Any]:
        """
        Returns the sigma for each data point
        """
        return self.sigma_data

    def get_current_sigma(self) -> float:
        """Returns the current sigma value"""
        return self.current_sigma

    def pop(self, idx: int = -1) -> None:
        """Removes the data at index idx"""
        raise NotImplementedError("This code needs to be fixed")
        # TODO Fix this code - does not pass mypy
        # self.data.pop(idx)
        # self.time.pop(idx)
        # for data in self.sigma_data.values():
        #     data.pop(idx)


def angle_wrap(
    x: Union[float, list[float], npt.NDArray[Any]]
) -> Union[float, npt.NDArray[Any]]:
    """Converts angles to be within -pi and pi"""
    xwrap = np.array(np.mod(x, 2 * np.pi))
    mask = np.abs(xwrap) > np.pi
    xwrap[mask] -= 2 * np.pi * np.sign(xwrap[mask])
    if np.size(xwrap) == 1:
        return float(xwrap)
    return xwrap

"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - BGM
        12/21 - GND
"""

from typing import Any, Optional, cast

import numpy as np
import numpy.typing as npt
import pyqtgraph.opengl as gl
from mav_sim.message_types.msg_path import MsgPath


class DrawPath:
    """Drawing tool to create path"""

    def __init__(
        self, path: MsgPath, color: npt.NDArray[Any], window: gl.GLViewWidget
    ) -> None:
        """
        Initialize path

        Args:
            path: path to draw
            color: color of path
            window: window in which to plot the path
        """
        self.color = color
        if path.type == "line":
            scale = 1000
            points = straight_line_points(path, scale)
        elif path.type == "orbit":
            points = orbit_points(path)
        path_color = np.tile(color, (points.shape[0], 1))
        self.path_plot_object = gl.GLLinePlotItem(
            pos=points, color=path_color, width=1, antialias=True, mode="line_strip"
        )
        window.addItem(self.path_plot_object)

    def update(self, path: MsgPath, color: Optional[npt.NDArray[Any]] = None) -> None:
        """Update the path being drawn

        Args:
            path: path to draw
            color: color of path
        """
        if color is None:
            color = self.color
        if path.type == "line":
            scale = 1000
            points = straight_line_points(path, scale)
        elif path.type == "orbit":
            points = orbit_points(path)
        path_color = np.tile(color, (points.shape[0], 1))
        self.path_plot_object.setData(pos=points, color=path_color)


def straight_line_points(path: MsgPath, scale: float) -> npt.NDArray[Any]:
    """Get points for drawing a straight line

    Args:
        path: information for line to draw
        scale: size of line to be drawn

    Returns:
        points: The points along the straight line
    """
    points = np.array(
        [
            [
                path.line_origin.item(0),
                path.line_origin.item(1),
                path.line_origin.item(2),
            ],
            [
                path.line_origin.item(0) + scale * path.line_direction.item(0),
                path.line_origin.item(1) + scale * path.line_direction.item(1),
                path.line_origin.item(2) + scale * path.line_direction.item(2),
            ],
        ]
    )
    # convert North-East Down to East-North-Up for rendering
    R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    points = points @ R.T
    return cast(npt.NDArray[Any], points)


def orbit_points(path: MsgPath) -> npt.NDArray[Any]:
    """Get points for drawing a circular orbit

    Args:
        path: information for drawing the orbit

    Returns:
        points: Points that make up the circle
    """
    theta = 0.0
    theta_list: list[float] = [theta]
    while theta < 2 * np.pi:
        theta += 0.1
        theta_list.append(theta)
    points = np.array(
        [
            [
                path.orbit_center.item(0) + path.orbit_radius,
                path.orbit_center.item(1),
                path.orbit_center.item(2),
            ]
        ]
    )
    for angle in theta_list:
        new_point = np.array(
            [
                [
                    path.orbit_center.item(0) + path.orbit_radius * np.cos(angle),
                    path.orbit_center.item(1) + path.orbit_radius * np.sin(angle),
                    path.orbit_center.item(2),
                ]
            ]
        )
        points = np.concatenate((points, new_point), axis=0)
    # convert North-East Down to East-North-Up for rendering
    R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    points = points @ R.T
    return cast(npt.NDArray[Any], points)

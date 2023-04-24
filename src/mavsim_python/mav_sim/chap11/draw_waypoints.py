"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - BGM
        12/21 - GND
"""

from typing import cast

import numpy as np
import pyqtgraph.opengl as gl
from mav_sim.chap11.dubins_parameters import DubinsParameters
from mav_sim.message_types.msg_waypoints import MsgWaypoints
from mav_sim.tools.types import NP_MAT


class DrawWaypoints:
    """Drawing tool to illustrate waypoints"""

    def __init__(
        self,
        waypoints: MsgWaypoints,
        radius: float,
        color: NP_MAT,
        window: gl.GLViewWidget,
    ) -> None:
        """Initialize plotter

        Args:
            waypoints: waypoints to be followed
            radius: minimum radius circle for the mav
            color: color of path
            window: window in which to plot the path
        """
        self.radius = radius
        self.color = color
        if waypoints.type in ("straight_line", "fillet"):
            points = straight_waypoint_points(waypoints)
        elif waypoints.type == "dubins":
            points = dubins_points(waypoints, self.radius, 0.1)
        waypoint_color = np.tile(color, (points.shape[0], 1))
        self.waypoint_plot_object = gl.GLLinePlotItem(
            pos=points, color=waypoint_color, width=2, antialias=True, mode="line_strip"
        )
        window.addItem(self.waypoint_plot_object)

    def update(self, waypoints: MsgWaypoints) -> None:
        """Update the plot for new waypoints

        Args:
            waypoints: waypoints to be followed
        """
        if waypoints.type in ("straight_line", "fillet"):
            points = straight_waypoint_points(waypoints)
        elif waypoints.type == "dubins":
            points = dubins_points(waypoints, self.radius, 0.1)
        self.waypoint_plot_object.setData(pos=points)


def dubins_points(waypoints: MsgWaypoints, radius: float, Del: float) -> NP_MAT:
    """
    Returns a list of points along the dubins path

    Args:
        waypoints: waypoints to be followed
        radius: minimum radius circle for the mav
        Del: TODO document
    """
    initialize_points = True
    for j in range(0, waypoints.num_waypoints - 1):
        dubins_path = DubinsParameters(
            waypoints.ned[:, j : j + 1],
            waypoints.course.item(j),
            waypoints.ned[:, j + 1 : j + 2],
            waypoints.course.item(j + 1),
            radius,
        )

        # points along start circle
        th1 = np.arctan2(
            dubins_path.p_s.item(1) - dubins_path.center_s.item(1),
            dubins_path.p_s.item(0) - dubins_path.center_s.item(0),
        )
        th1 = mod(th1)
        th2 = np.arctan2(
            dubins_path.r1.item(1) - dubins_path.center_s.item(1),
            dubins_path.r1.item(0) - dubins_path.center_s.item(0),
        )
        th2 = mod(th2)
        th = th1
        theta_list = [th]
        if dubins_path.dir_s > 0:
            if th1 >= th2:
                while th < th2 + 2 * np.pi:
                    th += Del
                    theta_list.append(th)
            else:
                while th < th2:
                    th += Del
                    theta_list.append(th)
        else:
            if th1 <= th2:
                while th > th2 - 2 * np.pi:
                    th -= Del
                    theta_list.append(th)
            else:
                while th > th2:
                    th -= Del
                    theta_list.append(th)

        if initialize_points:
            points = np.array(
                [
                    [
                        dubins_path.center_s.item(0)
                        + dubins_path.radius * np.cos(theta_list[0]),
                        dubins_path.center_s.item(1)
                        + dubins_path.radius * np.sin(theta_list[0]),
                        dubins_path.center_s.item(2),
                    ]
                ]
            )
            initialize_points = False
        for angle in theta_list:
            new_point = np.array(
                [
                    [
                        dubins_path.center_s.item(0)
                        + dubins_path.radius * np.cos(angle),
                        dubins_path.center_s.item(1)
                        + dubins_path.radius * np.sin(angle),
                        dubins_path.center_s.item(2),
                    ]
                ]
            )
            points = np.concatenate((points, new_point), axis=0)

        # points along straight line
        sig = 0.0
        while sig <= 1:
            new_point = np.array(
                [
                    [
                        (1 - sig) * dubins_path.r1.item(0)
                        + sig * dubins_path.r2.item(0),
                        (1 - sig) * dubins_path.r1.item(1)
                        + sig * dubins_path.r2.item(1),
                        (1 - sig) * dubins_path.r1.item(2)
                        + sig * dubins_path.r2.item(2),
                    ]
                ]
            )
            points = np.concatenate((points, new_point), axis=0)
            sig += Del

        # points along end circle
        th2 = np.arctan2(
            dubins_path.p_e.item(1) - dubins_path.center_e.item(1),
            dubins_path.p_e.item(0) - dubins_path.center_e.item(0),
        )
        th2 = mod(th2)
        th1 = np.arctan2(
            dubins_path.r2.item(1) - dubins_path.center_e.item(1),
            dubins_path.r2.item(0) - dubins_path.center_e.item(0),
        )
        th1 = mod(th1)
        th = th1
        theta_list = [th]
        if dubins_path.dir_e > 0:
            if th1 >= th2:
                while th < th2 + 2 * np.pi:
                    th += Del
                    theta_list.append(th)
            else:
                while th < th2:
                    th += Del
                    theta_list.append(th)
        else:
            if th1 <= th2:
                while th > th2 - 2 * np.pi:
                    th -= Del
                    theta_list.append(th)
            else:
                while th > th2:
                    th -= Del
                    theta_list.append(th)
        for angle in theta_list:
            new_point = np.array(
                [
                    [
                        dubins_path.center_e.item(0)
                        + dubins_path.radius * np.cos(angle),
                        dubins_path.center_e.item(1)
                        + dubins_path.radius * np.sin(angle),
                        dubins_path.center_e.item(2),
                    ]
                ]
            )
            points = np.concatenate((points, new_point), axis=0)

    R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    points = points @ R.T
    return points


def mod(x: float) -> float:
    """
    force x to be between 0 and 2*pi

    Args:
        x: angle to be updated

    Returns:
        x: angle shifted to be within 0 and 2 pi
    """
    while x < 0:
        x += 2 * np.pi
    while x > 2 * np.pi:
        x -= 2 * np.pi
    return x


def straight_waypoint_points(waypoints: MsgWaypoints) -> NP_MAT:
    """Return waypoints as a straight line for plotting"""
    R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    points = R @ np.copy(waypoints.ned)
    return cast(NP_MAT, points.T)

"""
rrt straight line path planner for mavsim_python

mavsim_python
    - Beard & McLain, PUP, 2012
    - Last updated:
        4/3/2019 - Brady Moon
        4/11/2019 - RWB
        3/31/2020 - RWB
        4/2022 - GND
"""
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from mav_sim.chap11.draw_waypoints import DrawWaypoints
from mav_sim.chap12.draw_map import DrawMap
from mav_sim.chap12.planner_utilities import (
    column,
    exist_feasible_path,
    find_closest_configuration,
    find_shortest_path,
    generate_random_configuration,
    plan_path,
    smooth_path,
)
from mav_sim.message_types.msg_waypoints import MsgWaypoints, Waypoint
from mav_sim.message_types.msg_world_map import MsgWorldMap
from mav_sim.tools.types import NP_MAT


class RRTStraightLine:
    """RRT planner for straight line plans"""

    def __init__(self) -> None:
        """Initialize parameters"""
        self.segment_length = 300  # standard length of path segments
        self.plot_window: gl.GLViewWidget
        self.plot_app: pg.QtGui.QApplication

    def plot_map(
        self,
        world_map: MsgWorldMap,
        tree: MsgWaypoints,
        waypoints: MsgWaypoints,
        smoothed_waypoints: MsgWaypoints,
        radius: float,
    ) -> None:
        """Plots the RRT tree

        Args:
            world_map: definition of the world for planning
            tree: Current set of waypoints in rrt search tree
            waypoints: Non-smoothed, minimum length path
            smoothed_waypoints: The path (waypoints) after smoothing
            radius: minimum radius circle for the mav
        """
        scale = 4000
        # initialize Qt gui application and window
        self.plot_app = pg.QtGui.QApplication([])  # initialize QT
        self.plot_window = gl.GLViewWidget()  # initialize the view object
        self.plot_window.setWindowTitle("World Viewer")
        self.plot_window.setGeometry(
            0, 0, 1500, 1500
        )  # args: upper_left_x, upper_right_y, width, height
        grid = gl.GLGridItem()  # make a grid to represent the ground
        grid.scale(
            scale / 20, scale / 20, scale / 20
        )  # set the size of the grid (distance between each line)
        self.plot_window.addItem(grid)  # add grid to viewer
        self.plot_window.setCameraPosition(distance=scale, elevation=50, azimuth=-90)
        self.plot_window.setBackgroundColor("k")  # set background color to black
        self.plot_window.show()  # display configured window
        # self.plot_window.raise_() # bring window to the front

        blue = np.array([[30, 144, 255, 255]]) / 255.0
        red = np.array([[204, 0, 0]]) / 255.0
        green = np.array([[0, 153, 51]]) / 255.0
        DrawMap(world_map, self.plot_window)
        DrawWaypoints(waypoints, radius, blue, self.plot_window)
        DrawWaypoints(smoothed_waypoints, radius, red, self.plot_window)
        draw_tree(tree, green, self.plot_window)
        # draw things to the screen
        self.plot_app.processEvents()

    def update(
        self,
        start_pose: NP_MAT,
        end_pose: NP_MAT,
        Va: float,
        world_map: MsgWorldMap,
        num_paths: int = 5,
    ) -> MsgWaypoints:
        """Creates a plan from the start pose to the end pose.

        Args:
            start_pose: starting pose of the mav
            end_pose: desired end pose of the mav
            Va: airspeed
            world_map: definition of the world for planning
            num_paths: Number of paths to find before selecting the best path

        Returns:
            waypoints: Waypoints defining the planned path
        """
        return create_rrt_plan(
            start_pose=start_pose,
            end_pose=end_pose,
            Va=Va,
            world_map=world_map,
            segment_length=self.segment_length,
            num_paths=num_paths,
        )


def create_rrt_plan(
    start_pose: NP_MAT,
    end_pose: NP_MAT,
    Va: float,
    world_map: MsgWorldMap,
    segment_length: float,
    num_paths: int = 5,
) -> MsgWaypoints:
    """Update the plan using fillets for basic motion primitives

    Implements Algorithm 12 with a small modification. Instead of stopping
    once the first path is found to the goal, it stops once `num_paths` different
    paths have been found and then selects the shortest path found for the return.

    Args:
        start_pose: starting pose of the mav
        end_pose: desired end pose of the mav
        Va: airspeed
        world_map: definition of the world for planning
        segment_length: standard length of path segments - maximum segment length
        num_paths: Number of paths to find before selecting the best path

    Returns:
        waypoints: Waypoints defining the planned path
    """
    # Initialize the tree (Algorithm 12 line 1)
    tree = MsgWaypoints()
    tree.type = "fillet"  # Could also be: tree.type = 'straight_line'
    tree.add(ned=start_pose, airspeed=Va)  # add the start pose to the tree
    num_paths_found = 0

    while num_paths_found < num_paths:
        point = generate_random_configuration(world_map, start_pose.item(2))
        clos_ned, idx, _ = find_closest_configuration(tree, point)
        path_ned, path_dist = plan_path(clos_ned, point, segment_length)
        if exist_feasible_path(clos_ned, path_ned, world_map):
            new_node = Waypoint()
            new_node.airspeed = Va
            new_node.course = np.arctan2(
                path_ned.item(1) - clos_ned.item(1), path_ned.item(0) - clos_ned.item(0)
            )
            new_node.cost = path_dist + tree.get_waypoint(idx).cost
            new_node.parent = idx
            new_node.ned = path_ned
            if exist_feasible_path(path_ned, end_pose, world_map):
                new_node.connect_to_goal = 1
                num_paths_found += 1
            else:
                new_node.connect_to_goal = 0
            tree.add_waypoint(new_node)
    waypoints = find_shortest_path(tree, end_pose)
    waypoints = smooth_path(waypoints, world_map)

    return waypoints


def draw_tree(tree: MsgWaypoints, color: NP_MAT, window: gl.GLViewWidget) -> None:
    """Draw the tree in the given window

    Args:
        tree: Current set of waypoints in rrt search tree
        color: color of tree
        window: window in which to plot the tree
    """
    R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    points = R @ tree.ned
    for i in range(points.shape[1]):
        line_color = np.tile(color, (2, 1))
        parent = int(tree.parent.item(i))
        line_pts = np.concatenate(
            (column(points, i).T, column(points, parent).T), axis=0
        )
        line = gl.GLLinePlotItem(
            pos=line_pts, color=line_color, width=2, antialias=True, mode="line_strip"
        )
        window.addItem(line)

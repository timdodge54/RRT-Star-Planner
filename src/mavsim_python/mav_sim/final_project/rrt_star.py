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
import typing

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from mav_sim.chap11.draw_waypoints import DrawWaypoints
from mav_sim.chap12 import *
from mav_sim.chap12.draw_map import DrawMap
from mav_sim.chap12.planner_utilities import (
    column,
    distance,
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


class RRTStarStraightLine:
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
        iteration_count: int = 1000,
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
        return create_rrt_star_plan(
            start_pose=start_pose,
            end_pose=end_pose,
            Va=Va,
            world_map=world_map,
            segment_length=self.segment_length,
            max_iter=iteration_count,
        )


def create_rrt_star_plan(
    start_pose: NP_MAT,
    end_pose: NP_MAT,
    Va: float,
    world_map: MsgWorldMap,
    segment_length: float,
    max_iter: int,
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
    
    count = 0
    min_cost = np.inf
    min_cost_idx = None

    while count < max_iter:
        count += 1
        point = generate_random_configuration(world_map, start_pose.item(2))
        parent_ned, rewire_array, parent_idx = extend(point, tree, world_map)
        path_ned = point
        path_dist = distance(parent_ned, point)

        # path_ned, path_dist = plan_path(parent_ned, point, segment_length)
        if exist_feasible_path(parent_ned, path_ned, world_map):
            new_node = Waypoint()
            new_node.airspeed = Va
            new_node.course = np.arctan2(
                path_ned.item(1) - parent_ned.item(1),
                path_ned.item(0) - parent_ned.item(0),
            )
            new_node.cost = path_dist + tree.get_waypoint(parent_idx).cost
            new_node.parent = parent_idx
            new_node.ned = path_ned
            if exist_feasible_path(path_ned, end_pose, world_map):
                cost_sol = new_node.cost + float(np.linalg.norm(path_ned - end_pose))
                if cost_sol < min_cost:
                    min_cost = cost_sol
                    min_cost_idx = tree.num_waypoints 
                new_node.connect_to_goal = 1
            else:
                new_node.connect_to_goal = 0
            tree.add_waypoint(new_node)
            # if check_for_cycle(0, 1, tree):
            #     print("Cycle detected in tree main")
            #     raise Exception("Cycle detected in tree main")

            if len(rewire_array) > 0:
                tree = rewire(new_node, rewire_array, tree, world_map)
    waypoints = find_shortest_path_star(tree, end_pose, min_cost_idx, start_pose)

    return waypoints


def rewire(
    rewire_node: Waypoint,
    surround_nodes: typing.List[int],
    tree: MsgWaypoints,
    world_map: MsgWorldMap,
) -> MsgWaypoints:
    """Rewire tree for optimal cost.

    Args:
        rewire_node: Node to rewire
        surround_nodes: List of node indices to rewire
        tree: Tree to rewire
    Returns:
        tree: Rewired tree

    """
    idx_of_rewire_node = tree.num_waypoints - 1
    rewire_ned = rewire_node.ned
    for idx in surround_nodes:
        node = tree.get_waypoint(idx)
        node_parent = tree.get_waypoint(node.parent)
        cost_from_parent = node.cost - node_parent.cost
        dist_to_rewire = distance(rewire_node.ned, node.ned)
        cost_from_rewire = dist_to_rewire + rewire_node.cost
        if cost_from_rewire < node.cost:
            if exist_feasible_path(node.ned, rewire_node.ned, world_map):
                node_ned = node.ned
                course = np.arctan2(
                    node_ned.item(1) - rewire_ned.item(1),
                    node_ned.item(0) - rewire_ned.item(0),
                )
                tree.replace_idx(
                    idx=idx,
                    rewire_idx=idx_of_rewire_node,
                    prev_parent_idx=node.parent,
                    cost_add=dist_to_rewire + rewire_node.cost,
                    cost_sub=node.cost,
                    course=course,
                )
    return tree

def check_for_cycle(rewire_idx, child_idx, tree):
    """
    Check if adding rewire_node as the parent of parent_idx would create a cycle in the tree.
    """
    past_inds = set()
    return not better_check_for_cycle(0, past_inds, tree)
    truth = False
    child_idx = int(child_idx)
    if not len(tree.children[child_idx]) == 0:
        print(f"Checking rewire_idx: {rewire_idx}")
        for child in tree.children[child_idx]:
            print(f"Checking child: {child}")
            if child == rewire_idx:
                print(f"Cycle detected: {rewire_idx}")
            else:
                truth = truth or check_for_cycle(rewire_idx, child, tree)
    return truth


def better_check_for_cycle(cur_ind, past_inds, tree):
    if len(tree.children[cur_ind]) == 0 or tree.children[cur_ind] is None:
        return True
    if cur_ind in past_inds:
        print(cur_ind)
        print(past_inds)
        raise Exception("Cycle detected")
        return False
    past_inds.add(cur_ind)
    ret = True
    for child in tree.children[cur_ind]:
        ret = ret and better_check_for_cycle(child, past_inds, tree)
        if not ret:
            print(cur_ind)
            print(past_inds)
            raise Exception("Cycle detected")
    return ret 


def extend(
    current_node: NP_MAT, tree: MsgWaypoints, world_map: MsgWorldMap
) -> tuple[NP_MAT, list[int], int]:
    """Set node parent to node that is the lowest cost path to the current node.

    Args:
        current_node: Node to extend
        tree: Tree to extend
        world_map: World map to use for planning
    returns:
        optimal_parent: Optimal parent nodes ned position
        surrounding_nodes: List of surrounding nodes indices
        optimal_index: Index of the node with the lowest cost
    """
    radius = world_map.city_width / 10
    current_pos = current_node
    index_of_nodes_within_radius = []
    if check_for_cycle(0, 1, tree):
        print("Cycle detected ext")
        raise Exception("Cycle detected ext")
    for i in range(tree.num_waypoints - 1):
        node_ned = tree.get_waypoint(i).ned
        if distance(current_pos, node_ned) < radius:
            index_of_nodes_within_radius.append(i)
    if len(index_of_nodes_within_radius) != 0:
        min_cost = np.inf
        min_cost_index = None
        for i, cost in enumerate(tree.cost):
            if i in index_of_nodes_within_radius:
                modified_cost = cost + distance(current_pos, tree.get_waypoint(i).ned)
                if modified_cost < min_cost:
                    min_cost = modified_cost
                    min_cost_index = i
        array_to_rewire = []
        for i in index_of_nodes_within_radius:
            if i != min_cost_index:
                array_to_rewire.append(i)
        return tree.get_waypoint(min_cost_index).ned, array_to_rewire, min_cost_index
    else:
        clos_ned, idx, _ = find_closest_configuration(tree, current_pos)
        return clos_ned, [], idx


def draw_tree(tree: MsgWaypoints, color: NP_MAT, window: gl.GLViewWidget) -> None:
    """Draw the tree in the given window.

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


def find_shortest_path_star(
    tree: MsgWaypoints, end_pose: NP_MAT, min_idx: int, start_pose: NP_MAT
) -> MsgWaypoints:
    """Find the lowest cost path to the end node.

    findShortestPath(...) from Algorithm 12

    Args:
        tree: Current set of waypoints in rrt search tree. Note that
              tree.connect_to_goal.item(i) == 1 iff the node is connected to
              the goal
        end_pose: Desired end pose of the path

    Returns:
        waypoints: The shortest path
    """
    shortest_path = MsgWaypoints()
    list_of_ned = []
    idx_ = min_idx
    while True:
        node = tree.get_waypoint(idx_)
        idx_ = int(node.parent)
        list_of_ned.append(node.ned)
        if compare_ned(node.ned, start_pose):
            break

    for ned in reversed(list_of_ned):
        waypoint = Waypoint()
        waypoint.ned = ned
        waypoint.parent = 0
        waypoint.cost = 0.0
        waypoint.connect_to_goal = 0
        waypoint.airspeed = 25.0
        waypoint.course = np.inf
        shortest_path.add_waypoint(waypoint)
    waypoint = Waypoint()
    waypoint.ned = end_pose
    waypoint.parent = 0
    waypoint.cost = 0.0
    waypoint.connect_to_goal = 0
    waypoint.airspeed = 25.0
    waypoint.course = np.inf
    shortest_path.add_waypoint(waypoint)
    return shortest_path


def compare_ned(first: NP_MAT, second: NP_MAT) -> bool:
    """Compare two ned positions.

    Args:
        first: First ned position
        second: Second ned position

    Returns:
        True if the two ned positions are the same, False otherwise
    """
    if np.allclose(first, second):
        return True
    else:
        return False

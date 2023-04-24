"""
Utilities that can be used throughout a variety of path planners

mavsim_python
    - Beard & McLain, PUP, 2012
    - Last updated:
        4/3/2019 - Brady Moon
        4/11/2019 - RWB
        3/31/2020 - RWB
        4/2022 - GND
"""
from typing import Optional

import numpy as np
from mav_sim.message_types.msg_waypoints import MsgWaypoints, Waypoint
from mav_sim.message_types.msg_world_map import MsgWorldMap, map_height
from mav_sim.tools.types import NP_MAT


def smooth_path(waypoints: MsgWaypoints, world_map: MsgWorldMap) -> MsgWaypoints:
    """smooth the waypoint path - Implementation of Algorithm 13

    Args:
        waypoints: The path to be smoothed
        world_map: definition of the world for planning
    """
    # Return smoothed path (Fix. This currently does nothing)
    smoothed_path = MsgWaypoints()
    smoothed_path.type = waypoints.type
    node = waypoints.get_waypoint(0)
    smoothed_path.add_waypoint(node)
    i = 0
    j = 1
    while j < waypoints.num_waypoints - 1:
        w_s = waypoints.get_waypoint(i)
        w_plus = waypoints.get_waypoint(j + 1)
        if not exist_feasible_path(w_s.ned, w_plus.ned, world_map):
            last_node = waypoints.get_waypoint(j)
            last_node.cost = distance(w_s.ned, last_node.ned) + w_s.cost
            smoothed_path.add_waypoint(last_node)
            i = j
        j += 1
    prev_node = smoothed_path.get_waypoint(smoothed_path.num_waypoints - 1)
    last_node = waypoints.get_waypoint(waypoints.num_waypoints - 1)
    last_node.cost = distance(prev_node.ned, last_node.ned) + prev_node.cost
    smoothed_path.add_waypoint(last_node)
    return smoothed_path


def find_shortest_path(tree: MsgWaypoints, end_pose: NP_MAT) -> MsgWaypoints:
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
    list_of_ned = []
    shortest_path = MsgWaypoints()
    shortest_path.type = tree.type
    all_connect = tree.connect_to_goal == 1
    cost_true = tree.cost[all_connect]
    low_valueidx = np.argmin(tree.cost[all_connect])
    low_value = cost_true[low_valueidx]
    idicies_of_min: NP_MAT = np.logical_and(
        tree.cost == low_value, tree.connect_to_goal == 1
    )
    idx = np.where(idicies_of_min)
    ix = np.array(idx)
    idx_ = int(ix.item(0))
    while True:
        node = tree.get_waypoint(idx_)
        idx_ = int(node.parent)
        list_of_ned.append(node.ned)
        if node.parent == 0:
            node = tree.get_waypoint(0)
            list_of_ned.append(node.ned)
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


def generate_random_configuration(world_map: MsgWorldMap, pd: float) -> NP_MAT:
    """Generates a random pose in the world.

    :The generated pose is generated randomly in the 2D plane with the
    down element (altitude) fixed. Note that the city is assumed square
    with world_map.city_width providing the lenght of one side of the square.

    generateRandomConfiguration() routine in Algorithm 12

    Args:
        world_map: definition of the world for planning
        pd: The down position (i.e., altitude) to use for the search

    Returns:
        pose: 3x1 vector with (pn, pe) defined using a random distribution over the width
              of map.
    """
    pn = np.random.rand() * world_map.city_width
    pe = np.random.rand() * world_map.city_width
    pose = np.array([[pn], [pe], [pd]])
    return pose


def find_closest_configuration(
    tree: MsgWaypoints, pos_in: NP_MAT
) -> tuple[NP_MAT, int, float]:
    """Returns the closest waypoint in tree to the passed in 3x1 position

        findClosestConfiguration() routine used in Algorithm 12

    Args:
        tree: Current set of waypoints in rrt search tree
        pos_in: The position to be used for comparison

    Returns:
        pos_closest: The 3x1 position that is closest to pos_in
        idx: The index of pos_closest in tree.ned
        dist: The distance from tree.get_ned(idx)
    """

    # create the outputs
    diff: NP_MAT = tree.ned - pos_in
    dist = np.linalg.norm(diff, axis=0)
    index = int(np.argmin(dist))
    closet_point = tree.get_ned(index)
    return closet_point, index, dist[index]


def plan_path(
    start_point: NP_MAT,
    desired_point: NP_MAT,
    max_edge_length: float,
    dist: Optional[float] = None,
) -> tuple[NP_MAT, float]:
    """Returns a point along the line formed between the two input points that is
        at most the max edge length away from the starting point

        planPath() routine from Algorithm 12

    Args:
        start_point: Starting point of the new line
        desired_point: The point that is in the direction of the tree extension
        max_edge_length: The maximum edge length allowed
        dist: The distance between start and desired points.
              If None is passed in then the distance is calculated.
              Note that dist must be positive

    Returns:
        new_point: The along the line between start and desired points
        dist_to_new: The distance to the new point
    """
    print(f"dist {dist}")
    distance_ = np.linalg.norm(desired_point - start_point)
    diff: NP_MAT = desired_point - start_point
    new_point = desired_point
    dist_to_new = float(distance_)
    if distance_ > max_edge_length:
        new_point = start_point + max_edge_length * diff / distance_
        dist_to_new = max_edge_length
    return new_point, dist_to_new


def exist_feasible_path(
    start_pose: NP_MAT, end_pose: NP_MAT, world_map: MsgWorldMap
) -> bool:
    """check to see of path from start_pose to end_pose colliding with map

    existFeasiblePath() routine from Algorithm 12

    Args:
        start_pose: starting point on a line
        end_pose: ending point on a line
        world_map: definition of the world for planning

    Returns:
        True => path is feasible, False => path collides with obstacle
    """
    points = points_along_path(start_pose, end_pose, 100)
    for i in range(points.shape[1]):
        if height_above_ground(world_map, column(points, i)) <= 0:
            # No need to search through remaining points as a collision is found
            return False
    return True


def distance(start_pose: NP_MAT, end_pose: NP_MAT) -> float:
    """compute distance between start and end pose

    Args:
        start_pose: pose one
        end_pose: pose two

    Returns:
        d: distance between start and end poses
    """
    d = np.linalg.norm(start_pose - end_pose)
    return float(d)


def height_above_ground(world_map: MsgWorldMap, point: NP_MAT) -> float:
    """find the altitude of point above ground level

    Args:
        world_map: definition of the world for planning
        point: location in ned to calculate height

    Returns:
        h_agl: Height at the position (A negative value implies a collision)
    """
    point_height = -point.item(2)
    map_height_val = map_height(world_map, point)
    h_agl = point_height - map_height_val
    return float(h_agl)


def points_along_path(start_pose: NP_MAT, end_pose: NP_MAT, N: int) -> NP_MAT:
    """Returns N points along path defined by the starting and ending poses

    Args:
        start_pose: starting point on a line
        end_pose: ending point on a line
        N: Number of points to return

    Returns:
        points: Points along line between start and end pose
    """
    points = start_pose
    q: NP_MAT = end_pose - start_pose
    L = np.linalg.norm(q)
    q = q / L
    w = start_pose
    for _ in range(1, N):
        w = w + (L / N) * q
        points = np.append(points, w, axis=1)
    return points


def column(A: NP_MAT, i: int) -> NP_MAT:
    """Extracts the ith column of A and return column vector"""
    tmp = A[:, i]
    col = tmp.reshape(A.shape[0], 1)
    return col

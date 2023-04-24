"""
msg_map
    - messages type for map of the world

part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        4/10/2019 - RWB
"""
from typing import Optional

import mav_sim.parameters.planner_parameters as PLAN
import numpy as np
from mav_sim.tools.types import NP_MAT


class MsgWorldMap:
    """Defines the world map"""

    def __init__(
        self,
        city_width: float = PLAN.city_width,
        num_city_blocks: int = PLAN.num_blocks,
        building_max_height: float = PLAN.building_height,
        street_width: float = PLAN.street_width,
        building_heights: Optional[NP_MAT] = None,
    ) -> None:
        """Initializes the map to default parameters with random initialization for building heights

        Args:
            city_width: The width of the city (note, the city is square)
            num_city_blocks: The number of city blocks in one direction
            building_max_height: The maximum height of the buildings
            street_width: The width of the streets
            building_heights: Heights of each building. If no heights are provided then they will be randomly generated
        """
        ### Independent Parameters for generating city ###
        # flag to indicate if the map has changed
        self.flag_map_changed: bool = False
        # the city is of size (width)x(width)
        self.city_width: float = city_width
        # number of blocks in city
        self.num_city_blocks: int = num_city_blocks
        # actual width of the street
        self.street_width_val = street_width
        # percent of block that is street.
        self.street_width: float = city_width / num_city_blocks * street_width
        # maximum height of buildings
        self.building_max_height: float = building_max_height

        ### Dependent city parameters ###
        # the width of the buildings (all the same)
        self.building_width: float = city_width / num_city_blocks * (1 - street_width)
        # north coordinate of center of buildings
        self.building_north: NP_MAT = np.zeros((1, num_city_blocks))
        for i in range(num_city_blocks):
            self.building_north[0, i] = (
                0.5 * (city_width / num_city_blocks) * (2 * i + 1)
            )
        # east coordinate of center of buildings
        self.building_east = np.copy(self.building_north)

        ### Randomly generated variables ###
        # an array of building heights
        self.building_height: NP_MAT
        self.set_heights(building_heights=building_heights)

    def set_heights(self, building_heights: Optional[NP_MAT] = None) -> None:
        """Sets the building heights. If no heights are provided then they are produced randomly.

        Args:
            building_heights: Heights of each building. If no heights are provided then they will be randomly generated
        """
        # Set the building heights
        if building_heights is None:
            self.building_height = self.building_max_height * np.random.rand(
                self.num_city_blocks, self.num_city_blocks
            )
        else:
            self.building_height = building_heights

        # Check the size of the building heights
        if (
            np.size(self.building_height, 0) != self.num_city_blocks
            or np.size(self.building_height, 1) != self.num_city_blocks
        ):
            raise ValueError(
                "The buildings heights provided do not have the correct dimensions"
            )


def map_height(world_map: MsgWorldMap, point: NP_MAT) -> float:
    """find the height of point at ground level

    Args:
        world_map: definition of the world for planning
        point: location in ned to calculate height of the ground (in enu frame)

    Returns:
        map_height: Height of ground at (north,east) position (i.e., the up value in the enu frame)
    """
    tmp = np.abs(point.item(0) - world_map.building_north)
    d_n = np.min(tmp)
    idx_n = np.argmin(tmp)
    tmp = np.abs(point.item(1) - world_map.building_east)
    d_e = np.min(tmp)
    idx_e = np.argmin(tmp)
    if (d_n < world_map.building_width) and (d_e < world_map.building_width):
        map_height_ = float(world_map.building_height[idx_n, idx_e])
    else:
        map_height_ = 0.0
    return map_height_

"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - BGM
"""
from typing import Any

import numpy as np
import numpy.typing as npt
import pyqtgraph.opengl as gl
from mav_sim.tools import types
from mav_sim.tools.rotations import Euler2Rotation


class DrawMav:
    """Draw the MAV

    Need to update the description
    """

    def __init__(self, state: types.Pose, window: gl.GLViewWidget) -> None:
        """Draw the MAV.

        This function takes the non-translated, non-rotated points defining the MAV, transforms them into place,
        converts them for proper rendering, and then creates a 3D mesh with the points

        Args:
            state: Position and orientation of the vehicle
            window: Viewing window for plotting
        """
        # get points that define the non-rotated, non-translated mav and the mesh colors
        self.mav_points: types.Points
        self.mav_points, self.mav_mesh_colors = get_points()

        # Transform the points to MAV location and orientation
        mav_position = types.Vector(
            np.array([[state.north], [state.east], [-state.altitude]])
        )

        # NED coordinates
        # attitude of mav as a rotation matrix R from body to inertial
        R = Euler2Rotation(state.phi, state.theta, state.psi)
        # rotate and translate points defining mav
        rotated_points = rotate_points(self.mav_points, R)
        translated_points = translate_points(rotated_points, mav_position)

        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        translated_points = types.Points(R @ translated_points)

        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = points_to_mesh(translated_points)
        self.mav_body = gl.GLMeshItem(
            vertexes=mesh,  # defines the triangular mesh (Nx3x3)
            vertexColors=self.mav_mesh_colors,  # defines mesh colors (Nx1)
            drawEdges=True,  # draw edges between mesh elements
            smooth=False,  # speeds up rendering
            computeNormals=False,
        )  # speeds up rendering
        window.addItem(self.mav_body)  # add body to plot

    def update(self, state: types.Pose) -> None:
        """Updates the MAV state to a new state

        Replots the MAV position given the new state

        Args:
            state: Updated position and orientation variables for plotting
        """
        mav_position = types.Vector(
            np.array([[state.north], [state.east], [-state.altitude]])
        )  # NED coordinates
        # attitude of mav as a rotation matrix R from body to inertial
        R = Euler2Rotation(state.phi, state.theta, state.psi)
        # rotate and translate points defining mav
        rotated_points = rotate_points(self.mav_points, R)
        translated_points = translate_points(rotated_points, mav_position)
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        translated_points = types.Points(R @ translated_points)
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = points_to_mesh(translated_points)
        # draw MAV by resetting mesh using rotated and translated points
        self.mav_body.setMeshData(vertexes=mesh, vertexColors=self.mav_mesh_colors)


def rotate_points(points: types.Points, R: types.RotMat) -> types.Points:
    "Rotate points by the rotation matrix R"
    # Check inputs
    types.check_points(points)
    types.check_rotation_matrix(R)

    # Rotation points
    rotated_points = types.Points(R @ points)
    return rotated_points


def translate_points(points: types.Points, translation: types.Vector) -> types.Points:
    "Translate points by the vector translation"
    # Check inputs
    types.check_points(points)
    types.check_vector(translation)

    # Translate points
    translated_points = types.Points(
        points + np.dot(translation, np.ones([1, points.shape[1]]))
    )
    return translated_points


def get_points() -> tuple[types.Points, npt.NDArray[Any]]:
    """ "
    Points that define the mav, and the colors of the triangular mesh
    Define the points on the aircraft following diagram in Figure C.3
    """
    # define MAV body parameters
    unit_length = 0.25
    fuse_h = unit_length
    fuse_w = unit_length
    fuse_l1 = unit_length * 2
    fuse_l2 = unit_length
    fuse_l3 = unit_length * 4
    wing_l = unit_length
    wing_w = unit_length * 6
    tail_h = unit_length
    tail_l = unit_length
    tail_w = unit_length * 2

    # points are in NED coordinates
    #   define the points on the aircraft following diagram Fig 2.14
    points = np.array(
        [
            [fuse_l1, 0, 0],  # point 1 [0]
            [fuse_l2, fuse_w / 2.0, -fuse_h / 2.0],  # point 2 [1]
            [fuse_l2, -fuse_w / 2.0, -fuse_h / 2.0],  # point 3 [2]
            [fuse_l2, -fuse_w / 2.0, fuse_h / 2.0],  # point 4 [3]
            [fuse_l2, fuse_w / 2.0, fuse_h / 2.0],  # point 5 [4]
            [-fuse_l3, 0, 0],  # point 6 [5]
            [0, wing_w / 2.0, 0],  # point 7 [6]
            [-wing_l, wing_w / 2.0, 0],  # point 8 [7]
            [-wing_l, -wing_w / 2.0, 0],  # point 9 [8]
            [0, -wing_w / 2.0, 0],  # point 10 [9]
            [-fuse_l3 + tail_l, tail_w / 2.0, 0],  # point 11 [10]
            [-fuse_l3, tail_w / 2.0, 0],  # point 12 [11]
            [-fuse_l3, -tail_w / 2.0, 0],  # point 13 [12]
            [-fuse_l3 + tail_l, -tail_w / 2.0, 0],  # point 14 [13]
            [-fuse_l3 + tail_l, 0, 0],  # point 15 [14]
            [-fuse_l3, 0, -tail_h],  # point 16 [15]
        ]
    ).T

    # scale points for better rendering
    scale = 20
    points = scale * points

    #   define the colors for each face of triangular mesh
    red = np.array([1.0, 0.0, 0.0, 1])
    green = np.array([0.0, 1.0, 0.0, 1])
    blue = np.array([0.0, 0.0, 1.0, 1])
    yellow = np.array([1.0, 1.0, 0.0, 1])
    mesh_colors = np.empty((13, 3, 4), dtype=np.float32)
    mesh_colors[0] = yellow  # nose-top
    mesh_colors[1] = yellow  # nose-right
    mesh_colors[2] = yellow  # nose-bottom
    mesh_colors[3] = yellow  # nose-left
    mesh_colors[4] = blue  # fuselage-left
    mesh_colors[5] = blue  # fuselage-top
    mesh_colors[6] = blue  # fuselage-right
    mesh_colors[7] = red  # fuselage-bottom
    mesh_colors[8] = green  # wing
    mesh_colors[9] = green  # wing
    mesh_colors[10] = green  # horizontal tail
    mesh_colors[11] = green  # horizontal tail
    mesh_colors[12] = blue  # vertical tail

    return types.Points(points), mesh_colors


def points_to_mesh(points_in: types.Points) -> npt.NDArray[Any]:
    """ "
    Converts points to triangular mesh
    Each mesh face is defined by three 3D points
        (a rectangle requires two triangular mesh faces)
    """
    # Check inputs
    types.check_points(points_in)

    # Convert points to a mesh
    points = points_in.T
    mesh = np.array(
        [
            [points[0], points[1], points[2]],  # nose-top
            [points[0], points[1], points[4]],  # nose-right
            [points[0], points[3], points[4]],  # nose-bottom
            [points[0], points[3], points[2]],  # nose-left
            [points[5], points[2], points[3]],  # fuselage-left
            [points[5], points[1], points[2]],  # fuselage-top
            [points[5], points[1], points[4]],  # fuselage-right
            [points[5], points[3], points[4]],  # fuselage-bottom
            [points[6], points[7], points[9]],  # wing
            [points[7], points[8], points[9]],  # wing
            [points[10], points[11], points[12]],  # horizontal tail
            [points[10], points[12], points[13]],  # horizontal tail
            [points[5], points[14], points[15]],  # vertical tail
        ]
    )
    return mesh

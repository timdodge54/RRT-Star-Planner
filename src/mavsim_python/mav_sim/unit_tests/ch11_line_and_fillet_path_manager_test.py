"""ch11_line_and_fillet_path_manager_tests.py: Implements some basic tests for the chapter 11 martial."""
# pylint: disable=too-many-lines

import gc
import itertools
import os
import pickle
from typing import Any, Dict, List, Tuple

import numpy as np
from mav_sim.chap11.fillet_manager import construct_fillet_circle, construct_fillet_line
from mav_sim.chap11.line_manager import construct_line
from mav_sim.chap11.path_manager_utilities import (
    HalfSpaceParams,
    WaypointIndices,
    inHalfSpace,
)
from mav_sim.message_types.msg_waypoints import MsgWaypoints

PRECISION = 1e-4


def inHalfSpace_test(test_case: Tuple[Dict[str, Any], Any]) -> bool:
    """Tests the inHalfSpace function."""
    calculated_output = inHalfSpace(**test_case[0])

    if bool(calculated_output) != bool(test_case[1]):
        print("Failed test!")
        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(test_case[1])
        return False
    return True


def construct_line_test(test_case: Tuple[Any, List[Any]]) -> int:
    """Tests the construct_line function."""
    ptr = WaypointIndices()
    for ptr_it in range(test_case[0].num_waypoints - 1):
        calculated_output = construct_line(waypoints=test_case[0], ptr=ptr)
        # pylint: disable=too-many-boolean-expressions
        if (
            (calculated_output[0].type != test_case[1][ptr_it][0].type)
            or (
                np.abs(calculated_output[0].airspeed - test_case[1][ptr_it][0].airspeed)
                > PRECISION
            )
            or (
                np.abs(
                    np.array(calculated_output[0].line_origin)
                    - np.array(test_case[1][ptr_it][0].line_origin)
                )
                > PRECISION
            ).any()
            or (
                np.abs(
                    np.array(calculated_output[0].line_direction)
                    - np.array(test_case[1][ptr_it][0].line_direction)
                )
                > PRECISION
            ).any()
            or (
                np.abs(
                    np.array(calculated_output[0].orbit_center)
                    - np.array(test_case[1][ptr_it][0].orbit_center)
                )
                > PRECISION
            ).any()
            or (
                np.abs(
                    calculated_output[0].orbit_radius
                    - test_case[1][ptr_it][0].orbit_radius
                )
                > PRECISION
            )
            or (
                calculated_output[0].orbit_direction
                != test_case[1][ptr_it][0].orbit_direction
            )
            or (
                np.abs(
                    (
                        np.array(calculated_output[1].normal).reshape(1, -1)
                        @ np.array(test_case[1][ptr_it][1].normal).reshape(-1, 1)
                    )
                    - 1
                )
                > PRECISION
            )
            or (
                np.abs(
                    np.array(calculated_output[1].point)
                    - np.array(test_case[1][ptr_it][1].point)
                )
                > PRECISION
            ).any()
        ):
            print("Failed test!")
            print("Calculated output:")
            calculated_output[0].print()
            calculated_output[1].print()
            print("Expected output:")
            test_case[1][ptr_it][0].print()
            test_case[1][ptr_it][1].print()
            return ptr_it
        ptr.increment_pointers(num_waypoints=test_case[0].num_waypoints)
    return -1


def construct_fillet_line_test(test_case: Tuple[Tuple[Any, Any], List[Any]]) -> int:
    """Tests the construct_fillet_line function."""
    ptr = WaypointIndices()
    for ptr_it in range(test_case[0][0].num_waypoints - 1):
        calculated_output = construct_fillet_line(
            waypoints=test_case[0][0], ptr=ptr, radius=test_case[0][1]
        )
        # pylint: disable=too-many-boolean-expressions
        if (
            (calculated_output[0].type != test_case[1][ptr_it][0].type)
            or (
                np.abs(calculated_output[0].airspeed - test_case[1][ptr_it][0].airspeed)
                > PRECISION
            )
            or (
                np.abs(
                    np.array(calculated_output[0].line_origin)
                    - np.array(test_case[1][ptr_it][0].line_origin)
                )
                > PRECISION
            ).any()
            or (
                np.abs(
                    np.array(calculated_output[0].line_direction)
                    - np.array(test_case[1][ptr_it][0].line_direction)
                )
                > PRECISION
            ).any()
            or (
                np.abs(
                    np.array(calculated_output[0].orbit_center)
                    - np.array(test_case[1][ptr_it][0].orbit_center)
                )
                > PRECISION
            ).any()
            or (
                np.abs(
                    calculated_output[0].orbit_radius
                    - test_case[1][ptr_it][0].orbit_radius
                )
                > PRECISION
            )
            or (
                calculated_output[0].orbit_direction
                != test_case[1][ptr_it][0].orbit_direction
            )
            or (
                np.abs(
                    (
                        np.array(calculated_output[1].normal).reshape(1, -1)
                        @ np.array(test_case[1][ptr_it][1].normal).reshape(-1, 1)
                    )
                    - 1
                )
                > PRECISION
            )
            or (
                np.abs(
                    np.array(calculated_output[1].point)
                    - np.array(test_case[1][ptr_it][1].point)
                )
                > PRECISION
            ).any()
        ):
            print("Failed test!")
            print("Calculated output:")
            calculated_output[0].print()
            calculated_output[1].print()
            print("Expected output:")
            test_case[1][ptr_it][0].print()
            test_case[1][ptr_it][1].print()
            return ptr_it
        ptr.increment_pointers(num_waypoints=test_case[0][0].num_waypoints)
    return -1


def construct_fillet_circle_test(test_case: Tuple[Tuple[Any, Any], List[Any]]) -> int:
    """Tests the construct_fillet_circle function."""
    ptr = WaypointIndices()
    for ptr_it in range(test_case[0][0].num_waypoints - 1):
        calculated_output = construct_fillet_circle(
            waypoints=test_case[0][0], ptr=ptr, radius=test_case[0][1]
        )
        # pylint: disable=too-many-boolean-expressions
        if (
            (calculated_output[0].type != test_case[1][ptr_it][0].type)
            or (
                np.abs(calculated_output[0].airspeed - test_case[1][ptr_it][0].airspeed)
                > PRECISION
            )
            or (
                np.abs(
                    np.array(calculated_output[0].line_origin)
                    - np.array(test_case[1][ptr_it][0].line_origin)
                )
                > PRECISION
            ).any()
            or (
                np.abs(
                    np.array(calculated_output[0].line_direction)
                    - np.array(test_case[1][ptr_it][0].line_direction)
                )
                > PRECISION
            ).any()
            or (
                np.abs(
                    np.array(calculated_output[0].orbit_center)
                    - np.array(test_case[1][ptr_it][0].orbit_center)
                )
                > PRECISION
            ).any()
            or (
                np.abs(
                    calculated_output[0].orbit_radius
                    - test_case[1][ptr_it][0].orbit_radius
                )
                > PRECISION
            )
            or (
                calculated_output[0].orbit_direction
                != test_case[1][ptr_it][0].orbit_direction
            )
            or (
                np.abs(
                    (
                        np.array(calculated_output[1].normal).reshape(1, -1)
                        @ np.array(test_case[1][ptr_it][1].normal).reshape(-1, 1)
                    )
                    - 1
                )
                > PRECISION
            )
            or (
                np.abs(
                    np.array(calculated_output[1].point)
                    - np.array(test_case[1][ptr_it][1].point)
                )
                > PRECISION
            ).any()
        ):
            print("Failed test!")
            print("Calculated output:")
            calculated_output[0].print()
            calculated_output[1].print()
            print("Expected output:")
            test_case[1][ptr_it][0].print()
            test_case[1][ptr_it][1].print()
            return ptr_it
        ptr.increment_pointers(num_waypoints=test_case[0][0].num_waypoints)
    return -1


def run_all_tests() -> None:
    """Run all tests."""
    gc.disable()
    # Open archive
    with open(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "ch11_test_archive.pkl"
        ),
        "rb",
    ) as file:
        tests_archive = pickle.load(file)
    gc.enable()
    # Run tests
    print("Starting inHalfSpace test")
    for test_count in range(len(tests_archive["inHalfSpace"])):
        if not inHalfSpace_test(tests_archive["inHalfSpace"][test_count]):
            print("Failed on test id: " + str(test_count))
            break
    print("End of test\n")
    print("Starting construct_line test")
    for test_count in range(len(tests_archive["construct_line"])):
        fail_case = construct_line_test(tests_archive["construct_line"][test_count])
        if -1 != fail_case:
            print(
                "Failed on test id: "
                + str(test_count)
                + " list index: "
                + str(fail_case)
            )
            break
    print("End of test\n")
    print("Starting construct_fillet_line test")
    for test_count in range(len(tests_archive["construct_fillet_line"])):
        fail_case = construct_fillet_line_test(
            tests_archive["construct_fillet_line"][test_count]
        )
        if -1 != fail_case:
            print(
                "Failed on test id: "
                + str(test_count)
                + " list index: "
                + str(fail_case)
            )
            break
    print("End of test\n")
    print("Starting construct_fillet_circle test")
    for test_count in range(len(tests_archive["construct_fillet_circle"])):
        fail_case = construct_fillet_circle_test(
            tests_archive["construct_fillet_circle"][test_count]
        )
        if -1 != fail_case:
            print(
                "Failed on test id: "
                + str(test_count)
                + " list index: "
                + str(fail_case)
            )
            break
    print("End of test\n")


if __name__ == "__main__":
    run_all_tests()

    # gc.disable()
    # # Open archive
    # with open(
    #     os.path.join(
    #         os.path.dirname(os.path.realpath(__file__)), "ch11_test_archive.pkl"
    #     ),
    #     "rb",
    # ) as file:
    #     tests_archive = pickle.load(file)
    # gc.enable()

    # construct_line_test(tests_archive["construct_line"][2])

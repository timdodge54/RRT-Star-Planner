"""ch11b_dubins_path_manager_tests.py: Implements some basic tests for the chapter 11 martial."""
# pylint: disable=too-many-lines

import gc
import os
import pickle
from typing import Any, Dict, List, Tuple, Union, cast

PRECISION = 1e-4

import numpy as np
from mav_sim.chap11.dubins_parameters import (
    calculate_lsl,
    calculate_lsr,
    calculate_rsl,
    calculate_rsr,
)


def rsr_test(test_case: Tuple[Dict[str, Any], Any]) -> bool:
    """Test the calculate_rsr function."""
    calculated_output = calculate_rsr(**test_case[0])

    if (
        (np.abs(calculated_output.L - test_case[1].L) > PRECISION)
        or (
            (
                np.array(calculated_output.c_s).reshape(-1, 1)
                - np.array(test_case[1].c_s).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (np.abs(calculated_output.lam_s - test_case[1].lam_s) > PRECISION)
        or (
            (
                np.array(calculated_output.c_e).reshape(-1, 1)
                - np.array(test_case[1].c_e).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (np.abs(calculated_output.lam_e - test_case[1].lam_e) > PRECISION)
        or (
            (
                np.array(calculated_output.z1).reshape(-1, 1)
                - np.array(test_case[1].z1).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (
            (
                np.array(calculated_output.q1).reshape(-1, 1)
                - np.array(test_case[1].q1).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (
            (
                np.array(calculated_output.z2).reshape(-1, 1)
                - np.array(test_case[1].z2).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (
            (
                np.array(calculated_output.z3).reshape(-1, 1)
                - np.array(test_case[1].z3).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (
            (
                np.array(calculated_output.q3).reshape(-1, 1)
                - np.array(test_case[1].q3).reshape(-1, 1)
            )
            > PRECISION
        ).any()
    ):
        print("Failed test!")
        print("Calculated output:")
        calculated_output.print()
        print("Expected output:")
        test_case[1].print()
        return False
    return True


def rsl_test(test_case: Tuple[Dict[str, Any], Any]) -> bool:
    """Test the calculate_rsl function."""
    calculated_output = calculate_rsl(**test_case[0])

    if (
        (np.abs(calculated_output.L - test_case[1].L) > PRECISION)
        or (
            (
                np.array(calculated_output.c_s).reshape(-1, 1)
                - np.array(test_case[1].c_s).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (np.abs(calculated_output.lam_s - test_case[1].lam_s) > PRECISION)
        or (
            (
                np.array(calculated_output.c_e).reshape(-1, 1)
                - np.array(test_case[1].c_e).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (np.abs(calculated_output.lam_e - test_case[1].lam_e) > PRECISION)
        or (
            (
                np.array(calculated_output.z1).reshape(-1, 1)
                - np.array(test_case[1].z1).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (
            (
                np.array(calculated_output.q1).reshape(-1, 1)
                - np.array(test_case[1].q1).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (
            (
                np.array(calculated_output.z2).reshape(-1, 1)
                - np.array(test_case[1].z2).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (
            (
                np.array(calculated_output.z3).reshape(-1, 1)
                - np.array(test_case[1].z3).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (
            (
                np.array(calculated_output.q3).reshape(-1, 1)
                - np.array(test_case[1].q3).reshape(-1, 1)
            )
            > PRECISION
        ).any()
    ):
        print("Failed test!")
        print("Calculated output:")
        calculated_output.print()
        print("Expected output:")
        test_case[1].print()
        return False
    return True


def lsr_test(test_case: Tuple[Dict[str, Any], Any]) -> bool:
    """Test the calculate_lsr function."""
    calculated_output = calculate_lsr(**test_case[0])

    if (
        (np.abs(calculated_output.L - test_case[1].L) > PRECISION)
        or (
            (
                np.array(calculated_output.c_s).reshape(-1, 1)
                - np.array(test_case[1].c_s).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (np.abs(calculated_output.lam_s - test_case[1].lam_s) > PRECISION)
        or (
            (
                np.array(calculated_output.c_e).reshape(-1, 1)
                - np.array(test_case[1].c_e).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (np.abs(calculated_output.lam_e - test_case[1].lam_e) > PRECISION)
        or (
            (
                np.array(calculated_output.z1).reshape(-1, 1)
                - np.array(test_case[1].z1).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (
            (
                np.array(calculated_output.q1).reshape(-1, 1)
                - np.array(test_case[1].q1).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (
            (
                np.array(calculated_output.z2).reshape(-1, 1)
                - np.array(test_case[1].z2).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (
            (
                np.array(calculated_output.z3).reshape(-1, 1)
                - np.array(test_case[1].z3).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (
            (
                np.array(calculated_output.q3).reshape(-1, 1)
                - np.array(test_case[1].q3).reshape(-1, 1)
            )
            > PRECISION
        ).any()
    ):
        print("Failed test!")
        print("Calculated output:")
        calculated_output.print()
        print("Expected output:")
        test_case[1].print()
        return False
    return True


def lsl_test(test_case: Tuple[Dict[str, Any], Any]) -> bool:
    """Test the calculate_lsl function."""
    calculated_output = calculate_lsl(**test_case[0])

    if (
        (np.abs(calculated_output.L - test_case[1].L) > PRECISION)
        or (
            (
                np.array(calculated_output.c_s).reshape(-1, 1)
                - np.array(test_case[1].c_s).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (np.abs(calculated_output.lam_s - test_case[1].lam_s) > PRECISION)
        or (
            (
                np.array(calculated_output.c_e).reshape(-1, 1)
                - np.array(test_case[1].c_e).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (np.abs(calculated_output.lam_e - test_case[1].lam_e) > PRECISION)
        or (
            (
                np.array(calculated_output.z1).reshape(-1, 1)
                - np.array(test_case[1].z1).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (
            (
                np.array(calculated_output.q1).reshape(-1, 1)
                - np.array(test_case[1].q1).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (
            (
                np.array(calculated_output.z2).reshape(-1, 1)
                - np.array(test_case[1].z2).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (
            (
                np.array(calculated_output.z3).reshape(-1, 1)
                - np.array(test_case[1].z3).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (
            (
                np.array(calculated_output.q3).reshape(-1, 1)
                - np.array(test_case[1].q3).reshape(-1, 1)
            )
            > PRECISION
        ).any()
    ):
        print("Failed test!")
        print("Calculated output:")
        calculated_output.print()
        print("Expected output:")
        test_case[1].print()
        return False
    return True


def run_tests(test_case: Union[int, None] = None) -> None:
    """Run tests.

    @Arguments
    test_case: Either None to run all tests or the test case that you want to run.
    """
    gc.disable()
    # Open archive
    with open(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "ch11b_test_archive.pkl"
        ),
        "rb",
    ) as file:
        tests_archive = pickle.load(file)
    gc.enable()
    # Run tests
    indexes: List[int] = cast(List[int], range(len(tests_archive["inputs"])))
    if test_case is not None:
        indexes = [test_case]
    print("Starting RSR test")
    for test_count in indexes:
        if not rsr_test(
            (tests_archive["inputs"][test_count], tests_archive["rsr"][test_count])
        ):
            print("Failed on test id: " + str(test_count))
            break
    print("Starting RSL test")
    for test_count in indexes:
        if not rsl_test(
            (tests_archive["inputs"][test_count], tests_archive["rsl"][test_count])
        ):
            print("Failed on test id: " + str(test_count))
            break
    print("Starting LSR test")
    for test_count in indexes:
        if not lsr_test(
            (tests_archive["inputs"][test_count], tests_archive["lsr"][test_count])
        ):
            print("Failed on test id: " + str(test_count))
            break
    print("Starting LSL test")
    for test_count in indexes:
        if not lsl_test(
            (tests_archive["inputs"][test_count], tests_archive["lsl"][test_count])
        ):
            print("Failed on test id: " + str(test_count))
            break
    print("End of test\n")


if __name__ == "__main__":
    run_tests(None)

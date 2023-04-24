"""ch7_sensors_tests.py: Implements some basic tests for the chapter 7 martial."""
# pylint: disable=too-many-lines

import itertools
import os
import pickle
from typing import Any, Dict, List

import numpy as np
from mav_sim.chap7 import mav_dynamics


def accelerometer_test(test_cases: List[Dict[str, Any]]) -> None:
    """Tests the accelerometer function."""
    print("Starting accelerometer test")
    for test_case_it in test_cases:
        calculated_output = mav_dynamics.accelerometer(**test_case_it["input"])

        if (
            1e-12
            < np.abs(np.array(calculated_output) - np.array(test_case_it["output"]))
        ).any():
            print("Failed test!")
            print("Calculated output:")
            print(calculated_output)
            print("Expected output:")
            print(test_case_it["output"])
            print("Difference:")
            print(np.array(calculated_output) - np.array(test_case_it["output"]))
            break
    print("End of test\n")


def gyro_test(test_cases: List[Dict[str, Any]]) -> None:
    """Tests the gyro function."""
    print("Starting gyro test")
    for test_case_it in test_cases:
        calculated_output = mav_dynamics.gyro(**test_case_it["input"])
        if (
            1e-12
            < np.abs(np.array(calculated_output) - np.array(test_case_it["output"]))
        ).any():
            print("Failed test!")
            print("Calculated output:")
            print(calculated_output)
            print("Expected output:")
            print(test_case_it["output"])
            print("Difference:")
            print(np.array(calculated_output) - np.array(test_case_it["output"]))
            break
    print("End of test\n")


def pressure_test(test_cases: List[Dict[str, Any]]) -> None:
    """Tests the pressure function."""
    print("Starting pressure test")
    for test_case_it in test_cases:
        calculated_output = mav_dynamics.pressure(**test_case_it["input"])
        if (
            1e-12
            < np.abs(np.array(calculated_output) - np.array(test_case_it["output"]))
        ).any():
            print("Failed test!")
            print("Calculated output:")
            print(calculated_output)
            print("Expected output:")
            print(test_case_it["output"])
            print("Difference:")
            print(np.array(calculated_output) - np.array(test_case_it["output"]))
            break
    print("End of test\n")


def magnometer_test(test_cases: List[Dict[str, Any]]) -> None:
    """Tests the magnometer function."""
    print("Starting magnometer test")
    for test_case_it in test_cases:
        calculated_output = mav_dynamics.magnetometer(**test_case_it["input"])
        if (
            1e-12
            < np.abs(np.array(calculated_output) - np.array(test_case_it["output"]))
        ).any():
            print("Failed test!")
            print("Calculated output:")
            print(calculated_output)
            print("Expected output:")
            print(test_case_it["output"])
            print("Difference:")
            print(np.array(calculated_output) - np.array(test_case_it["output"]))
            break
    print("End of test\n")


def gps_error_trans_update_test(test_cases: List[Dict[str, Any]]) -> None:
    """Tests the gps_error_trans_update function."""
    print("Starting gps_error_trans_update test")
    for test_case_it in test_cases:
        calculated_output = mav_dynamics.gps_error_trans_update(**test_case_it["input"])
        if (
            1e-12
            < np.abs(calculated_output.to_array() - test_case_it["output"].to_array())
        ).any():
            print("Failed test!")
            print("Calculated output:")
            calculated_output.print()
            print("Expected output:")
            test_case_it["output"].print()
            print("Difference:")
            print(calculated_output.to_array() - test_case_it["output"].to_array())
            break
    print("End of test\n")


def gps_test(test_cases: List[Dict[str, Any]]) -> None:
    """Tests the gps function."""
    print("Starting gps test")
    for test_case_it in test_cases:
        calculated_output = mav_dynamics.gps(**test_case_it["input"])
        if (
            1e-12
            < np.abs(np.array(calculated_output) - np.array(test_case_it["output"]))
        ).any():
            print("Failed test!")
            print("Calculated output:")
            print(calculated_output)
            print("Expected output:")
            print(test_case_it["output"])
            print("Difference:")
            print(np.array(calculated_output) - np.array(test_case_it["output"]))
            break
    print("End of test\n")


def calculate_sensor_readings_test(test_cases: List[Dict[str, Any]]) -> None:
    """Tests the calculate_sensor_readings function."""
    print("Starting calculate_sensor_readings test")
    for test_case_it in test_cases:
        temp = mav_dynamics.calculate_sensor_readings(**test_case_it["input"])
        calculated_output = np.concatenate((temp[0].to_array(), temp[1].to_array()))
        if (
            1e-12
            < np.abs(np.array(calculated_output) - np.array(test_case_it["output"]))
        ).any():
            print("Failed test!")
            print("Calculated output:")
            print(calculated_output)
            print("Expected output:")
            print(test_case_it["output"])
            print("Difference:")
            print(calculated_output - test_case_it["output"])
            break
    print("End of test\n")


def run_all_tests() -> None:
    """Run all tests."""
    # Open archive
    with open(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "ch7_test_archive.pkl"
        ),
        "rb",
    ) as file:
        tests_archive = pickle.load(file)
    # Run tests
    accelerometer_test(tests_archive["accelerometer"])
    gyro_test(tests_archive["gyro"])
    pressure_test(tests_archive["pressure"])
    magnometer_test(tests_archive["magnometer"])
    gps_error_trans_update_test(tests_archive["gps_error_tran_update"])
    gps_test(tests_archive["gps"])
    calculate_sensor_readings_test(tests_archive["calculate_sensor_readings"])


if __name__ == "__main__":
    run_all_tests()

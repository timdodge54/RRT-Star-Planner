"""ch7_sensors_tests.py: Implements some basic tests for the chapter 7 martial."""
# pylint: disable=too-many-lines

import itertools
import os
import pickle
from typing import Any, Dict, List, Literal, Tuple, Union, cast

import numpy as np
from mav_sim.chap10.path_follower import follow_orbit, follow_straight_line
from mav_sim.message_types.msg_path import MsgPath
from mav_sim.message_types.msg_state import MsgState


def gen_follow_straight_line_vars() -> List[Tuple[Dict[str, Any], Any]]:
    """Generates test inputs and outputs for the follow_straight_line function."""
    output: List[Tuple[Dict[str, Any], Any]] = []
    # Base cases
    inputs: Dict[str, Any] = {
        "path": MsgPath(
            type="line",
            plot_updated=False,
            airspeed=0,
            line_origin=np.zeros([3, 1], dtype=float),
            line_direction=np.array((1, 0, 0), dtype=float).reshape(-1, 1),
            orbit_center=np.array(
                (float("nan"), float("nan"), float("nan")), dtype=float
            ).reshape(-1, 1),
            orbit_radius=float("nan"),
            orbit_direction="CW",
        ),
        "state": MsgState(
            north=0,
            east=0,
            altitude=0,
            phi=0,
            theta=0,
            psi=0,
            Va=0,
            alpha=0,
            beta=0,
            p=0,
            q=0,
            r=0,
            Vg=0,
            gamma=0,
            chi=0,
            wn=0,
            we=0,
            bx=0,
            by=0,
            bz=0,
        ),
        "k_path": float(0),
        "chi_inf": float(0),
    }
    output.append((inputs, follow_straight_line(**inputs)))
    # Exhaustive cases
    for airspeed in np.arange(25, 25.1, 1):
        for line_origin in itertools.product(np.arange(-100, 100.1, 200), repeat=3):
            for line_direction_it in itertools.product(np.arange(-1, 1.1, 2), repeat=3):
                line_direction = np.array(line_direction_it) / np.linalg.norm(
                    line_direction_it
                )
                path = MsgPath(
                    type="line",
                    plot_updated=False,
                    airspeed=airspeed,
                    line_origin=np.array(line_origin, dtype=float).reshape(-1, 1),
                    line_direction=np.array(line_direction, dtype=float).reshape(-1, 1),
                    orbit_center=np.array(
                        (float("nan"), float("nan"), float("nan")), dtype=float
                    ).reshape(-1, 1),
                    orbit_radius=float("nan"),
                    orbit_direction="CW",
                )
                for position_vec in itertools.product(
                    np.arange(-190, -189, 2), repeat=3
                ):
                    for direction_vec in itertools.product(
                        np.arange(-2 * np.pi, 2 * np.pi + 0.1, 7), repeat=3
                    ):
                        for alpha_beta in itertools.product(
                            np.arange(-2 * np.pi, 2 * np.pi + 0.1, 7), repeat=2
                        ):
                            for angle_rate_vec in itertools.product(
                                np.arange(-np.pi / 2, np.pi / 2 + 0.1, np.pi), repeat=3
                            ):
                                for ground_speed in np.arange(0, 50.1, 50):
                                    for gamma_chi in itertools.product(
                                        np.arange(-2 * np.pi, 2 * np.pi + 0.1, 7),
                                        repeat=2,
                                    ):
                                        for wind_speed_vec in itertools.product(
                                            np.arange(-50, 50 + 0.1, 100), repeat=2
                                        ):
                                            for gyro_bias_vec in itertools.product(
                                                np.arange(
                                                    -np.pi / 2,
                                                    np.pi / 2 + 0.1,
                                                    2 * np.pi,
                                                ),
                                                repeat=3,
                                            ):
                                                state = MsgState(
                                                    north=position_vec[0],
                                                    east=position_vec[1],
                                                    altitude=position_vec[2],
                                                    phi=direction_vec[0],
                                                    theta=direction_vec[1],
                                                    psi=direction_vec[2],
                                                    Va=airspeed,
                                                    alpha=alpha_beta[0],
                                                    beta=alpha_beta[1],
                                                    p=angle_rate_vec[0],
                                                    q=angle_rate_vec[1],
                                                    r=angle_rate_vec[2],
                                                    Vg=ground_speed,
                                                    gamma=gamma_chi[0],
                                                    chi=gamma_chi[1],
                                                    wn=wind_speed_vec[0],
                                                    we=wind_speed_vec[1],
                                                    bx=gyro_bias_vec[0],
                                                    by=gyro_bias_vec[1],
                                                    bz=gyro_bias_vec[2],
                                                )
                                                for k_path in np.arange(0, 100.1, 50):
                                                    for chi_inf in np.arange(
                                                        -np.pi, np.pi + 0.1, 4
                                                    ):
                                                        inputs = {
                                                            "path": path,
                                                            "state": state,
                                                            "k_path": k_path,
                                                            "chi_inf": chi_inf,
                                                        }
                                                        output.append(
                                                            (
                                                                inputs,
                                                                follow_straight_line(
                                                                    **inputs
                                                                ),
                                                            )
                                                        )
    return output


def follow_straight_line_test(test_cases: List[Tuple[Dict[str, Any], Any]]) -> None:
    """Tests the follow_straight_line function."""
    print("Starting follow_straight_line test")
    for test_case_it in test_cases:
        calculated_output = follow_straight_line(**test_case_it[0])

        if (
            (
                1e-6 < np.abs(calculated_output.to_array() - test_case_it[1].to_array())
            ).any()
            or np.isnan(calculated_output.to_array()).any()
            or np.isinf(calculated_output.to_array()).any()
        ):
            print("Failed test!")
            print("Calculated output:")
            print(calculated_output.to_array())
            print("Expected output:")
            print(test_case_it[1].to_array())
            print("Difference:")
            print(calculated_output.to_array() - test_case_it[1].to_array())
            break
    print("End of test\n")


def gen_follow_orbit_vars() -> List[Tuple[Dict[str, Any], Any]]:
    """Generates test inputs and outputs for the follow_orbit function."""
    output: List[Tuple[Dict[str, Any], Any]] = []
    # Base cases
    inputs: Dict[str, Any] = {
        "path": MsgPath(
            type="orbit",
            plot_updated=False,
            airspeed=float(0),
            line_origin=np.array(
                (float("nan"), float("nan"), float("nan")), dtype=float
            ).reshape(-1, 1),
            line_direction=np.array(
                (float("nan"), float("nan"), float("nan")), dtype=float
            ).reshape(-1, 1),
            orbit_center=np.zeros([3, 1], dtype=float),
            orbit_radius=float(1),
            orbit_direction="CW",
        ),
        "state": MsgState(
            north=0,
            east=0,
            altitude=0,
            phi=0,
            theta=0,
            psi=0,
            Va=0,
            alpha=0,
            beta=0,
            p=0,
            q=0,
            r=0,
            Vg=0,
            gamma=0,
            chi=0,
            wn=0,
            we=0,
            bx=0,
            by=0,
            bz=0,
        ),
        "k_orbit": float(0),
        "gravity": float(9.81999999),
    }
    output.append((inputs, follow_orbit(**inputs)))
    # Exhaustive cases
    for airspeed in np.arange(25, 25.1, 1):
        for orbit_center in itertools.product(np.arange(-100, 100.1, 200), repeat=3):
            for orbit_radius in np.arange(1, 100, 50):
                for orbit_direction in ["CW", "CCW"]:
                    path = MsgPath(
                        type="orbit",
                        plot_updated=False,
                        airspeed=airspeed,
                        line_origin=np.array(
                            (float("nan"), float("nan"), float("nan")), dtype=float
                        ).reshape(-1, 1),
                        line_direction=np.array(
                            (float("nan"), float("nan"), float("nan")), dtype=float
                        ).reshape(-1, 1),
                        orbit_center=np.array(orbit_center, dtype=float).reshape(-1, 1),
                        orbit_radius=orbit_radius,
                        orbit_direction=cast(
                            Union[Literal["CW"], Literal["CCW"]], orbit_direction
                        ),
                    )
                    for position_vec in itertools.product(
                        np.arange(-190, -189, 2), repeat=3
                    ):
                        for direction_vec in itertools.product(
                            np.arange(-2 * np.pi, 2 * np.pi + 0.1, 7), repeat=3
                        ):
                            for alpha_beta in itertools.product(
                                np.arange(-2 * np.pi, 2 * np.pi + 0.1, 7), repeat=2
                            ):
                                for angle_rate_vec in itertools.product(
                                    np.arange(-np.pi / 2, np.pi / 2 + 0.1, np.pi),
                                    repeat=3,
                                ):
                                    for ground_speed in np.arange(0, 50.1, 50):
                                        for gamma_chi in itertools.product(
                                            np.arange(-2 * np.pi, 2 * np.pi + 0.1, 7),
                                            repeat=2,
                                        ):
                                            for wind_speed_vec in itertools.product(
                                                np.arange(-50, 50 + 0.1, 100), repeat=2
                                            ):
                                                for gyro_bias_vec in itertools.product(
                                                    np.arange(
                                                        -np.pi / 2,
                                                        np.pi / 2 + 0.1,
                                                        2 * np.pi,
                                                    ),
                                                    repeat=3,
                                                ):
                                                    state = MsgState(
                                                        north=position_vec[0],
                                                        east=position_vec[1],
                                                        altitude=position_vec[2],
                                                        phi=direction_vec[0],
                                                        theta=direction_vec[1],
                                                        psi=direction_vec[2],
                                                        Va=airspeed,
                                                        alpha=alpha_beta[0],
                                                        beta=alpha_beta[1],
                                                        p=angle_rate_vec[0],
                                                        q=angle_rate_vec[1],
                                                        r=angle_rate_vec[2],
                                                        Vg=ground_speed,
                                                        gamma=gamma_chi[0],
                                                        chi=gamma_chi[1],
                                                        wn=wind_speed_vec[0],
                                                        we=wind_speed_vec[1],
                                                        bx=gyro_bias_vec[0],
                                                        by=gyro_bias_vec[1],
                                                        bz=gyro_bias_vec[2],
                                                    )
                                                    for k_orbit in np.arange(
                                                        0, 100 + 0.1, 50
                                                    ):
                                                        inputs = {
                                                            "path": path,
                                                            "state": state,
                                                            "k_orbit": k_orbit,
                                                            "gravity": float(
                                                                9.81999999
                                                            ),
                                                        }
                                                        output.append(
                                                            (
                                                                inputs,
                                                                follow_orbit(**inputs),
                                                            )
                                                        )
    return output


def follow_orbit_test(test_cases: List[Tuple[Dict[str, Any], Any]]) -> None:
    """Tests the follow_orbit function."""
    print("Starting follow_orbit test")
    for test_case_it in test_cases:
        calculated_output = follow_orbit(**test_case_it[0])

        if (
            (
                1e-12
                < np.abs(calculated_output.to_array() - test_case_it[1].to_array())
            ).any()
            or np.isnan(calculated_output.to_array()).any()
            or np.isinf(calculated_output.to_array()).any()
        ):
            print("Failed test!")
            print("Calculated output:")
            print(calculated_output.to_array())
            print("Expected output:")
            print(test_case_it[1].to_array())
            print("Difference:")
            print(calculated_output.to_array() - test_case_it[1].to_array())
            break
    print("End of test\n")


def gen_test_archive() -> None:
    """Makes the input/output archive of awnsers."""
    archive: Dict[str, List[Tuple[Dict[str, Any], Any]]] = {}
    # Gather test cases
    print("Making follow_straight_line archive")
    archive["follow_straight_line"] = gen_follow_straight_line_vars()
    print("Making follow_orbit archive")
    archive["follow_orbit"] = gen_follow_orbit_vars()
    # Save to archive
    with open(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "ch10_test_archive.pkl"
        ),
        "wb",
    ) as file:
        pickle.dump(archive, file)


def run_all_tests() -> None:
    """Run all tests."""
    # Open archive
    with open(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "ch10_test_archive.pkl"
        ),
        "rb",
    ) as file:
        tests_archive = pickle.load(file)
    # Run tests
    follow_straight_line_test(tests_archive["follow_straight_line"])
    follow_orbit_test(tests_archive["follow_orbit"])


if __name__ == "__main__":
    # gen_test_archive()
    run_all_tests()

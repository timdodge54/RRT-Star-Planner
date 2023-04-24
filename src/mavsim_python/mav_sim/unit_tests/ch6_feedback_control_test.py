"""ch6_feedback_control_tests.py: Implements some basic tests for the chapter 6 material."""


import os
import pickle
from typing import Any, Dict, List, cast

import mav_sim.parameters.simulation_parameters as SIM
import numpy as np
from mav_sim.chap6.autopilot import Autopilot
from mav_sim.chap6.pd_control_with_rate import PDControlWithRate
from mav_sim.chap6.pi_control import PIControl
from mav_sim.chap6.tf_control import TFControl
from mav_sim.message_types.msg_autopilot import MsgAutopilot
from mav_sim.message_types.msg_delta import MsgDelta
from mav_sim.message_types.msg_state import MsgState
from mav_sim.parameters import control_parameters as CP


#############  Test structure definitions ########################
class PDControlWithRateTest:
    """Stores a test for the PDControlWithRate"""

    def __init__(self) -> None:
        # Initialization paramaters
        self.kp: float
        self.kd: float
        self.limit: float

        # Inputs
        self.y_ref: list[float] = []
        self.y: list[float] = []
        self.ydot: list[float] = []

        # Outputs
        self.outputs: list[float] = []

    def __str__(self) -> str:
        """Outputs the updates and outputs"""
        out = (
            "\nInitialization: kp: "
            + str(self.kp)
            + ", kd = "
            + str(self.kd)
            + ", limit = "
            + str(self.limit)
            + "\nInputs:\ny_ref = "
            + str(self.y_ref)
            + "\ny = "
            + str(self.y)
            + "\nydot = "
            + str(self.ydot)
            + "\noutputs:\n  "
            + str(self.outputs)
        )
        return out


class PIControlTest:
    """Stores a test for the PIControl"""

    def __init__(self) -> None:
        # Initialization paramaters
        self.kp: float
        self.ki: float
        self.Ts: float
        self.limit: float

        # Inputs
        self.y_ref: list[float] = []
        self.y: list[float] = []

        # Outputs
        self.outputs: list[float] = []

    def __str__(self) -> str:
        """Outputs the updates and outputs"""
        out = (
            "\nInitialization: kp: "
            + str(self.kp)
            + ", ki = "
            + str(self.ki)
            + ", Ts = "
            + str(self.Ts)
            + ", limit = "
            + str(self.limit)
            + "\nInputs:\ny_ref = "
            + str(self.y_ref)
            + "\ny = "
            + str(self.y)
            + "\noutputs:\n  "
            + str(self.outputs)
        )
        return out


class TFControlTest:
    """Stores a test for the TFControl"""

    def __init__(self) -> None:
        # Initialization paramaters
        self.k: float
        self.n0: float
        self.n1: float
        self.d0: float
        self.d1: float
        self.Ts: float
        self.limit: float

        # Inputs
        self.y: list[float] = []

        # Outputs
        self.outputs: list[float] = []

    def __str__(self) -> str:
        """Outputs the updates and outputs"""
        out = (
            "\nInitialization: k: "
            + str(self.k)
            + ", n0 = "
            + str(self.n0)
            + ", n1 = "
            + str(self.n1)
            + ", d0 = "
            + str(self.d0)
            + ", d1 = "
            + str(self.d1)
            + ", Ts = "
            + str(self.Ts)
            + ", limit = "
            + str(self.limit)
            + "\nInputs:\ny = "
            + str(self.y)
            + "\noutputs:\n  "
            + str(self.outputs)
        )
        return out


class AutopilotTest:
    """Stores a test for the TFControl"""

    def __init__(self) -> None:
        # Initialization paramaters
        self.ts_control: float

        # Inputs
        self.cmd: list[MsgAutopilot] = []
        self.state: list[MsgState] = []

        # Outputs
        self.delta: list[MsgDelta] = []
        self.commanded_state: list[MsgState] = []

    def __str__(self) -> str:
        """Outputs the updates and outputs"""
        out = "\nInitialization: ts_control: " + str(self.ts_control)

        count = 0
        for (cmd, state, delta, commanded_state) in zip(
            self.cmd, self.state, self.delta, self.commanded_state
        ):
            out += (
                "\nInputs("
                + str(count)
                + "):\ncmd = "
                + str(cmd)
                + "\nstate = "
                + str(state)
                + "\noutputs("
                + str(count)
                + "):\ndelta = "
                + str(delta)
                + "\ncommanded_state = "
                + str(commanded_state)
            )
            count += 1
        return out


#############  auto test run ######################
def run_auto_tests() -> None:
    """Runs all of the auto-generated tests"""
    # Load the tests
    with open(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "ch06_test_archive.pkl"
        ),
        "rb",
    ) as file:
        data = pickle.load(file)

        # Run the tests
        pd_with_rate_tests(tests=data["pd_with_rate"])
        pi_control_tests(tests=data["pi_control"])
        tf_control_tests(tests=data["tf_control"])
        autopilot_tests(tests=data["autopilot"])


def pd_with_rate_tests(
    tests: list[PDControlWithRateTest], threshold: float = 1e-4
) -> bool:
    """Runs the tests for PDControlWithRate"""
    # Evaluate the results
    print("\nStarting PDControlWithRate test")
    success = True
    for test in tests:
        # Create the controller
        ctrl = PDControlWithRate(kd=test.kd, kp=test.kp, limit=test.limit)

        # Generate the input / output date for the result
        out: list[float] = []
        for (y_ref, y, ydot) in zip(test.y_ref, test.y, test.ydot):
            # Generate inputs and output
            out.append(ctrl.update(y_ref=y_ref, y=y, ydot=ydot))

        # Test the results
        diff = np.linalg.norm(np.array(out) - np.array(test.outputs))
        if diff >= threshold:
            success = False
            print("\n\nFailed test!")
            print("test:\n", test, "\nActual_output:\n ", out)

    # Indicate success
    if success:
        print("Passed test on PDControlWithRate\n")
    return success


def pi_control_tests(tests: list[PIControlTest], threshold: float = 1e-4) -> bool:
    """Runs the tests for PIControlTest"""
    # Evaluate the results
    print("\nStarting PIControlTest test")
    success = True
    for test in tests:
        # Create the controller
        ctrl = PIControl(kp=test.kp, ki=test.ki, Ts=test.Ts, limit=test.limit)

        # Generate the input / output date for the result
        out: list[float] = []
        for (y_ref, y) in zip(test.y_ref, test.y):
            # Generate inputs and output
            out.append(ctrl.update(y_ref=y_ref, y=y))

        # Test the results
        diff = np.linalg.norm(np.array(out) - np.array(test.outputs))
        if diff >= threshold:
            success = False
            print("\n\nFailed test!")
            print("test:\n", test, "\nActual_output:\n ", out)

    # Indicate success
    if success:
        print("Passed test on PIControlTest\n")
    return success


def tf_control_tests(tests: list[TFControlTest], threshold: float = 1e-4) -> bool:
    """Runs the tests for TFControl"""
    # Evaluate the results
    print("\nStarting TFControl test")
    success = True
    for test in tests:
        # Create the controller
        ctrl = TFControl(
            k=test.k,
            n0=test.n0,
            n1=test.n1,
            d0=test.d0,
            d1=test.d1,
            Ts=test.Ts,
            limit=test.limit,
        )

        # Generate the input / output date for the result
        out: list[float] = []
        for y in test.y:
            # Generate inputs and output
            out.append(ctrl.update(y=y))

        # Test the results
        diff = np.linalg.norm(np.array(out) - np.array(test.outputs))
        if diff >= threshold:
            success = False
            print("\n\nFailed test!")
            print("test:\n", test, "\nActual_output:\n ", out)

    # Indicate success
    if success:
        print("Passed test on TFControl\n")
    return success


def mod_angle(angle: float) -> float:
    """Converts and angle to be in -pi and pi"""
    angle = np.arctan2(np.sin(angle.real), np.cos(angle.real))
    return cast(float, angle)


def compare_state_msg(msg1: MsgState, msg2: MsgState) -> float:
    """Computes the difference between to state messages"""
    diff = 0.0
    diff += np.abs(msg1.north - msg2.north)
    diff += np.abs(msg1.east - msg2.east)
    diff += np.abs(msg1.altitude - msg2.altitude)
    diff += np.abs(mod_angle(msg1.phi - msg2.phi))
    diff += np.abs(mod_angle(msg1.theta - msg2.theta))
    diff += np.abs(mod_angle(msg1.psi - msg2.psi))
    diff += np.abs(msg1.Va - msg2.Va)
    diff += np.abs(mod_angle(msg1.alpha - msg2.alpha))
    diff += np.abs(mod_angle(msg1.beta - msg2.beta))
    diff += np.abs(msg1.p - msg2.p)
    diff += np.abs(msg1.q - msg2.q)
    diff += np.abs(msg1.r - msg2.r)
    diff += np.abs(msg1.Vg - msg2.Vg)
    diff += np.abs(mod_angle(msg1.gamma - msg2.gamma))
    diff += np.abs(mod_angle(msg1.chi - msg2.chi))
    diff += np.abs(msg1.wn - msg2.wn)
    diff += np.abs(msg1.we - msg2.we)
    diff += np.abs(msg1.bx - msg2.bx)
    diff += np.abs(msg1.by - msg2.by)
    diff += np.abs(msg1.bz - msg2.bz)
    return diff


def compare_delta_msg(msg1: MsgDelta, msg2: MsgDelta) -> float:
    """Computes the difference between delta messages"""
    diff = 0.0
    diff += np.abs(mod_angle(msg1.elevator - msg2.elevator))
    diff += np.abs(mod_angle(msg1.aileron - msg2.aileron))
    diff += np.abs(mod_angle(msg1.rudder - msg2.rudder))
    diff += np.abs(msg1.throttle - msg2.throttle)
    return diff


def autopilot_tests(tests: list[AutopilotTest], threshold: float = 1e-4) -> bool:
    """Runs the tests for Autopilot"""
    # Evaluate the results
    print("\nStarting Autopilot test")
    success = True
    for test in tests:
        # Create the autopilot
        autopilot = Autopilot(ts_control=test.ts_control)

        # Generate the input / output date for the result
        diff = 0.0
        delta_out_vec: list[MsgDelta] = []
        cmd_state_vec: list[MsgState] = []
        for (cmd, state, delta, commanded_state) in zip(
            test.cmd, test.state, test.delta, test.commanded_state
        ):
            # Create the output
            delta_out, command_state_out = autopilot.update(cmd=cmd, state=state)
            delta_out_vec.append(delta_out)
            cmd_state_vec.append(command_state_out)

            # Compare the difference
            if delta != delta_out:
                if delta.throttle != delta_out.throttle:
                    print("throttle: ", delta.throttle, delta_out.throttle)
                if delta.elevator != delta_out.elevator:
                    print("elevator: ", delta.elevator, delta_out.elevator)
                if delta.aileron != delta_out.aileron:
                    print("aileron: ", delta.aileron, delta_out.aileron)
                if delta.rudder != delta_out.rudder:
                    print("rudder: ", delta.rudder, delta_out.rudder)
                # print("End")
            diff += compare_delta_msg(delta, delta_out)
            # diff += compare_state_msg(commanded_state, command_state_out)

        # Test the results
        if diff >= threshold:
            # Aggregate output data
            out = ""
            count = 0
            for (delta, cmd_state) in zip(delta_out_vec, cmd_state_vec):
                out += "\ndelta_out(" + str(count) + "): " + str(delta)
                out += "\ncommanded_state_out(" + str(count) + "): " + str(cmd_state)
                count += 1

            # Output failed test
            success = False
            print("\n\nFailed test!")
            print("test:\n", test, "\nActual_output:\n ", out)

    # Indicate success
    if success:
        print("Passed test on Autopilot\n")
    return success


#############  Hand tests ########################
def run_hand_tests() -> None:
    """Run all hand tests."""
    pd_control_with_rate_test()
    pi_control_test()
    tf_control_test()
    autopilot_test()


def pd_control_with_rate_test() -> None:
    """Tests the PDControlWithRate class."""
    print("Starting pd_control_with_rate test")
    # Inputs
    inputs: List[Dict[str, Any]] = [
        {
            "y_ref": float(0),
            "y": float(0),
            "ydot": float(0),
        },
        {
            "y_ref": float(3),
            "y": float(4),
            "ydot": float(5),
        },
    ]
    # Expected outputs
    outputs = [
        float(0),
        float(-0.43974730752905633),
    ]

    test_class = PDControlWithRate(kp=CP.roll_kp, kd=CP.roll_kd, limit=np.radians(45))
    for input_it, output_it in zip(inputs, outputs):
        calculated_output = test_class.update(**input_it)

        if (1e-6 < np.abs(np.array(calculated_output) - np.array(output_it))).any():
            print("Failed test!")
            print("Calculated output:")
            print(calculated_output)
            print("Expected output:")
            print(output_it)

    print("End of test\n")


def pi_control_test() -> None:
    """Tests the PIControl class."""
    print("Starting pi_control test")
    # Inputs
    inputs: List[Dict[str, Any]] = [
        {
            "y_ref": float(0),
            "y": float(0),
        },
        {
            "y_ref": float(3),
            "y": float(4),
        },
    ]
    # Expected outputs
    outputs = [
        float(0),
        float(-0.5235987755982988),
    ]

    test_class = PIControl(
        kp=CP.course_kp, ki=CP.course_ki, Ts=SIM.ts_simulation, limit=np.radians(30)
    )

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = test_class.update(**input_it)

        if (1e-6 < np.abs(np.array(calculated_output) - np.array(output_it))).any():
            print("Failed test!")
            print("Calculated output:")
            print(calculated_output)
            print("Expected output:")
            print(output_it)

    print("End of test\n")


def tf_control_test() -> None:
    """Tests the TFControl class."""
    print("Starting tf_control test")
    # Inputs
    inputs: List[Dict[str, Any]] = [
        {
            "y": float(0),
        },
        {
            "y": float(4),
        },
    ]
    # Expected outputs
    outputs = [
        float(0),
        float(0.794620186396522),
    ]

    for input_it, output_it in zip(inputs, outputs):
        test_class = TFControl(
            k=CP.yaw_damper_kr,
            n0=0.0,
            n1=1.0,
            d0=CP.yaw_damper_p_wo,
            d1=1,
            Ts=SIM.ts_simulation,
        )
        _ = test_class.update(**input_it)
        calculated_output = test_class.update(**input_it)

        if (1e-6 < np.abs(np.array(calculated_output) - np.array(output_it))).any():
            print("Failed test!")
            print("Calculated output:")
            print(calculated_output)
            print("Expected output:")
            print(output_it)

    print("End of test\n")


def autopilot_test() -> None:
    """Tests the AutoPilot class."""
    print("Starting autopilot test")
    # Inputs
    inputs: List[Dict[str, Any]] = [
        {
            "cmd": MsgAutopilot(),
            "state": MsgState(),
        },
        {
            "cmd": MsgAutopilot(),
            "state": MsgState(
                north=111,
                east=222,
                altitude=555,
                phi=0.4,
                theta=0.26,
                psi=1.7,
                Va=16,
                alpha=0.7,
                beta=0.32,
                p=0.3,
                q=0.2,
                r=0.1,
                Vg=102,
                gamma=0.123,
                chi=0.234,
                wn=1.1,
                we=1.23,
                bx=0.45,
                by=0.1765,
                bz=0.3465,
            ),
        },
    ]
    inputs[1]["cmd"].airspeed_command = 4
    inputs[1]["cmd"].course_command = 2
    inputs[1]["cmd"].altitude_command = 12
    inputs[1]["cmd"].phi_feedforward = 0.35
    # Expected outputs
    outputs = [
        (
            MsgDelta(elevator=0, aileron=0, rudder=0, throttle=0),
            MsgState(),
        ),
        (
            MsgDelta(
                elevator=0.7853981633974483,
                aileron=0.11389145757219538,
                rudder=0.01995510102269893,
                throttle=0,
            ),
            MsgState(
                north=0,
                east=0,
                altitude=12,
                phi=0.5235987755982988,
                theta=-0.5235987755982988,
                psi=0,
                Va=4,
                alpha=0,
                beta=0,
                p=0,
                q=0,
                r=0,
                Vg=0,
                gamma=0,
                chi=2,
                wn=0,
                we=0,
                bx=0,
                by=0,
                bz=0,
            ),
        ),
    ]

    test_class = Autopilot(ts_control=SIM.ts_simulation)
    for input_it, output_it in zip(inputs, outputs):
        calculated_output = test_class.update(**input_it)

        if (
            1e-6 < np.abs(calculated_output[0].to_array() - output_it[0].to_array())
        ).any():
            print("Failed test!")
            print("Calculated MsgDelta:")
            calculated_output[0].print()
            print("Expected MsgDelta:")
            output_it[0].print()

        if (
            1e-6 < np.abs(calculated_output[1].to_array() - output_it[1].to_array())
        ).any():
            print("Failed test!")
            print("Calculated MsgState:")
            calculated_output[1].print()
            print("Expected MsgState:")
            output_it[1].print()

    print("End of test\n")


#############  Combining all tests ########################
def run_all_tests() -> None:
    """Run hand and auto generated tests"""
    run_auto_tests()
    run_hand_tests()


if __name__ == "__main__":
    run_all_tests()

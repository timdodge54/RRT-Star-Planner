"""transforms_tests.py: Implements some basic tests for the functions defined in transforms.py
"""
from mav_sim.chap2.transforms import *


def rot_x_test() -> None:
    """
    test rot_x function
    """
    print("Starting rot_x test\n")
    # Inputs
    inputs = [{"angle": np.pi}, {"angle": 1.5}]
    # Expected outputs
    outputs = [
        np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]),
        np.array([[1, 0, 0], [0, 0.0707372, 0.99749499], [0, -0.99749499, 0.0707372]]),
    ]

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = rot_x(**input_it)

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (1e-8 < np.abs(calculated_output - output_it)).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def rot_y_test() -> None:
    """
    test rot_y function
    """
    print("Starting rot_y test\n")
    # Inputs
    inputs = [{"angle": np.pi}, {"angle": 1.5}]
    # Expected outputs
    outputs = [
        np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]]),
        np.array([[0.0707372, 0, -0.99749499], [0, 1, 0], [0.99749499, 0, 0.0707372]]),
    ]

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = rot_y(**input_it)

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (1e-8 < np.abs(calculated_output - output_it)).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def rot_z_test() -> None:
    """
    test rot_z function
    """
    print("Starting rot_z test\n")
    # Inputs
    inputs = [{"angle": np.pi}, {"angle": 1.5}]
    # Expected outputs
    outputs = [
        np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]]),
        np.array([[0.0707372, 0.99749499, 0], [-0.99749499, 0.0707372, 0], [0, 0, 1]]),
    ]

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = rot_z(**input_it)

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (1e-8 < np.abs(calculated_output - output_it)).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def rot_v_to_v1_test() -> None:
    """
    test rot_v_to_v1 function
    """
    print("Starting rot_v_to_v1 test\n")
    # Inputs
    inputs = [{"psi": (3 * np.pi) / 4}, {"psi": 2.5}]
    # Expected outputs
    outputs = [
        np.array(
            [[-0.70710678, 0.70710678, 0], [-0.70710678, -0.70710678, 0], [0, 0, 1]]
        ),
        np.array(
            [[-0.80114362, 0.59847214, 0], [-0.59847214, -0.80114362, 0], [0, 0, 1]]
        ),
    ]

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = rot_v_to_v1(**input_it)

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (1e-8 < np.abs(calculated_output - output_it)).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def rot_v1_to_v2_test() -> None:
    """
    test rot_v1_to_v2 function
    """
    print("Starting rot_v1_to_v2 test\n")
    # Inputs
    inputs = [{"theta": (3 * np.pi) / 4}, {"theta": 2.5}]
    # Expected outputs
    outputs = [
        np.array(
            [[-0.70710678, 0, -0.70710678], [0, 1, 0], [0.70710678, 0, -0.70710678]]
        ),
        np.array(
            [[-0.80114362, 0, -0.59847214], [0, 1, 0], [0.59847214, 0, -0.80114362]]
        ),
    ]

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = rot_v1_to_v2(**input_it)

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (1e-8 < np.abs(calculated_output - output_it)).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def rot_v2_to_b_test() -> None:
    """
    test rot_v2_to_b function
    """
    print("Starting rot_v2_to_b test\n")
    # Inputs
    inputs = [{"phi": (3 * np.pi) / 4}, {"phi": 2.5}]
    # Expected outputs
    outputs = [
        np.array(
            [[1, 0, 0], [0, -0.70710678, 0.70710678], [0, -0.70710678, -0.70710678]]
        ),
        np.array(
            [[1, 0, 0], [0, -0.80114362, 0.59847214], [0, -0.59847214, -0.80114362]]
        ),
    ]

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = rot_v2_to_b(**input_it)

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (1e-8 < np.abs(calculated_output - output_it)).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def rot_b_to_s_test() -> None:
    """
    test rot_b_to_s function
    """
    print("Starting rot_b_to_s test\n")
    # Inputs
    inputs = [{"alpha": (3 * np.pi) / 4}, {"alpha": 2.5}]
    # Expected outputs
    outputs = [
        np.array(
            [[-0.70710678, 0, 0.70710678], [0, 1, 0], [-0.70710678, 0, -0.70710678]]
        ),
        np.array(
            [[-0.80114362, 0, 0.59847214], [0, 1, 0], [-0.59847214, 0, -0.80114362]]
        ),
    ]

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = rot_b_to_s(**input_it)

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (1e-8 < np.abs(calculated_output - output_it)).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def rot_s_to_w_test() -> None:
    """
    test rot_s_to_w function
    """
    print("Starting rot_s_to_w test\n")
    # Inputs
    inputs = [{"beta": (3 * np.pi) / 4}, {"beta": 2.5}]
    # Expected outputs
    outputs = [
        np.array(
            [[-0.70710678, 0.70710678, 0], [-0.70710678, -0.70710678, 0], [0, 0, 1]]
        ),
        np.array(
            [[-0.80114362, 0.59847214, 0], [-0.59847214, -0.80114362, 0], [0, 0, 1]]
        ),
    ]

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = rot_s_to_w(**input_it)

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (1e-8 < np.abs(calculated_output - output_it)).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def rot_v_to_b_test() -> None:
    """
    test rot_v_to_b function
    """
    print("Starting rot_v_to_b test\n")
    # Inputs
    inputs = [
        {"psi": (3 * np.pi) / 4, "theta": -(3 * np.pi) / 4, "phi": np.pi / 4},
        {"psi": 2.5, "theta": 3.5, "phi": 0.2},
    ]
    # Expected outputs
    outputs = [
        np.array(
            [
                [0.5, -0.5, 0.70710678],
                [-0.14644661, -0.85355339, -0.5],
                [0.85355339, 0.14644661, -0.5],
            ]
        ),
        np.array(
            [
                [0.7502363, -0.56044324, 0.35078323],
                [-0.53071095, -0.82688153, -0.18604522],
                [0.39432396, -0.04658662, -0.9177899],
            ]
        ),
    ]

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = rot_v_to_b(**input_it)

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (1e-8 < np.abs(calculated_output - output_it)).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def rot_b_to_v_test() -> None:
    """
    test rot_b_to_v function
    """
    print("Starting rot_b_to_v test\n")
    # Inputs
    inputs = [
        {"psi": (3 * np.pi) / 4, "theta": -(3 * np.pi) / 4, "phi": np.pi / 4},
        {"psi": 2.5, "theta": 3.5, "phi": 0.2},
    ]
    # Expected outputs
    outputs = [
        np.array(
            [
                [0.5, -0.14644661, 0.85355339],
                [-0.5, -0.85355339, 0.14644661],
                [0.70710678, -0.5, -0.5],
            ]
        ),
        np.array(
            [
                [0.7502363, -0.53071095, 0.39432396],
                [-0.56044324, -0.82688153, -0.04658662],
                [0.35078323, -0.18604522, -0.9177899],
            ]
        ),
    ]

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = rot_b_to_v(**input_it)

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (1e-8 < np.abs(calculated_output - output_it)).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def trans_i_to_v_test() -> None:
    """
    test trans_i_to_v function
    """
    print("Starting trans_i_to_v test\n")
    # Inputs
    class Pose:
        def __init__(self) -> None:
            self.north: float = 42.0
            self.east: float = 82.0
            self.altitude: float = 194.0
            self.phi: float = (3 * np.pi) / 4
            self.theta: float = -(3 * np.pi) / 4
            self.psi: float = np.pi / 4

    inputs = [{"pose": Pose(), "p_i": np.array([[15], [98], [48]])}]
    # Expected outputs
    outputs = [np.array([[-27], [16], [242]])]

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = trans_i_to_v(**input_it)  # type: ignore

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (1e-8 < np.abs(calculated_output - output_it)).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def trans_v_to_i_test() -> None:
    """
    test trans_v_to_i function
    """
    print("Starting trans_v_to_i test\n")
    # Inputs
    class Pose:
        def __init__(self) -> None:
            self.north: float = 42.0
            self.east: float = 82.0
            self.altitude: float = 194.0
            self.phi: float = (3 * np.pi) / 4
            self.theta: float = -(3 * np.pi) / 4
            self.psi: float = np.pi / 4

    inputs = [{"pose": Pose(), "p_v": np.array([[15], [98], [48]])}]
    # Expected outputs
    outputs = [np.array([[57], [180], [-146]])]

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = trans_v_to_i(**input_it)  # type: ignore

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (1e-8 < np.abs(calculated_output - output_it)).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def trans_i_to_b_test() -> None:
    """
    test trans_i_to_b function
    """
    print("Starting trans_i_to_b test\n")
    # Inputs
    class Pose:
        def __init__(self) -> None:
            self.north: float = 42.0
            self.east: float = 82.0
            self.altitude: float = 194.0
            self.phi: float = (3 * np.pi) / 4
            self.theta: float = -(3 * np.pi) / 4
            self.psi: float = np.pi / 4

    inputs = [{"pose": Pose(), "p_i": np.array([[15], [98], [48]])}]
    # Expected outputs
    outputs = [np.array([[176.61984105], [-138.6109127], [95.6109127]])]

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = trans_i_to_b(**input_it)  # type: ignore

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (1e-8 < np.abs(calculated_output - output_it)).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def trans_b_to_i_test() -> None:
    """
    test trans_b_to_i function
    """
    print("Starting trans_b_to_i test\n")
    # Inputs
    class Pose:
        def __init__(self) -> None:
            self.north: float = 42.0
            self.east: float = 82.0
            self.altitude: float = 194.0
            self.phi: float = (3 * np.pi) / 4
            self.theta: float = -(3 * np.pi) / 4
            self.psi: float = np.pi / 4

    inputs = [{"pose": Pose(), "p_b": np.array([[15], [98], [48]])}]
    # Expected outputs
    outputs = [np.array([[89.82233047], [-16.17766953], [-208.39339828]])]

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = trans_b_to_i(**input_it)  # type: ignore

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (1e-8 < np.abs(calculated_output - output_it)).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def run_all_tests() -> None:
    """
    Run all tests
    """
    rot_x_test()
    rot_y_test()
    rot_z_test()
    rot_v_to_v1_test()
    rot_v1_to_v2_test()
    rot_v2_to_b_test()
    rot_b_to_s_test()
    rot_s_to_w_test()
    rot_v_to_b_test()
    rot_b_to_v_test()
    trans_i_to_v_test()
    trans_v_to_i_test()
    trans_i_to_b_test()
    trans_b_to_i_test()


if __name__ == "__main__":
    run_all_tests()

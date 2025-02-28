import numpy as np
from typing import Callable, List, Union
import inspect


class LabTester:
    """
    A test helper class for validating 2D/3D rotation and transformation implementations
    """

    def __init__(self):
        self.passed_tests = 0
        self.total_tests = 0

    def assert_array_almost_equal(
        self,
        actual: np.ndarray,
        expected: np.ndarray,
        decimal: int = 6,
        message: str = "",
    ):
        """Custom assertion for numpy arrays with detailed output"""
        self.total_tests += 1
        try:
            np.testing.assert_array_almost_equal(actual, expected, decimal=decimal)
            print(f"✅ {message}")
            self.passed_tests += 1
        except AssertionError as e:
            print(f"❌ {message}")
            print(f"   Expected: {expected}")
            print(f"   Got: {actual}")


    def test_rotation_three_angle(self, rotation_three_angle: Callable):
        print("\n=== Testing Three Angle Rotation Implementation ===")


        point = np.array([1, 1, 1])

        # Test-00: No rotation
        alpha = gamma = theta = 0.0
        rotation = rotation_three_angle(alpha, gamma, theta, point)
        self.assert_array_almost_equal(
            rotation, [1, 1, 1], decimal=3, message="No Rotation"
        )

        # Test-01: 45-degree rotation
        alpha = gamma = theta = 45.0
        rotation = rotation_three_angle(alpha, gamma, theta, point)
        self.assert_array_almost_equal(
            rotation, [1.707, -0.207, 0.207], decimal=3, message="45 Degree Rotation"
        )

        # Test-02: Full rotation (360 degrees)
        alpha = gamma = theta = 360.0
        rotation = rotation_three_angle(alpha, gamma, theta, point)
        self.assert_array_almost_equal(
            rotation, [1, 1, 1], decimal=3, message="Full Rotation"
        )

        # Test-03: Negative rotation (-45 degrees)
        alpha = gamma = theta = -45.0
        rotation = rotation_three_angle(alpha, gamma, theta, point)
        self.assert_array_almost_equal(
            rotation, [0.707, 1.5, -0.5 ], decimal=3, message="Negative 45 Degree Rotation"
        )

    def test_rotation_two_vector(self, rotation_two_vector: Callable):
        print("\n=== Testing Two Vector Rotation Implementation ===")

        point = np.array([1, 1, 1])


        # Test: Rotation about the x-axis (90-degree rotation)
        a = np.array([0, 1, 0])   # Initial y-axis
        o = np.array([0, 0, 1])   # Target z-axis
        expected_rotation = np.array([-1, 1, 1])
        rotation = rotation_two_vector(a, o, point)
        self.assert_array_almost_equal(rotation, expected_rotation, decimal=3, message="X-Axis Rotation")

        # Test: Rotation about the Y-axis (90-degree rotation)
        a = np.array([0, 0, 1])   # Initial z-axis
        o = np.array([1, 0, 0])   # Target x-axis
        expected_rotation = np.array([1, -1, 1])
        rotation = rotation_two_vector(a, o, point)
        self.assert_array_almost_equal(rotation, expected_rotation, decimal=3, message="Y-Axis Rotation")

        # Test: Rotate around the z-axis (90-degree rotation)
        a = np.array([1, 0, 0])   # Initial x-axis
        o = np.array([0, 1, 0])   # Target y-axis
        expected_rotation = np.array([1, 1, -1])
        rotation = rotation_two_vector(a, o, point)
        self.assert_array_almost_equal(rotation, expected_rotation, decimal=3, message="Z-Axis Rotation")

        # Edge Case: No rotation (a_v == o_v)
        # a = np.array([1, 1, 1])
        # o = np.array([1, 1, 1])
        # try:
        #     rotation = rotation_two_vector(a, o, point)
        #     print("❌ No Rotation Case: Expected ValueError but got no error")
        # except ValueError:
        #     print("✅ No Rotation Case: Correctly raised ValueError")

        # # Test: 180-degree rotation around the x-axis
        # a = np.array([1, 0, 0])   # Initial x-axis
        # o = np.array([-1, 0, 0])  # Target -x-axis
        # try:
        #     rotation = rotation_two_vector(a, o, point)
        #     print("❌ Anti Parallel Case: Expected ValueError but got no error")
        # except ValueError:
        #     print("✅ Anti Parallel Case: Correctly raised ValueError")

        # Additional Test: 45-degree rotation around the z-axis
        a = np.array([1, 0, 0])   # Initial x-axis
        o = np.array([np.sqrt(2)/2, np.sqrt(2)/2, 0])  # Target 45-degree in xy-plane
        expected_rotation = np.array([1, 1, -1])  # Correct expected output
        rotation = rotation_two_vector(a, o, point)
        self.assert_array_almost_equal(rotation, expected_rotation, decimal=3, message="45-degree Z Rotation")

    def test_rotation_eigen_vector(self, rotation_eigen_vector: Callable):
        print("\n=== Testing Eigen Vector Rotation Implementation ===")

        # Existing Test: No rotation
        point = np.array([1, 1, 1])
        neta = np.array([0, 0, 1])
        theta = 0.0
        rotation = rotation_eigen_vector(neta, theta, point)
        self.assert_array_almost_equal(rotation, [1, 1, 1], decimal=3, message="No Rotation")

        # Test: 180-degree rotation about the z-axis
        theta = 180.0
        rotation = rotation_eigen_vector(neta, theta, point)
        self.assert_array_almost_equal(rotation, [-1, -1, 1], decimal=3, message="180-degree Z Rotation")

        # Test: 90-degree rotation about the x-axis
        neta = np.array([1, 0, 0])
        theta = 90.0
        rotation = rotation_eigen_vector(neta, theta, point)
        self.assert_array_almost_equal(rotation, [1, -1, 1], decimal=3, message="90-degree X Rotation")

        # Test: 45-degree rotation about the y-axis
        neta = np.array([0, 1, 0])
        theta = 45.0
        rotation = rotation_eigen_vector(neta, theta, point)
        self.assert_array_almost_equal(rotation, [1.414, 1, 0], decimal=3, message="45-degree Y Rotation")

        # Test: Negative rotation (-90 degrees) about the z-axis
        neta = np.array([0, 0, 1])
        theta = -90.0
        rotation = rotation_eigen_vector(neta, theta, point)
        self.assert_array_almost_equal(rotation, [1, -1, 1], decimal=3, message="Negative 90-degree Z Rotation")

    def test_rotation_unit_quaternion(self, rotation_unit_quaternion: Callable):
        print("\n=== Testing Unit Quaternion Rotation Implementation ===")

        # Existing Test: No rotation
        point = np.array([1, 1, 1])
        q = np.array([1, 0, 0, 0])
        rotation = rotation_unit_quaternion(q, point)
        self.assert_array_almost_equal(rotation, [1, 1, 1], decimal=3, message="No Rotation")

        # New Test: Rotation with a normalized quaternion
        q = np.array([0, 1, 0, 0])
        rotation = rotation_unit_quaternion(q, point)
        self.assert_array_almost_equal(rotation, [1, -1, -1], decimal=3, message="90-degree X Rotation")


        # Additional Test: 90-degree rotation around the y-axis
        q = np.array([0, 0, 1, 0])
        point = np.array([1, 1, 1])
        expected_rotation = np.array([-1, 1, -1])
        rotation = rotation_unit_quaternion(q, point)
        self.assert_array_almost_equal(rotation, expected_rotation, decimal=3, message="180-degree Y Rotation")

        # Additional Test: 45-degree rotation around the z-axis
        q = np.array([0.9239, 0.0000,  0.0000,  0.3827])
        point = np.array([1, 1, 1])
        expected_rotation = np.array([0, 1.414, 1])
        rotation = rotation_unit_quaternion(q, point)
        self.assert_array_almost_equal(rotation, expected_rotation, decimal=3, message="90-degree Z Rotation")

        # Additional Test: Negative 90-degree rotation around the x-axis
        q = np.array([0, -1, 0, 0])
        point = np.array([1, 1, 1])
        expected_rotation = np.array([1, -1, -1])
        rotation = rotation_unit_quaternion(q, point)
        self.assert_array_almost_equal(rotation, expected_rotation, decimal=3, message="Negative 180-degree X Rotation")

    def test_rotation_matrix_exp(self, rotation_matrix_exp: Callable):
        print("\n=== Testing Matrix Exponential Rotation Implementation ===")

        point = np.array([1, 1, 1])

        # Test: No rotation
        w_x = np.array([1, 1, 1])
        theta = 0.0
        rotation = rotation_matrix_exp(w_x, theta, point)
        self.assert_array_almost_equal(rotation, [1, 1, 1], decimal=3, message="No Rotation")

        # Test: Rotation along x axis
        w_x = np.array([1, 0, 0])
        theta = 90.0
        rotation = rotation_matrix_exp(w_x, theta, point)
        self.assert_array_almost_equal(rotation, [1, -1, 1], decimal=3, message="90-degree X-axis Rotation")

        # Test: Rotation along y axis
        w_x = np.array([0, 1, 0])
        theta = 90.0
        rotation = rotation_matrix_exp(w_x, theta, point)
        self.assert_array_almost_equal(rotation, [1, 1, -1], decimal=3, message="90-degree Y-axis Rotation")

        # Test: Rotation along z axis
        w_x = np.array([0, 0, 1])
        theta = 90.0
        rotation = rotation_matrix_exp(w_x, theta, point)
        self.assert_array_almost_equal(rotation, [-1, 1, 1], decimal=3, message="90-degree Z-axis Rotation")

        # Test: 180-degree rotation along x axis
        w_x = np.array([1, 0, 0])
        theta = 180.0
        rotation = rotation_matrix_exp(w_x, theta, point)
        self.assert_array_almost_equal(rotation, [1, -1, -1], decimal=3, message="180-degree X-axis Rotation")

        # Test: 180-degree rotation along y axis
        w_x = np.array([0, 1, 0])
        theta = 180.0
        rotation = rotation_matrix_exp(w_x, theta, point)
        self.assert_array_almost_equal(rotation, [-1, 1, -1], decimal=3, message="180-degree Y-axis Rotation")

        # Test: 180-degree rotation along z axis
        w_x = np.array([0, 0, 1])
        theta = 180.0
        rotation = rotation_matrix_exp(w_x, theta, point)
        self.assert_array_almost_equal(rotation, [-1, -1, 1], decimal=3, message="180-degree Z-axis Rotation")

        # Test: 45-degree rotation along x axis
        w_x = np.array([1, 0, 0])
        theta = 45.0
        rotation = rotation_matrix_exp(w_x, theta, point)
        self.assert_array_almost_equal(rotation, [1, 0,1.414], decimal=3, message="45-degree X-axis Rotation")



    def print_summary(self):
        """Print summary of test results"""
        print(f"\n=== Test Summary ===")
        print(f"Passed: {self.passed_tests}/{self.total_tests} tests")
        if self.passed_tests == self.total_tests:
            print("🎉 All tests passed!")
        else:
            print(f"❌ {self.total_tests - self.passed_tests} tests failed")
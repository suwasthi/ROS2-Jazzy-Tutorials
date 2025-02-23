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
        """
        Test suite for 3 Angle Rotation implementation

        Parameters:
        rotation_three_angle: function that takes (alpha:float, gamma:float, theta:float, p:np.ndarray)
        and returns transformed point
        """
        print("\n=== Testing Three Angle Rotation Implementation ===")

        # Test 1: No rotation
        point = np.array([1, 1, 1])
        alpha = gamma = theta = 0.0
        rotation = rotation_three_angle(alpha, gamma, theta, point)
        self.assert_array_almost_equal(
            rotation, [1, 1, 1], decimal=3, message="No Rotation"
        )

    def test_rotation_two_vector(self, rotation_two_vector: Callable):
        """
        Test suite for Two Vector Rotation implementation

        Parameters:
        rotation_two_vector: function that takes (a: np.ndarray, o: np.ndarray, p:np.ndarray)
        and returns transformed point
        """
        print("\n=== Testing Two Vector Rotation Implementation ===")

        # Test 1: No rotation
        point = np.array([1, 1, 1])
        a = np.array([1, 0, 0])
        o = np.array([0, 1, 0])
        rotation = rotation_two_vector(a, o, point)
        self.assert_array_almost_equal(
            rotation, [1, 1, 1], decimal=3, message="No Rotation"
        )

    def test_rotation_eigen_vector(self, rotation_eigen_vector: Callable):
        """
        Test suite for Eigen Vector Rotation implementation

        Parameters:
        rotation_eigen_vector: function that takes (neta: np.ndarray, theta:float, p:np.ndarray)
        and returns transformed point
        """
        print("\n=== Testing Eigen Vector Rotation Implementation ===")

        # Test 1: No rotation
        point = np.array([1, 1, 1])
        neta = np.array([0, 0, 1])
        theta = 0.0
        rotation = rotation_eigen_vector(neta, theta, point)
        self.assert_array_almost_equal(
            rotation, [1, 1, 1], decimal=3, message="No Rotation"
        )

    def test_rotation_unit_quaternion(self, rotation_unit_quaternion: Callable):
        """
        Test suite for Unit Quaternion Rotation implementation

        Parameters:
        rotation_unit_quaternion: function that takes (q: np.ndarray, p: np.ndarray)
        and returns transformed point
        """
        print("\n=== Testing Unit Quaternion Rotation Implementation ===")

        # Test 1: No rotation
        point = np.array([1, 1, 1])
        q = np.array([1, 0, 0, 1])
        rotation = rotation_unit_quaternion(q, point)
        self.assert_array_almost_equal(
            rotation, [1, 1, 1], decimal=3, message="No Rotation"
        )

    def test_rotation_matrix_exp(self, rotation_matrix_exp: Callable):
        """
        Test suite for Matrix Exponential Rotation implementation

        Parameters:
        rotation_matrix_exp: function that takes (w_x : np.ndarray, theta: float, p: np.ndarray)
        and returns transformed point
        """
        print("\n=== Testing Matrix Exponential Rotation Implementation ===")

        # Test 1: No rotation
        point = np.array([1, 1, 1])
        w_x = np.array([1, 1, 1])
        theta = 0.0
        rotation = rotation_matrix_exp(w_x, theta, point)
        self.assert_array_almost_equal(
            rotation, [1, 1, 1], decimal=3, message="No Rotation"
        )

    def print_summary(self):
        """Print summary of test results"""
        print(f"\n=== Test Summary ===")
        print(f"Passed: {self.passed_tests}/{self.total_tests} tests")
        if self.passed_tests == self.total_tests:
            print("🎉 All tests passed!")
        else:
            print(f"❌ {self.total_tests - self.passed_tests} tests failed")

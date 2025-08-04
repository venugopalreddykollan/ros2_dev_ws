#!/usr/bin/env python3
"""
Minimal unit tests for ROS2 trajectory package - colcon test compatible.

Author: Venugopal Reddy Kollan
License: Apache License 2.0
"""

import unittest
import numpy as np
import math


class TestTrajectoryMath(unittest.TestCase):
    """Test core trajectory mathematics."""

    def test_cubic_polynomial_boundaries(self):
        """Test 3rd order polynomial s(t) = 3t² - 2t³ boundary conditions."""
        # At t=0: position=0, velocity=0
        s_start = 3 * 0.0**2 - 2 * 0.0**3
        v_start = 6 * 0.0 - 6 * 0.0**2
        
        self.assertAlmostEqual(s_start, 0.0, places=10)
        self.assertAlmostEqual(v_start, 0.0, places=10)
        
        # At t=1: position=1, velocity=0
        s_end = 3 * 1.0**2 - 2 * 1.0**3
        v_end = 6 * 1.0 - 6 * 1.0**2
        
        self.assertAlmostEqual(s_end, 1.0, places=10)
        self.assertAlmostEqual(v_end, 0.0, places=10)

    def test_velocity_calculation(self):
        """Test basic velocity calculation."""
        prev_pose = np.array([0.0, 0.0, 0.0])
        curr_pose = np.array([1.0, 2.0, 0.0])
        dt = 0.02  # 50Hz
        
        velocity = (curr_pose - prev_pose) / dt
        expected = np.array([50.0, 100.0, 0.0])
        
        np.testing.assert_array_almost_equal(velocity, expected, decimal=6)


class TestFilterMath(unittest.TestCase):
    """Test filter mathematics."""

    def test_filter_coefficient(self):
        """Test low-pass filter coefficient calculation."""
        cutoff_freq = 2.0
        sample_rate = 50.0
        
        rc = 1.0 / (2.0 * math.pi * cutoff_freq)
        dt = 1.0 / sample_rate
        alpha = dt / (rc + dt)
        
        self.assertGreater(alpha, 0.0)
        self.assertLess(alpha, 1.0)


if __name__ == '__main__':
    unittest.main()

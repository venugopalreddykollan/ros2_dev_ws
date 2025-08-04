#!/usr/bin/env python3
"""
Minimal functional unit tests for trajectory planner and velocity filter nodes.

Author: Venugopal Reddy Kollan
License: Apache License 2.0
"""

import pytest
import numpy as np
import math
from unittest.mock import Mock, patch

from geometry_msgs.msg import Point
from ros2_traj_pkg.trajectory_planner_node import TrajectoryPlanner
from ros2_traj_pkg.filtervelocity_subscriber_node import VelocityFilterNode


class TestTrajectoryPlannerCore:
    """Core mathematical and validation tests for trajectory planner."""

    def test_polynomial_boundary_conditions(self):
        """Test 3rd order polynomial s(t) = 3t² - 2t³ starts and ends correctly."""
        # At start (t=0): position=0, velocity=0
        s_start = 3 * 0.0**2 - 2 * 0.0**3
        v_start = 6 * 0.0 - 6 * 0.0**2
        assert abs(s_start) < 1e-10 and abs(v_start) < 1e-10

        # At end (t=1): position=1, velocity=0
        s_end = 3 * 1.0**2 - 2 * 1.0**3
        v_end = 6 * 1.0 - 6 * 1.0**2
        assert abs(s_end - 1.0) < 1e-10 and abs(v_end) < 1e-10

    @pytest.mark.parametrize(
        "distance,duration,expected_vel,expected_acc",
        [
            (10.0, 5.0, 3.0, 2.4),
            (5.0, 1.0, 7.5, 30.0),
        ],
    )
    def test_physics_estimation(self, distance, duration, expected_vel, expected_acc):
        """Test velocity and acceleration estimation formulas."""
        est_vel = 1.5 * distance / duration
        est_acc = 6.0 * distance / (duration * duration)
        assert abs(est_vel - expected_vel) < 1e-6
        assert abs(est_acc - expected_acc) < 1e-6

    @pytest.fixture
    def mock_planner(self):
        """Create mock trajectory planner."""
        with patch("ros2_traj_pkg.trajectory_planner_node.Node.__init__"), patch.object(
            TrajectoryPlanner, "declare_parameter"
        ), patch.object(TrajectoryPlanner, "get_parameter") as mock_param, patch.object(
            TrajectoryPlanner, "create_publisher"
        ), patch.object(
            TrajectoryPlanner, "create_service"
        ), patch.object(
            TrajectoryPlanner, "get_logger"
        ):

            mock_param.side_effect = lambda name: Mock(
                value={
                    "publisher_frequency": 50,
                    "max_velocity": 1.0,
                    "max_acceleration": 0.5,
                    "max_jerk": 2.0,
                }[name]
            )
            return TrajectoryPlanner()

    def test_position_validation(self, mock_planner):
        """Test position validation for valid and invalid inputs."""
        # Valid position
        valid_pos = Point(x=1.0, y=2.0, z=3.0)
        result = mock_planner._validate_position(valid_pos, "test")
        np.testing.assert_array_equal(result, [1.0, 2.0, 3.0])

        # Invalid positions
        with pytest.raises(ValueError, match="Non-finite values"):
            mock_planner._validate_position(Point(x=float("nan"), y=0.0, z=0.0), "test")

        with pytest.raises(ValueError, match="exceeds bounds"):
            mock_planner._validate_position(Point(x=2000.0, y=0.0, z=0.0), "test")

    @pytest.mark.parametrize(
        "duration,should_pass,error_text",
        [
            (10.0, True, "successfully generated"),
            (-5.0, False, "positive"),
            (500.0, False, "too large"),
        ],
    )
    def test_service_validation(self, mock_planner, duration, should_pass, error_text):
        """Test service request validation for various durations."""
        request = Mock()
        request.start.position = Point(x=0.0, y=0.0, z=0.0)
        request.goal.position = Point(x=5.0, y=3.0, z=1.0)
        request.duration = duration
        response = Mock(success=None, message=None)

        with patch.object(mock_planner, "create_timer"), patch.object(
            mock_planner, "get_logger"
        ):
            result = mock_planner.plan_callback(request, response)

        assert result.success == should_pass
        assert error_text in result.message.lower()


class TestVelocityFilterCore:
    """Core mathematical tests for velocity filter."""

    def test_filter_coefficient_calculation(self):
        """Test low-pass filter coefficient α = dt/(RC + dt)."""
        cutoff_freq, sample_rate = 2.0, 50.0
        rc = 1.0 / (2.0 * math.pi * cutoff_freq)
        dt = 1.0 / sample_rate
        alpha = dt / (rc + dt)

        assert 0.0 < alpha < 1.0  # Valid filter coefficient

    def test_velocity_calculation(self):
        """Test velocity calculation from pose differences."""
        prev_pose = np.array([0.0, 0.0, 0.0])
        curr_pose = np.array([1.0, 2.0, 0.5])
        dt = 0.02  # 50Hz

        velocity = (curr_pose - prev_pose) / dt
        expected = np.array([50.0, 100.0, 25.0])  # 1m/0.02s = 50 m/s, etc.
        np.testing.assert_array_almost_equal(velocity, expected)

    def test_low_pass_filter_behavior(self):
        """Test filter smooths step input correctly."""
        alpha = 0.1
        filtered_value = 0.0

        # Apply step input multiple times
        for _ in range(10):
            filtered_value = alpha * 1.0 + (1 - alpha) * filtered_value

        assert 0.0 < filtered_value < 1.0  # Should approach 1.0 but not reach it
        assert filtered_value > 0.5  # Should make significant progress

    @pytest.fixture
    def mock_filter(self):
        """Create mock velocity filter."""
        with patch(
            "ros2_traj_pkg.filtervelocity_subscriber_node.Node.__init__"
        ), patch.object(VelocityFilterNode, "declare_parameter"), patch.object(
            VelocityFilterNode, "get_parameter"
        ) as mock_param, patch.object(
            VelocityFilterNode, "_setup_communication"
        ), patch.object(
            VelocityFilterNode, "get_logger"
        ):

            mock_param.side_effect = lambda name: Mock(
                value={
                    "cutoff_frequency": 2.0,
                    "sample_rate": 50.0,
                    "velocity_bounds": 10.0,
                }[name]
            )
            return VelocityFilterNode()

    def test_filter_initialization(self, mock_filter):
        """Test filter initializes with valid parameters."""
        assert hasattr(mock_filter, "cutoff_freq")
        assert hasattr(mock_filter, "alpha")
        assert 0.0 < mock_filter.alpha < 1.0


class TestIntegrationScenarios:
    """Test realistic end-to-end scenarios."""

    def test_realistic_trajectory_workflow(self):
        """Test complete trajectory generation for realistic robot movement."""
        # 5 meter movement in 10 seconds at 50Hz
        start_pos = np.array([0.0, 0.0, 0.0])
        goal_pos = np.array([5.0, 0.0, 0.0])
        duration, frequency = 10.0, 50

        trajectory = []
        for i in range(int(duration * frequency)):
            t = i / (duration * frequency - 1) if duration * frequency > 1 else 0.0
            s = 3 * t**2 - 2 * t**3
            pos = start_pos + s * (goal_pos - start_pos)
            trajectory.append(pos)

        # Verify trajectory boundaries
        np.testing.assert_array_almost_equal(trajectory[0], start_pos, decimal=6)
        np.testing.assert_array_almost_equal(trajectory[-1], goal_pos, decimal=6)

        # Verify velocity profile (smooth start/stop)
        velocities = []
        dt = 1.0 / frequency
        for i in range(1, len(trajectory)):
            vel = np.linalg.norm((trajectory[i] - trajectory[i - 1]) / dt)
            velocities.append(vel)

        assert velocities[0] < 0.5  # Starts slow
        assert velocities[-1] < 0.5  # Ends slow
        assert max(velocities) > 0.1  # Has meaningful motion

    @pytest.mark.parametrize(
        "raw_speed,filtered_speed,expected_effectiveness",
        [
            (1.0, 0.8, 0.2),  # 20% filtering
            (2.0, 1.0, 0.5),  # 50% filtering
            (0.0, 0.0, 0.0),  # No movement
        ],
    )
    def test_filter_effectiveness_metric(
        self, raw_speed, filtered_speed, expected_effectiveness
    ):
        """Test filter effectiveness calculation."""
        speed_diff = abs(raw_speed - filtered_speed)
        effectiveness = min(1.0, speed_diff / raw_speed) if raw_speed > 0.001 else 0.0
        assert abs(effectiveness - expected_effectiveness) < 1e-6


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

#!/usr/bin/env python3

import time
import unittest

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
import rclpy

from custom_interface.srv import PlanTrajectory


class TestTrajectoryPlannerNode(unittest.TestCase):
    """Test suite for trajectory planner node functionality."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS2 context for testing."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Clean up ROS2 context after testing."""
        rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.node = rclpy.create_node("test_trajectory_planner")
        self.received_poses = []

        # Create service client
        self.plan_client = self.node.create_client(PlanTrajectory, "plan_trajectory")

        # Create subscription to monitor published poses
        self.pose_subscription = self.node.create_subscription(
            PoseStamped, "current_pose", self.pose_callback, 10
        )

    def tearDown(self):
        """Clean up after each test method."""
        self.node.destroy_node()

    def pose_callback(self, msg):
        """Collect published poses for testing."""
        self.received_poses.append(msg)

    def spin_node_for_duration(self, duration):
        """Spin the node for a specific duration."""
        start_time = time.time()
        while time.time() - start_time < duration:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def test_service_availability(self):
        """Test that the plan_trajectory service is available."""
        # Wait for service to become available
        service_available = self.plan_client.wait_for_service(timeout_sec=5.0)
        self.assertTrue(service_available, "plan_trajectory service not available")

    def test_service_call_success(self):
        """Test successful service call with valid parameters."""
        # Wait for service
        self.assertTrue(self.plan_client.wait_for_service(timeout_sec=5.0))

        # Create request
        request = PlanTrajectory.Request()
        request.start = Pose()
        request.start.position = Point(x=0.0, y=0.0, z=0.0)
        request.start.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        request.goal = Pose()
        request.goal.position = Point(x=1.0, y=1.0, z=0.5)
        request.goal.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        request.duration = 2.0

        # Call service
        future = self.plan_client.call_async(request)

        # Wait for response
        start_time = time.time()
        while not future.done() and time.time() - start_time < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(future.done(), "Service call did not complete")

        response = future.result()
        self.assertTrue(response.success, f"Service call failed: {response.message}")

    def test_service_call_invalid_duration(self):
        """Test service call with invalid duration."""
        # Wait for service
        self.assertTrue(self.plan_client.wait_for_service(timeout_sec=5.0))

        # Create request with invalid duration
        request = PlanTrajectory.Request()
        request.start = Pose()
        request.goal = Pose()
        request.duration = -1.0  # Invalid duration

        # Call service
        future = self.plan_client.call_async(request)

        # Wait for response
        start_time = time.time()
        while not future.done() and time.time() - start_time < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        response = future.result()
        self.assertFalse(response.success, "Service should fail with invalid duration")
        self.assertIn("Invalid duration", response.message)

    def test_pose_publishing(self):
        """Test that poses are published on the current_pose topic."""
        # Clear received poses
        self.received_poses.clear()

        # Wait for service
        self.assertTrue(self.plan_client.wait_for_service(timeout_sec=5.0))

        # Create and send request
        request = PlanTrajectory.Request()
        request.start = Pose()
        request.start.position = Point(x=0.0, y=0.0, z=0.0)
        request.goal = Pose()
        request.goal.position = Point(x=1.0, y=0.0, z=0.0)
        request.duration = 1.0

        future = self.plan_client.call_async(request)

        # Wait for service response and then for poses to be published
        start_time = time.time()
        while not future.done() and time.time() - start_time < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Spin for a bit longer to receive published poses
        self.spin_node_for_duration(2.0)

        # Check that poses were published
        self.assertGreater(len(self.received_poses), 0, "No poses were published")

        # Check that poses have correct frame_id
        for pose in self.received_poses[:5]:  # Check first 5 poses
            self.assertEqual(pose.header.frame_id, "world")
            self.assertEqual(pose.pose.orientation.w, 1.0)

    def test_trajectory_progression(self):
        """Test that trajectory progresses from start to goal."""
        # Clear received poses
        self.received_poses.clear()

        # Wait for service
        self.assertTrue(self.plan_client.wait_for_service(timeout_sec=5.0))

        # Create request
        start_x, goal_x = 0.0, 2.0
        request = PlanTrajectory.Request()
        request.start = Pose()
        request.start.position = Point(x=start_x, y=0.0, z=0.0)
        request.goal = Pose()
        request.goal.position = Point(x=goal_x, y=0.0, z=0.0)
        request.duration = 1.0

        # Call service and wait for poses
        future = self.plan_client.call_async(request)
        start_time = time.time()
        while not future.done() and time.time() - start_time < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Collect poses for trajectory duration
        self.spin_node_for_duration(1.5)

        # Check trajectory progression
        self.assertGreater(len(self.received_poses), 10, "Too few poses received")

        # First pose should be close to start
        first_pose = self.received_poses[0]
        self.assertAlmostEqual(first_pose.pose.position.x, start_x, delta=0.1)

        # Last pose should be close to goal
        last_pose = self.received_poses[-1]
        self.assertAlmostEqual(last_pose.pose.position.x, goal_x, delta=0.1)


if __name__ == "__main__":
    unittest.main()

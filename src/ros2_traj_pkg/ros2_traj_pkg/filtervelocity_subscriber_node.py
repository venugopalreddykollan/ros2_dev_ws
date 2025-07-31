#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import Float64
import numpy as np
import math
from collections import deque


class VelocityFilterNode(Node):
    """
    ROS2 node that subscribes to pose topic, computes velocity,
    and publishes both raw and filtered velocity.
    """

    def __init__(self):
        super().__init__("velocity_filter_node")

        # Declare parameters
        self.declare_parameter("cutoff_frequency", 2.0)  # Hz
        self.declare_parameter("sample_rate", 50.0)  # Hz (should match trajectory publisher)

        # Get parameters
        self.cutoff_freq = self.get_parameter("cutoff_frequency").value
        self.sample_rate = self.get_parameter("sample_rate").value

        # Calculate filter coefficient for simple low-pass filter
        # RC = 1/(2*pi*fc), alpha = dt/(RC + dt)
        rc = 1.0 / (2.0 * math.pi * self.cutoff_freq)
        dt = 1.0 / self.sample_rate
        self.alpha = dt / (rc + dt)

        self.get_logger().info(f"Low-pass filter: cutoff={self.cutoff_freq} Hz, alpha={self.alpha:.4f}")

        # Subscriber to pose topic
        self.pose_subscriber = self.create_subscription(
            PoseStamped, "current_pose", self.pose_callback, 10
        )

        # Publishers for velocities
        self.raw_velocity_publisher = self.create_publisher(Vector3Stamped, "raw_velocity", 10)
        self.filtered_velocity_publisher = self.create_publisher(
            Vector3Stamped, "filtered_velocity", 10
        )
        
        # Publishers for velocity magnitudes (better for rqt_plot visualization)
        self.raw_speed_publisher = self.create_publisher(Float64, "raw_speed", 10)
        self.filtered_speed_publisher = self.create_publisher(Float64, "filtered_speed", 10)

        # State variables
        self.previous_pose = None
        self.previous_time = None
        self.filtered_velocity = np.array([0.0, 0.0, 0.0])
        self.pose_history = deque(maxlen=2)  # Store last 2 poses for velocity calculation

        self.get_logger().info("Velocity Filter Node initialized.")

    def pose_callback(self, msg):
        """
        Process received pose and compute velocities.

        Args:
            msg (PoseStamped): The received pose message
        """
        current_time = self.get_clock().now()
        current_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        # Store pose with timestamp
        self.pose_history.append((current_pose, current_time))

        # Log received pose with timestamp
        self.get_logger().info(
            f"Received pose at {current_time.nanoseconds/1e9:.3f}s: "
            f"[{current_pose[0]:.3f}, {current_pose[1]:.3f}, {current_pose[2]:.3f}]"
        )

        # Need at least 2 poses to compute velocity
        if len(self.pose_history) >= 2:
            self.compute_and_publish_velocity()

    def compute_and_publish_velocity(self):
        """Compute velocity from consecutive poses and publish both raw and filtered velocities."""
        # Get the two most recent poses
        (prev_pose, prev_time), (curr_pose, curr_time) = self.pose_history[-2], self.pose_history[-1]

        # Calculate time difference
        dt = (curr_time - prev_time).nanoseconds / 1e9  # Convert to seconds

        if dt <= 0:
            self.get_logger().warn("Invalid time difference for velocity calculation")
            return

        # Compute raw velocity (euclidean difference / time)
        position_diff = curr_pose - prev_pose
        raw_velocity = position_diff / dt

        # Apply low-pass filter: v_filtered = alpha * v_raw + (1 - alpha) * v_filtered_prev
        self.filtered_velocity = self.alpha * raw_velocity + (1 - self.alpha) * self.filtered_velocity

        # Calculate velocity magnitudes (speeds)
        raw_speed = np.linalg.norm(raw_velocity)
        filtered_speed = np.linalg.norm(self.filtered_velocity)

        # Log velocity information including magnitudes
        self.get_logger().info(
            f"Raw velocity at {curr_time.nanoseconds/1e9:.3f}s: "
            f"[{raw_velocity[0]:.3f}, {raw_velocity[1]:.3f}, {raw_velocity[2]:.3f}] m/s | "
            f"Speed: {raw_speed:.3f} m/s"
        )
        
        self.get_logger().info(
            f"Filtered velocity at {curr_time.nanoseconds/1e9:.3f}s: "
            f"[{self.filtered_velocity[0]:.3f}, {self.filtered_velocity[1]:.3f}, {self.filtered_velocity[2]:.3f}] m/s | "
            f"Speed: {filtered_speed:.3f} m/s | "
            f"Speed Reduction: {((raw_speed - filtered_speed) / raw_speed * 100) if raw_speed > 0 else 0:.1f}%"
        )

        # Publish raw velocity
        raw_vel_msg = Vector3Stamped()
        raw_vel_msg.header.stamp = curr_time.to_msg()
        raw_vel_msg.header.frame_id = "world"
        raw_vel_msg.vector.x = raw_velocity[0]
        raw_vel_msg.vector.y = raw_velocity[1]
        raw_vel_msg.vector.z = raw_velocity[2]
        self.raw_velocity_publisher.publish(raw_vel_msg)

        # Publish filtered velocity
        filtered_vel_msg = Vector3Stamped()
        filtered_vel_msg.header.stamp = curr_time.to_msg()
        filtered_vel_msg.header.frame_id = "world"
        filtered_vel_msg.vector.x = self.filtered_velocity[0]
        filtered_vel_msg.vector.y = self.filtered_velocity[1]
        filtered_vel_msg.vector.z = self.filtered_velocity[2]
        self.filtered_velocity_publisher.publish(filtered_vel_msg)

        # Publish velocity magnitudes for rqt_plot visualization
        raw_speed_msg = Float64()
        raw_speed_msg.data = raw_speed
        self.raw_speed_publisher.publish(raw_speed_msg)

        filtered_speed_msg = Float64()
        filtered_speed_msg.data = filtered_speed
        self.filtered_speed_publisher.publish(filtered_speed_msg)


def main(args=None):
    """Main entry point for the velocity filter node."""
    rclpy.init(args=args)
    node = VelocityFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
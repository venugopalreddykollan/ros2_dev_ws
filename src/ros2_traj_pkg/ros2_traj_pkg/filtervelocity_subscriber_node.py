#!/usr/bin/env python3
"""
Velocity Filter Node for ROS2.

This module subscribes to pose data and publishes computed velocity vectors
with low-pass filtering for noise reduction and analysis.

Author: Venugopal Reddy Kollan
License: Apache License 2.0
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math
from collections import deque

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class VelocityFilterNode(Node):
    """
    ROS2 node for velocity computation and low-pass filtering from pose data.

    Subscribed Topics
    -----------------
    /current_pose (geometry_msgs/PoseStamped): Input pose data

    Published Topics
    ----------------
    /raw_velocity, /filtered_velocity (geometry_msgs/Vector3Stamped): Velocity vectors
    /raw_speed, /filtered_speed (std_msgs/Float64): Velocity magnitudes
    /velocity_comparison (geometry_msgs/Vector3Stamped): Velocity difference for analysis
    /speed_difference (std_msgs/Float64): Speed difference for filter effectiveness
    /filter_effectiveness (std_msgs/Float64): Filter performance metric (0-1)

    Parameters
    ----------
    cutoff_frequency (float): Filter cutoff frequency [Hz] (default: 2.0)
    sample_rate (float): Expected pose rate [Hz] (default: 50.0)
    velocity_bounds (float): Maximum expected velocity [m/s] (default: 10.0)

    """

    def __init__(self):
        """Initialize the velocity filter node with parameters and communication."""
        super().__init__("velocity_filter")

        # Declare parameters with validation
        self.declare_parameter("cutoff_frequency", 2.0)
        self.declare_parameter("sample_rate", 50.0)
        self.declare_parameter("velocity_bounds", 10.0)

        # Get parameters
        self.cutoff_freq = self.get_parameter("cutoff_frequency").value
        self.sample_rate = self.get_parameter("sample_rate").value
        self.velocity_bounds = self.get_parameter("velocity_bounds").value

        # Validate parameters
        if self.cutoff_freq <= 0.0 or self.cutoff_freq > self.sample_rate / 2:
            self.get_logger().error(
                f"Invalid cutoff frequency: {self.cutoff_freq}. "
                f"Must be > 0 and < {self.sample_rate/2} Hz"
            )
            return

        # Calculate filter coefficient: α = dt/(RC + dt)
        rc = 1.0 / (2.0 * math.pi * self.cutoff_freq)
        dt = 1.0 / self.sample_rate
        self.alpha = dt / (rc + dt)

        self.get_logger().info("Filter parameters loaded:")
        self.get_logger().info(f"Cutoff frequency: {self.cutoff_freq} Hz")
        self.get_logger().info(f"Sample rate: {self.sample_rate} Hz")
        self.get_logger().info(f"Velocity bounds: {self.velocity_bounds} m/s")
        self.get_logger().debug(
            f"Filter initialized - RC: {rc:.4f}, dt: {dt:.4f}, "
            f"alpha: {self.alpha:.4f}"
        )

        self._setup_communication()

        # Initialize state variables
        self.pose_history = deque(maxlen=2)
        self.filtered_velocity = np.array([0.0, 0.0, 0.0])
        self.max_raw_speed = 0.0
        self.max_filtered_speed = 0.0

        self.get_logger().info("Velocity Filter Node initialized.")

    def _setup_communication(self):
        """Set up publishers and subscribers with appropriate QoS."""
        # Configure QoS for velocity data
        vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        # Publishers for velocity vectors
        self.raw_velocity_pub = self.create_publisher(
            Vector3Stamped, "raw_velocity", vel_qos
        )
        self.filtered_velocity_pub = self.create_publisher(
            Vector3Stamped, "filtered_velocity", vel_qos
        )

        # Publishers for speed magnitudes
        self.raw_speed_pub = self.create_publisher(
            Float64, "raw_speed", vel_qos
        )
        self.filtered_speed_pub = self.create_publisher(
            Float64, "filtered_speed", vel_qos
        )
        self.velocity_comparison_pub = self.create_publisher(
            Vector3Stamped, "velocity_comparison", vel_qos
        )
        self.speed_difference_pub = self.create_publisher(
            Float64, "speed_difference", vel_qos
        )
        self.filter_effectiveness_pub = self.create_publisher(
            Float64, "filter_effectiveness", vel_qos
        )

        # Subscriber for pose data
        self.pose_sub = self.create_subscription(
            PoseStamped, "current_pose", self.pose_callback, vel_qos
        )

    def pose_callback(self, msg: PoseStamped) -> None:
        """Process incoming pose data and compute filtered velocities."""
        try:
            current_time = self.get_clock().now()
            current_pose = np.array([
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
            ])

            # Validate pose data
            if not np.all(np.isfinite(current_pose)):
                self.get_logger().warn(f"Invalid pose data: {current_pose}")
                return

            # Store pose with timestamp
            self.pose_history.append((current_pose, current_time))

            # Need at least 2 poses to compute velocity
            if len(self.pose_history) >= 2:
                self._compute_and_publish_velocity()

        except Exception as e:
            self.get_logger().error(f"Error in pose callback: {e}")

    def _compute_and_publish_velocity(self) -> None:
        """Compute raw and filtered velocities from pose history."""
        try:
            # Get the last two poses and timestamps
            (prev_pose, prev_time), (curr_pose, curr_time) = (
                self.pose_history[-2], self.pose_history[-1]
            )

            # Calculate time difference in seconds
            dt = (curr_time - prev_time).nanoseconds / 1e9
            if dt <= 0:
                return

            # Compute raw velocity
            raw_velocity = (curr_pose - prev_pose) / dt
            raw_speed = np.linalg.norm(raw_velocity)

            # Check for excessive velocities (likely noise)
            if raw_speed > self.velocity_bounds:
                self.get_logger().warn(
                    f"Raw velocity {raw_speed:.2f} m/s exceeds bounds "
                    f"{self.velocity_bounds} m/s - filtering"
                )

            # Apply low-pass filter: y[n] = α*x[n] + (1-α)*y[n-1]
            self.filtered_velocity = (
                self.alpha * raw_velocity
                + (1 - self.alpha) * self.filtered_velocity
            )
            filtered_speed = np.linalg.norm(self.filtered_velocity)

            # Update maximum speeds for statistics
            self.max_raw_speed = max(self.max_raw_speed, raw_speed)
            self.max_filtered_speed = max(self.max_filtered_speed, filtered_speed)

            if self.max_raw_speed > 0:
                self.get_logger().debug(
                    f"Max speeds: Raw={self.max_raw_speed:.3f}, "
                    f"Filtered={self.max_filtered_speed:.3f} m/s"
                )

            self._publish_velocity_data(
                raw_velocity, self.filtered_velocity, curr_time,
                raw_speed, filtered_speed
            )

        except Exception as e:
            self.get_logger().error(f"Error computing velocity: {e}")

    def _publish_velocity_data(self, raw_vel: np.ndarray, filt_vel: np.ndarray,
                               timestamp, raw_speed: float,
                               filt_speed: float) -> None:
        """Publish all velocity-related data and analysis metrics."""
        # Create timestamp
        stamp = timestamp.to_msg()

        # Publish raw velocity vector
        raw_vel_msg = Vector3Stamped()
        raw_vel_msg.header.stamp = stamp
        raw_vel_msg.header.frame_id = "map"
        raw_vel_msg.vector.x, raw_vel_msg.vector.y, raw_vel_msg.vector.z = raw_vel
        self.raw_velocity_pub.publish(raw_vel_msg)

        # Publish filtered velocity vector
        filt_vel_msg = Vector3Stamped()
        filt_vel_msg.header.stamp = stamp
        filt_vel_msg.header.frame_id = "map"
        filt_vel_msg.vector.x, filt_vel_msg.vector.y, filt_vel_msg.vector.z = filt_vel
        self.filtered_velocity_pub.publish(filt_vel_msg)

        # Publish speed magnitudes
        self.raw_speed_pub.publish(Float64(data=raw_speed))
        self.filtered_speed_pub.publish(Float64(data=filt_speed))

        # Publish velocity comparison (difference vector)
        vel_diff = raw_vel - filt_vel
        comparison_msg = Vector3Stamped()
        comparison_msg.header.stamp = stamp
        comparison_msg.header.frame_id = "map"
        comparison_msg.vector.x, comparison_msg.vector.y, comparison_msg.vector.z = vel_diff
        self.velocity_comparison_pub.publish(comparison_msg)

        # Publish speed difference and filter effectiveness
        speed_diff = abs(raw_speed - filt_speed)
        self.speed_difference_pub.publish(Float64(data=speed_diff))

        # Calculate filter effectiveness (0 = no filtering, 1 = maximum filtering)
        effectiveness = (
            min(1.0, speed_diff / raw_speed) if raw_speed > 0.001 else 0.0
        )
        self.filter_effectiveness_pub.publish(Float64(data=effectiveness))


def main(args=None) -> None:
    """Initialize and run the velocity filter node."""
    rclpy.init(args=args)
    node = VelocityFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

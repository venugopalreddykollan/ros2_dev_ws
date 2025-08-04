#!/usr/bin/env python3
# Uncomment all the lines below if you you wish not to use the generate_parameter_library
# and comment the generated parameters
# declare and get the parameters
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

# import for generated parameters
from ros2_traj_pkg.velocity_filter_parameters import velocity_filter


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
    """

    def __init__(self):
        """Initialize the velocity filter node with advanced parameter management."""
        super().__init__("velocity_filter")

        #  Initialize parameter listener for generate_parameter_library
        self.param_listener = velocity_filter.ParamListener(self)
        self.params = self.param_listener.get_params()

        #  Use parameters from generated module
        self.cutoff_freq = self.params.cutoff_frequency
        self.sample_rate = self.params.sample_rate
        self.velocity_bounds = self.params.velocity_bounds

        self.get_logger().info("Velocity Filter Parameter Management Initialized")
        self.get_logger().info(f"Cutoff frequency: {self.cutoff_freq} Hz")
        self.get_logger().info(f"Sample rate: {self.sample_rate} Hz")
        self.get_logger().info(f"Velocity bounds: {self.velocity_bounds} m/s")

        # Validate parameters
        if self.cutoff_freq <= 0.0 or self.cutoff_freq > self.sample_rate / 2:
            self.get_logger().error(
                f"Invalid cutoff frequency: {self.cutoff_freq}. "
                f"Must be > 0 and < {self.sample_rate/2} Hz"
            )
            return

        # Calculate filter coefficient
        self._update_filter_coefficient()

        # Initialize state variables
        self.pose_history = deque(maxlen=2)
        self.filtered_velocity = np.array([0.0, 0.0, 0.0])
        self.max_raw_speed = 0.0
        self.max_filtered_speed = 0.0

        # Setup communication
        self._setup_communication()

        self.get_logger().info(" Velocity Filter Node initialized successfully!")

    def _update_parameters_if_changed(self):
        """Update parameters if they changed at runtime."""
        if self.param_listener.is_old(self.params):
            old_cutoff = self.cutoff_freq
            self.params = self.param_listener.get_params()
            self.cutoff_freq = self.params.cutoff_frequency
            self.sample_rate = self.params.sample_rate
            self.velocity_bounds = self.params.velocity_bounds

            if abs(self.cutoff_freq - old_cutoff) > 0.01:
                self.get_logger().info(
                    f" Cutoff frequency updated: {self.cutoff_freq} Hz"
                )
                self._update_filter_coefficient()

            self.get_logger().info(" Filter parameters refreshed")

    def _update_filter_coefficient(self):
        """Update filter coefficient based on current parameters."""
        dt = 1.0 / self.sample_rate
        rc = 1.0 / (2.0 * math.pi * self.cutoff_freq)
        self.alpha = dt / (rc + dt)
        self.get_logger().info(f"Filter coefficient α = {self.alpha:.4f}")

    def _setup_communication(self):
        """Set up publishers and subscribers with appropriate QoS."""
        # Configure QoS for velocity data
        vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # Vector Publishers (for ROS ecosystem)
        self.raw_velocity_pub = self.create_publisher(
            Vector3Stamped, "raw_velocity", vel_qos
        )
        self.filtered_velocity_pub = self.create_publisher(
            Vector3Stamped, "filtered_velocity", vel_qos
        )
        self.velocity_comparison_pub = self.create_publisher(
            Vector3Stamped, "velocity_comparison", vel_qos
        )

        # Scalar Publishers for PlotJuggler (Task requirement for visualization)
        # Raw velocity components
        self.raw_velocity_x_pub = self.create_publisher(
            Float64, "raw_velocity_x", vel_qos
        )
        self.raw_velocity_y_pub = self.create_publisher(
            Float64, "raw_velocity_y", vel_qos
        )
        self.raw_velocity_z_pub = self.create_publisher(
            Float64, "raw_velocity_z", vel_qos
        )

        # Filtered velocity components
        self.filtered_velocity_x_pub = self.create_publisher(
            Float64, "filtered_velocity_x", vel_qos
        )
        self.filtered_velocity_y_pub = self.create_publisher(
            Float64, "filtered_velocity_y", vel_qos
        )
        self.filtered_velocity_z_pub = self.create_publisher(
            Float64, "filtered_velocity_z", vel_qos
        )

        # Speed and analysis metrics
        self.raw_speed_pub = self.create_publisher(Float64, "raw_speed", vel_qos)
        self.filtered_speed_pub = self.create_publisher(
            Float64, "filtered_speed", vel_qos
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
        """Process incoming pose data and compute filtered velocities with detailed logging."""
        try:
            # Update parameters if changed
            self._update_parameters_if_changed()

            current_time = self.get_clock().now()
            current_pose = np.array(
                [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            )

            # Log received pose data with timestamp (Task requirement)
            timestamp = current_time.nanoseconds / 1e9
            self.get_logger().info(
                f"[{timestamp:.6f}s] RECEIVED POSE: "
                f"x={current_pose[0]:.4f}, y={current_pose[1]:.4f}, z={current_pose[2]:.4f} m"
            )

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
        """Compute raw and filtered velocities from pose history with logging."""
        try:
            # Get the last two poses and timestamps
            (prev_pose, prev_time), (curr_pose, curr_time) = (
                self.pose_history[-2],
                self.pose_history[-1],
            )

            # Calculate time difference in seconds
            dt = (curr_time - prev_time).nanoseconds / 1e9
            if dt <= 0:
                return

            # Compute raw velocity
            raw_velocity = (curr_pose - prev_pose) / dt
            raw_speed = np.linalg.norm(raw_velocity)

            # Log computed raw velocity with timestamp (Task requirement)
            timestamp = curr_time.nanoseconds / 1e9
            self.get_logger().info(
                f"[{timestamp:.6f}s] RAW VELOCITY: "
                f"vx={raw_velocity[0]:.4f}, vy={raw_velocity[1]:.4f}, vz={raw_velocity[2]:.4f} m/s, "
                f"speed={raw_speed:.4f} m/s"
            )

            # Check for excessive velocities (likely noise)
            if raw_speed > self.velocity_bounds:
                self.get_logger().warn(
                    f"Raw velocity {raw_speed:.2f} m/s exceeds bounds "
                    f"{self.velocity_bounds} m/s - filtering"
                )

            # Apply low-pass filter: y[n] = α*x[n] + (1-α)*y[n-1]
            self.filtered_velocity = (
                self.alpha * raw_velocity + (1 - self.alpha) * self.filtered_velocity
            )
            filtered_speed = np.linalg.norm(self.filtered_velocity)

            # Log filtered velocity with timestamp (Task requirement)
            self.get_logger().info(
                f"[{timestamp:.6f}s] FILTERED VELOCITY: "
                f"vx={self.filtered_velocity[0]:.4f}, vy={self.filtered_velocity[1]:.4f}, "
                f"vz={self.filtered_velocity[2]:.4f} m/s, speed={filtered_speed:.4f} m/s"
            )

            # Update maximum speeds for statistics
            self.max_raw_speed = max(self.max_raw_speed, raw_speed)
            self.max_filtered_speed = max(self.max_filtered_speed, filtered_speed)

            # Log filter effectiveness (Task requirement)
            speed_diff = abs(raw_speed - filtered_speed)
            effectiveness = (
                min(1.0, speed_diff / raw_speed) if raw_speed > 0.001 else 0.0
            )
            self.get_logger().info(
                f"[{timestamp:.6f}s] FILTER ANALYSIS: "
                f"speed_reduction={speed_diff:.4f} m/s, effectiveness={effectiveness:.3f}"
            )

            self._publish_velocity_data(
                raw_velocity,
                self.filtered_velocity,
                curr_time,
                raw_speed,
                filtered_speed,
            )

        except Exception as e:
            self.get_logger().error(f"Error computing velocity: {e}")

    def _publish_velocity_data(
        self,
        raw_vel: np.ndarray,
        filt_vel: np.ndarray,
        timestamp,
        raw_speed: float,
        filt_speed: float,
    ) -> None:
        """Publish all velocity-related data and analysis metrics with enhanced logging."""
        # Create timestamp
        stamp = timestamp.to_msg()

        # Enhanced logging for published values (Task requirement)
        self.get_logger().info(
            f"[{timestamp.nanoseconds / 1e9:.6f}s] PUBLISHING: "
            f"Raw speed={raw_speed:.4f} m/s, Filtered speed={filt_speed:.4f} m/s"
        )

        # Publish Vector Messages for ROS ecosystem
        raw_vel_msg = Vector3Stamped()
        raw_vel_msg.header.stamp = stamp
        raw_vel_msg.header.frame_id = "map"
        raw_vel_msg.vector.x, raw_vel_msg.vector.y, raw_vel_msg.vector.z = raw_vel
        self.raw_velocity_pub.publish(raw_vel_msg)

        # Filtered velocity vector
        filt_vel_msg = Vector3Stamped()
        filt_vel_msg.header.stamp = stamp
        filt_vel_msg.header.frame_id = "map"
        filt_vel_msg.vector.x, filt_vel_msg.vector.y, filt_vel_msg.vector.z = filt_vel
        self.filtered_velocity_pub.publish(filt_vel_msg)

        # Velocity comparison difference vector
        vel_diff = raw_vel - filt_vel
        comparison_msg = Vector3Stamped()
        comparison_msg.header.stamp = stamp
        comparison_msg.header.frame_id = "map"
        comparison_msg.vector.x, comparison_msg.vector.y, comparison_msg.vector.z = (
            vel_diff
        )
        self.velocity_comparison_pub.publish(comparison_msg)

        # Publish Scalar Messages for PlotJuggler (Task requirement for visualization)
        # Raw velocity components
        self.raw_velocity_x_pub.publish(Float64(data=float(raw_vel[0])))
        self.raw_velocity_y_pub.publish(Float64(data=float(raw_vel[1])))
        self.raw_velocity_z_pub.publish(Float64(data=float(raw_vel[2])))

        # Filtered velocity components
        self.filtered_velocity_x_pub.publish(Float64(data=float(filt_vel[0])))
        self.filtered_velocity_y_pub.publish(Float64(data=float(filt_vel[1])))
        self.filtered_velocity_z_pub.publish(Float64(data=float(filt_vel[2])))

        # Speed magnitudes
        self.raw_speed_pub.publish(Float64(data=raw_speed))
        self.filtered_speed_pub.publish(Float64(data=filt_speed))

        # Analysis metrics
        speed_diff = abs(raw_speed - filt_speed)
        self.speed_difference_pub.publish(Float64(data=speed_diff))

        # Calculate filter effectiveness (0 = no filtering, 1 = maximum filtering)
        effectiveness = min(1.0, speed_diff / raw_speed) if raw_speed > 0.001 else 0.0
        self.filter_effectiveness_pub.publish(Float64(data=effectiveness))

        # Log statistics periodically (Task requirement)
        if self.max_raw_speed > 0:
            self.get_logger().debug(
                f"Statistics: Max raw speed={self.max_raw_speed:.3f} m/s, "
                f"Max filtered speed={self.max_filtered_speed:.3f} m/s, "
                f"Current effectiveness={effectiveness:.3f}"
            )


def main(args=None) -> None:
    """Initialize and run the velocity filter node."""
    rclpy.init(args=args)
    node = None
    try:
        node = VelocityFilterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

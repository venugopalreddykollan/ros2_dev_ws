#!/usr/bin/env python3
"""
Velocity Filter Node for ROS2.

Computes and filters velocities from pose data using low-pass filtering.

Author: Venugopal Reddy Kollan
License: Apache License 2.0
"""

import math
from collections import deque
from typing import Optional, Deque, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import Float64, Header


class VelocityFilterNode(Node):
    """
    ROS2 node for velocity computation and low-pass filtering from pose data.
    
    Subscribed Topics:
        /current_pose (geometry_msgs/PoseStamped): Input pose data
    
    Published Topics:
        /raw_velocity, /filtered_velocity (geometry_msgs/Vector3Stamped): Velocity vectors
        /raw_speed, /filtered_speed (std_msgs/Float64): Velocity magnitudes
        /velocity_comparison (geometry_msgs/Vector3Stamped): Velocity difference for analysis
        /speed_difference (std_msgs/Float64): Speed difference for filter effectiveness
        /filter_effectiveness (std_msgs/Float64): Filter performance metric (0-1)
    
    Parameters:
        cutoff_frequency (float): Filter cutoff frequency [Hz] (default: 2.0)
        sample_rate (float): Expected pose rate [Hz] (default: 50.0)
    """

    def __init__(self) -> None:
        """Initialize the velocity filter node."""
        super().__init__("velocity_filter_node")

        # Declare parameters with descriptors
        self._declare_parameters()
        
        # Initialize parameters and filter
        self._initialize_filter()
        
        # Setup communication
        self._setup_communication()
        
        # State variables
        self.filtered_velocity: np.ndarray = np.array([0.0, 0.0, 0.0])
        self.pose_history: Deque[Tuple[np.ndarray, any]] = deque(maxlen=2)
        
        # Statistics for PlotJuggler visualization
        self.data_count = 0
        self.max_raw_speed = 0.0
        self.max_filtered_speed = 0.0
        
        self.get_logger().info(f"VelocityFilterNode started - cutoff: {self.cutoff_freq}Hz")
    
    def _declare_parameters(self) -> None:
        """Declare parameters with validation ranges."""
        cutoff_desc = ParameterDescriptor(
            description="Low-pass filter cutoff frequency in Hz",
            floating_point_range=[FloatingPointRange(from_value=0.1, to_value=50.0)]
        )
        sample_desc = ParameterDescriptor(
            description="Expected pose message rate in Hz",
            floating_point_range=[FloatingPointRange(from_value=1.0, to_value=1000.0)]
        )
        
        self.declare_parameter("cutoff_frequency", 2.0, cutoff_desc)
        self.declare_parameter("sample_rate", 50.0, sample_desc)
    
    def _initialize_filter(self) -> None:
        """Initialize filter parameters with validation."""
        self.cutoff_freq: float = self.get_parameter("cutoff_frequency").value
        self.sample_rate: float = self.get_parameter("sample_rate").value
        
        # Validate ranges
        if not (0.1 <= self.cutoff_freq <= 50.0):
            raise ValueError(f"Invalid cutoff frequency: {self.cutoff_freq}")
        if not (1.0 <= self.sample_rate <= 1000.0):
            raise ValueError(f"Invalid sample rate: {self.sample_rate}")
        
        # Calculate filter coefficient: α = dt/(RC + dt)
        rc = 1.0 / (2.0 * math.pi * self.cutoff_freq)
        dt = 1.0 / self.sample_rate
        self.alpha: float = dt / (rc + dt)
    
    def _setup_communication(self) -> None:
        """Setup publishers and subscribers with appropriate QoS."""
        # Reliable QoS for pose input
        pose_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        # Best effort QoS for velocity output
        vel_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscriber
        self.pose_subscriber = self.create_subscription(
            PoseStamped, "current_pose", self.pose_callback, pose_qos
        )
        
        # Publishers
        self.raw_velocity_pub = self.create_publisher(Vector3Stamped, "raw_velocity", vel_qos)
        self.filtered_velocity_pub = self.create_publisher(Vector3Stamped, "filtered_velocity", vel_qos)
        self.raw_speed_pub = self.create_publisher(Float64, "raw_speed", vel_qos)
        self.filtered_speed_pub = self.create_publisher(Float64, "filtered_speed", vel_qos)
        
        # Additional publishers for PlotJuggler visualization and analysis
        self.velocity_comparison_pub = self.create_publisher(Vector3Stamped, "velocity_comparison", vel_qos)
        self.speed_difference_pub = self.create_publisher(Float64, "speed_difference", vel_qos)
        self.filter_effectiveness_pub = self.create_publisher(Float64, "filter_effectiveness", vel_qos)

    def pose_callback(self, msg: PoseStamped) -> None:
        """Process pose message and compute velocities."""
        try:
            # Basic validation
            if not hasattr(msg.pose, 'position'):
                self.get_logger().error("Invalid pose message")
                return
                
            current_time = self.get_clock().now()
            current_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            
            # Check for valid values
            if not np.all(np.isfinite(current_pose)):
                self.get_logger().error(f"Invalid pose values: {current_pose}")
                return
            
            # Store pose and compute velocity if we have enough data
            self.pose_history.append((current_pose, current_time))
            
            if len(self.pose_history) >= 2:
                self._compute_and_publish_velocity()
                
        except Exception as e:
            self.get_logger().error(f"Error in pose callback: {e}")

    def _compute_and_publish_velocity(self) -> None:
        """Compute and publish raw and filtered velocities."""
        try:
            # Get recent poses
            (prev_pose, prev_time), (curr_pose, curr_time) = self.pose_history[-2], self.pose_history[-1]
            
            # Calculate time difference
            dt = (curr_time - prev_time).nanoseconds / 1e9
            if dt <= 0:
                self.get_logger().warn(f"Invalid time difference: {dt}")
                return
            
            # Compute raw velocity
            raw_velocity = (curr_pose - prev_pose) / dt
            
            # Validate velocity
            if not np.all(np.isfinite(raw_velocity)):
                self.get_logger().error("Invalid velocity computed")
                return

            # Apply low-pass filter
            self.filtered_velocity = self.alpha * raw_velocity + (1 - self.alpha) * self.filtered_velocity
            
            # Calculate speeds
            raw_speed = float(np.linalg.norm(raw_velocity))
            filtered_speed = float(np.linalg.norm(self.filtered_velocity))
            
            # Update statistics for PlotJuggler
            self.data_count += 1
            self.max_raw_speed = max(self.max_raw_speed, raw_speed)
            self.max_filtered_speed = max(self.max_filtered_speed, filtered_speed)
            
            # Log progress periodically (every 100 samples)
            if self.data_count % 100 == 0:
                self.get_logger().info(
                    f"Processed {self.data_count} velocity samples. "
                    f"Max speeds: Raw={self.max_raw_speed:.3f}, Filtered={self.max_filtered_speed:.3f} m/s"
                )
            
            # Publish data
            self._publish_velocity_data(raw_velocity, self.filtered_velocity, curr_time, raw_speed, filtered_speed)
            
        except Exception as e:
            self.get_logger().error(f"Error computing velocity: {e}")
    
    def _publish_velocity_data(self, raw_vel: np.ndarray, filt_vel: np.ndarray, 
                              timestamp, raw_speed: float, filt_speed: float) -> None:
        """Publish velocity data to all topics with enhanced PlotJuggler support."""
        try:
            # Create header for consistent timestamping
            header = Header()
            header.stamp = timestamp.to_msg()
            header.frame_id = "world"
            
            # Create and publish raw velocity
            raw_msg = Vector3Stamped()
            raw_msg.header = header
            raw_msg.vector.x, raw_msg.vector.y, raw_msg.vector.z = map(float, raw_vel)
            self.raw_velocity_pub.publish(raw_msg)

            # Create and publish filtered velocity
            filt_msg = Vector3Stamped()
            filt_msg.header = header
            filt_msg.vector.x, filt_msg.vector.y, filt_msg.vector.z = map(float, filt_vel)
            self.filtered_velocity_pub.publish(filt_msg)

            # Publish speeds
            self.raw_speed_pub.publish(Float64(data=raw_speed))
            self.filtered_speed_pub.publish(Float64(data=filt_speed))
            
            # Enhanced PlotJuggler visualization data
            
            # Velocity difference (shows filter effect)
            velocity_diff = raw_vel - filt_vel
            diff_msg = Vector3Stamped()
            diff_msg.header = header
            diff_msg.vector.x, diff_msg.vector.y, diff_msg.vector.z = map(float, velocity_diff)
            self.velocity_comparison_pub.publish(diff_msg)
            
            # Speed difference (magnitude of filtering effect)
            speed_diff = abs(raw_speed - filt_speed)
            self.speed_difference_pub.publish(Float64(data=speed_diff))
            
            # Filter effectiveness (0-1 scale: higher means more filtering)
            if raw_speed > 0.001:  # Avoid division by zero
                effectiveness = min(1.0, speed_diff / raw_speed)
            else:
                effectiveness = 0.0
            self.filter_effectiveness_pub.publish(Float64(data=effectiveness))
            
        except Exception as e:
            self.get_logger().error(f"Error publishing: {e}")


def main(args: Optional[list] = None) -> None:
    """Main entry point for the velocity filter node."""
    try:
        rclpy.init(args=args)
        node = VelocityFilterNode()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("Node stopped by user")
        finally:
            node.destroy_node()
            
    except Exception as e:
        print(f"Failed to start node: {e}")
        raise
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

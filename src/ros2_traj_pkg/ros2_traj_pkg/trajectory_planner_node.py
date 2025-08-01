#!/usr/bin/env python3
"""
Trajectory Planner Node for ROS2.

This module provides trajectory planning services using cubic polynomial interpolation
for smooth robot motion between waypoints with continuous acceleration profiles.

Author: Venugopal Reddy Kollan
License: Apache License 2.0
"""

import rclpy
from rclpy.node import Node

import numpy as np

from geometry_msgs.msg import PoseStamped
from typing import Tuple, List, Optional
from custom_interface.srv import PlanTrajectory
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class TrajectoryPlanner(Node):
    """
    ROS2 node for trajectory planning using cubic polynomial interpolation.
    
    This node provides a service to generate smooth trajectories between start
    and goal poses using 3rd-order polynomials with rest-to-rest boundary conditions.
    
    Subscribed Topics:
        None
    
    Published Topics:
        /current_pose (geometry_msgs/PoseStamped): Current trajectory pose
    
    Services:
        /plan_trajectory (custom_interface/PlanTrajectory): Plan trajectory service
    
    Parameters:
        publisher_frequency (int): Publishing frequency [Hz] (default: 50)
        max_velocity (float): Maximum velocity limit [m/s] (default: 1.0)
        max_acceleration (float): Maximum acceleration limit [m/s²] (default: 0.5)
        max_jerk (float): Maximum jerk limit [m/s³] (default: 2.0)
    """

    def __init__(self):
        """Initialize the trajectory planner node with parameters and services."""
        
        super().__init__("trajectory_planner")

        # Declare parameters
        self.declare_parameter("publisher_frequency", 50)
        self.declare_parameter("max_velocity", 1.0)
        self.declare_parameter("max_acceleration", 0.5)
        self.declare_parameter("max_jerk", 2.0)

        # Get parameters
        self.freq = self.get_parameter("publisher_frequency").value
        self.max_vel = self.get_parameter("max_velocity").value
        self.max_acc = self.get_parameter("max_acceleration").value
        self.max_jerk = self.get_parameter("max_jerk").value

        self.get_logger().info(f"Trajectory parameters loaded:")
        self.get_logger().info(f"Publisher frequency: {self.freq} Hz")
        self.get_logger().info(f"Max velocity: {self.max_vel} m/s")
        self.get_logger().info(f"Max acceleration: {self.max_acc} m/s²")
        self.get_logger().info(f"Max jerk: {self.max_jerk} m/s³")

        # # Configure QoS for reliable pose publishing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.pose_pub = self.create_publisher(PoseStamped, "current_pose", qos_profile)
        
        # Service
        self.srv = self.create_service(
            PlanTrajectory, "plan_trajectory", self.plan_callback
        )

        # Internal state for publishing
        self.trajectory = []
        self.current_index = 0
        self.timer = None

        self.get_logger().info("Trajectory Planner Node initialized.")
    
    def _validate_position(self, position, name: str) -> np.ndarray:
        """
        Validate and extract position from geometry message.
    
        Args:
        position: geometry_msgs.msg.Point - Position to validate
        name: str - Name for error messages ("start" or "goal")
    
        Returns:
        np.ndarray: Validated position array [x, y, z]
    
        Raises:
        ValueError: If position contains invalid values
        """
        try:
            pos = np.array([position.x, position.y, position.z])
        
            # Check for finite values (no NaN, inf, -inf)
            if not np.all(np.isfinite(pos)):
                raise ValueError(f"Non-finite values in {name} position: {pos}")
        
            # Check for reasonable bounds (adjust as needed for your application)
            max_coord = 1000.0  # 1km limit - adjust as needed
            if np.any(np.abs(pos) > max_coord):
                raise ValueError(f"{name} position exceeds bounds (±{max_coord}m): {pos}")
            return pos
        
        except AttributeError as e:
            raise ValueError(f"Invalid {name} position message format: {e}")
        except Exception as e:
            raise ValueError(f"Failed to extract {name} position: {e}")

    def plan_callback(self, request: PlanTrajectory.Request, response: PlanTrajectory.Response) -> PlanTrajectory.Response:
        """
        Service callback to plan trajectory from start to goal within duration.
        Returns immediately after trajectory generation; publishing happens asynchronously.
        """
        try:
            # Validate and extract positions with proper error handling
            start_pos = self._validate_position(request.start.position, "start")
            goal_pos = self._validate_position(request.goal.position, "goal")
        
            # Validate duration
            duration = float(request.duration)
            if duration <= 0.0:
                raise ValueError(f"Duration must be positive, got: {duration}")
            if duration > 300.0:  # 5 minutes max - adjust as needed
                raise ValueError(f"Duration too large (max 300s), got: {duration}")

            # Validate that start and goal are different
            distance = np.linalg.norm(goal_pos - start_pos)
            if distance < 1e-6:
                self.get_logger().warn("Start and goal positions are nearly identical")
                response.success = True
                response.message = "No movement required - positions are identical"
                return response

            # Continue with existing trajectory calculation...
            # Estimate maximum velocity and acceleration from 3rd order polynomial
            # For s(t) = 3t² - 2t³, max velocity occurs at t = 0.5
            # Max velocity = 1.5 * distance / duration
            estimated_max_vel = 1.5 * distance / duration

            # Max acceleration occurs at t = 0 and t = 1
            # Max acceleration = 6 * distance / duration²
            estimated_max_acc = 6.0 * distance / (duration * duration)

            # Validate against limits
            if estimated_max_vel > self.max_vel:
                self.get_logger().warn(
                    f"Estimated max velocity ({estimated_max_vel:.3f} m/s) exceeds limit ({self.max_vel} m/s)"
                )

            if estimated_max_acc > self.max_acc:
                self.get_logger().warn(
                    f"Estimated max acceleration ({estimated_max_acc:.3f} m/s²) exceeds limit ({self.max_acc} m/s²)"
                )

            self.get_logger().info(
                f"Planning trajectory: distance={distance:.3f}m, duration={duration}s, "
                f"max_vel={estimated_max_vel:.3f}m/s, max_acc={estimated_max_acc:.3f}m/s²"
            )

            # Generate 3rd order polynomial trajectory with continuous acceleration
            total_steps = int(duration * self.freq)
            trajectory = []

            # 3rd order polynomial: p(t) = a0 + a1*t + a2*t^2 + a3*t^3
            # Boundary conditions: p(0)=start, p(T)=goal, v(0)=0, v(T)=0
            # This gives continuous position, velocity, and acceleration

            for i in range(total_steps):
                t = i / (total_steps - 1) if total_steps > 1 else 0.0
                t_norm = t  # normalized time from 0 to 1

                # 3rd order polynomial coefficients for continuous acceleration
                # s(t) = 3*t^2 - 2*t^3 (this ensures smooth start and stop)
                s = 3 * t_norm**2 - 2 * t_norm**3

                # Interpolate position
                pos = start_pos + s * (goal_pos - start_pos)
                trajectory.append(pos.tolist())

            self.get_logger().info(
                f"Trajectory generated with {len(trajectory)} poses. Starting publishing..."
            )

            # Store trajectory and start timer-based publishing
            self.trajectory = trajectory
            self.current_index = 0

            # Cancel any existing timer before starting a new one
            if self.timer is not None:
                self.timer.cancel()

            self.timer = self.create_timer(1.0 / self.freq, self.publish_timer_callback)

            response.success = True
            response.message = (
                "Trajectory successfully generated and publishing started."
            )
            return response

        except ValueError as e:
            self.get_logger().error(f"Parameter validation error: {e}")
            response.success = False
            response.message = f"Invalid parameters: {str(e)}"
            return response
        except RuntimeError as e:
            self.get_logger().error(f"Trajectory generation error: {e}")
            response.success = False
            response.message = f"Generation failed: {str(e)}"
            return response

    def publish_timer_callback(self):
        """Timer callback to publish the trajectory poses one-by-one."""
        if self.current_index >= len(self.trajectory):
            self.get_logger().info("Finished publishing all trajectory poses.")
            self.timer.cancel()
            self.timer = None
            return

        pos = self.trajectory[self.current_index]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]
        # Orientation: no interpolation yet, so use default (w=1)
        msg.pose.orientation.w = 1.0

        self.pose_pub.publish(msg)
        self.current_index += 1


def main(args=None) -> None:
    """Main entry point for the trajectory planner node."""
    rclpy.init(args=args)
    node = None
    try:
        node = TrajectoryPlanner()
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

#!/usr/bin/env python3
# Uncomment all the lines below if you you wish not to use the generate_parameter_library
# and comment the generated parameters
# declare and get the parameters
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
from custom_interface.srv import PlanTrajectory
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Import generated parameters
from ros2_traj_pkg.trajectory_planner_parameters import trajectory_planner


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
    """

    def __init__(self):
        """Initialize the trajectory planner node with parameters and services."""
        super().__init__("trajectory_planner")

        # Declare using generated parameters
        self.param_listener = trajectory_planner.ParamListener(self)
        self.params = self.param_listener.get_params()

        # Use parameters
        self.freq = self.params.publisher_frequency
        self.max_vel = self.params.max_velocity
        self.max_acc = self.params.max_acceleration
        self.max_jerk = self.params.max_jerk

        self.get_logger().info("Trajectory parameters loaded:")
        self.get_logger().info(f"Publisher frequency: {self.freq} Hz")
        self.get_logger().info(f"Max velocity: {self.max_vel} m/s")
        self.get_logger().info(f"Max acceleration: {self.max_acc} m/s²")
        self.get_logger().info(f"Max jerk: {self.max_jerk} m/s³")

        # Configure QoS for reliable pose publishing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
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

    def _update_parameters_if_changed(self):
        """Update parameters if they changed at runtime."""
        if self.param_listener.is_old(self.params):
            old_freq = self.freq
            self.params = self.param_listener.get_params()
            self.freq = self.params.publisher_frequency
            self.max_vel = self.params.max_velocity
            self.max_acc = self.params.max_acceleration
            self.max_jerk = self.params.max_jerk

            if abs(self.freq - old_freq) > 0.1:
                self.get_logger().info(f" Publisher frequency updated: {self.freq} Hz")

            self.get_logger().info("Parameters refreshed from YAML")

    def _validate_position(self, position, name: str) -> np.ndarray:
        """
        Validate and extract position from geometry message.

        Args
        ----
        position: geometry_msgs.msg.Point
            Position to validate
        name: str
            Name for error messages ("start" or "goal")

        Returns
        -------
        np.ndarray
            Validated position array [x, y, z]

        Raises
        ------
        ValueError
            If position contains invalid values

        """
        try:
            pos = np.array([position.x, position.y, position.z])

            # Check for finite values (no NaN, inf, -inf)
            if not np.all(np.isfinite(pos)):
                raise ValueError(f"Non-finite values in {name} position: {pos}")

            # Check for reasonable bounds (adjust as needed for your application)
            max_coord = 1000.0  # 1km limit - adjust as needed
            if np.any(np.abs(pos) > max_coord):
                raise ValueError(
                    f"{name} position exceeds bounds (±{max_coord}m): {pos}"
                )
            return pos

        except AttributeError as e:
            raise ValueError(f"Invalid {name} position message format: {e}")
        except Exception as e:
            raise ValueError(f"Failed to extract {name} position: {e}")

    def _validate_duration(self, duration: float) -> None:
        """
        Validate trajectory duration parameter.

        Args
        ----
        duration: float
            Requested trajectory duration in seconds

        Raises
        ------
        ValueError
            If duration is invalid
        """
        if duration <= 0.0:
            raise ValueError(f"Duration must be positive, got: {duration}")
        if duration > 300.0:  # 5 minutes max - adjust as needed
            raise ValueError(f"Duration too large (max 300s), got: {duration}")

    def _check_distance_and_warn_limits(
        self, start_pos: np.ndarray, goal_pos: np.ndarray, duration: float
    ) -> float:
        """
        Calculate distance and validate against velocity/acceleration limits.

        Args
        ----
        start_pos: np.ndarray
            Start position [x, y, z]
        goal_pos: np.ndarray
            Goal position [x, y, z]
        duration: float
            Trajectory duration in seconds

        Returns
        -------
        float
            Distance between start and goal positions

        Raises
        ------
        RuntimeError
            If positions are nearly identical (special case for plan_callback)

        """
        distance = np.linalg.norm(goal_pos - start_pos)
        if distance < 1e-6:
            # This is a special case that should be handled by the caller
            raise RuntimeError("No movement required - positions are identical")

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
                f"Estimated max velocity ({estimated_max_vel:.3f} m/s) "
                f"exceeds limit ({self.max_vel} m/s)"
            )

        if estimated_max_acc > self.max_acc:
            self.get_logger().warn(
                f"Estimated max acceleration ({estimated_max_acc:.3f} m/s²) "
                f"exceeds limit ({self.max_acc} m/s²)"
            )

        self.get_logger().info(
            f"Planning trajectory: distance={distance:.3f}m, duration={duration}s, "
            f"max_vel={estimated_max_vel:.3f}m/s, max_acc={estimated_max_acc:.3f}m/s²"
        )

        return distance

    def _generate_cubic_trajectory(
        self, start_pos: np.ndarray, goal_pos: np.ndarray, duration: float
    ) -> list:
        """
        Generate trajectory using 3rd order polynomial interpolation.

        Args
        ----
        start_pos: np.ndarray
            Start position [x, y, z]
        goal_pos: np.ndarray
            Goal position [x, y, z]
        duration: float
            Trajectory duration in seconds

        Returns
        -------
        list
            List of trajectory positions as [x, y, z] lists

        """
        total_steps = int(duration * self.freq)
        trajectory = []

        for i in range(total_steps):
            t = i / (total_steps - 1) if total_steps > 1 else 0.0

            # 3rd order polynomial: s(t) = 3*t^2 - 2*t^3
            # This ensures smooth start and stop with continuous acceleration
            s = 3 * t**2 - 2 * t**3

            # Interpolate position
            pos = start_pos + s * (goal_pos - start_pos)
            trajectory.append(pos.tolist())

        self.get_logger().info(
            f"Trajectory generated with {len(trajectory)} poses. Starting publishing..."
        )
        return trajectory

    def _start_trajectory_publishing(self, trajectory: list) -> None:
        """
        Initialize trajectory publishing with timer.

        Args:
        ----
        trajectory: list
            Generated trajectory points

        """
        self.trajectory = trajectory
        self.current_index = 0

        # Cancel any existing timer before starting a new one
        if self.timer is not None:
            self.timer.cancel()

        self.timer = self.create_timer(1.0 / self.freq, self.publish_timer_callback)

    def plan_callback(
        self, request: PlanTrajectory.Request, response: PlanTrajectory.Response
    ) -> PlanTrajectory.Response:
        """
        Service callback to plan trajectory from start to goal within duration.

        Returns immediately after trajectory generation; publishing happens asynchronously.
        """

        try:
            # Update parameters if they changed
            self._update_parameters_if_changed()

            # Validate and extract positions
            start_pos = self._validate_position(request.start.position, "start")
            goal_pos = self._validate_position(request.goal.position, "goal")

            # Validate duration
            duration = float(request.duration)
            self._validate_duration(duration)

            # Check distance and validate against limits
            try:
                self._check_distance_and_warn_limits(start_pos, goal_pos, duration)
            except RuntimeError as e:
                # Handle identical positions case
                if "identical" in str(e):
                    self.get_logger().warn(
                        "Start and goal positions are nearly identical"
                    )
                    response.success = True
                    response.message = "No movement required - positions are identical"
                    return response
                raise  # Re-raise if it's a different RuntimeError

            # Generate trajectory
            trajectory = self._generate_cubic_trajectory(start_pos, goal_pos, duration)

            # Start publishing
            self._start_trajectory_publishing(trajectory)

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
        msg.header.frame_id = "map"
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]
        # Orientation: no interpolation yet, so use default (w=1)
        msg.pose.orientation.w = 1.0

        self.pose_pub.publish(msg)
        self.current_index += 1


def main(args=None) -> None:
    """Initialize and run the trajectory planner node."""
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

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node

from custom_interface.srv import PlanTrajectory


class TrajectoryPlanner(Node):

    def __init__(self):
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

        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, "current_pose", 10)

        # Service
        self.srv = self.create_service(
            PlanTrajectory, "plan_trajectory", self.plan_callback
        )

        # Internal state for publishing
        self.trajectory = []
        self.current_index = 0
        self.timer = None

        self.get_logger().info("Trajectory Planner Node initialized.")

    def plan_callback(self, request, response):
        """
        Service callback to plan trajectory from start to goal within duration.
        Returns immediately after trajectory generation; publishing happens asynchronously.
        """
        try:
            start_pos = np.array(
                [
                    request.start.position.x,
                    request.start.position.y,
                    request.start.position.z,
                ]
            )
            goal_pos = np.array(
                [
                    request.goal.position.x,
                    request.goal.position.y,
                    request.goal.position.z,
                ]
            )
            duration = request.duration

            if duration <= 0.0:
                raise ValueError("Invalid duration: must be positive.")

            # Calculate trajectory characteristics for validation
            distance = np.linalg.norm(goal_pos - start_pos)

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

        except Exception as e:
            self.get_logger().error(f"Trajectory planning failed: {e}")
            response.success = False
            response.message = str(e)
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


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

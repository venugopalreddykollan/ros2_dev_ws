#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to the config file
    config_file = os.path.join(
        get_package_share_directory('ros2_traj_pkg'),
        'config',
        'trajectory_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='ros2_traj_pkg',
            executable='trajectory_planner_node',
            name='trajectory_planner',
            parameters=[config_file],
            output='screen',
            emulate_tty=True
        )
    ])

#!/usr/bin/env python3
# Uncomment all the lines below if you you wish not to use the generate_parameter_library
#import os

#from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    #publisher_params_file = os.path.join(
        #get_package_share_directory("ros2_traj_pkg"), "config", "publisher_params.yaml"
    #)

    return LaunchDescription(
        [
            Node(
                package="ros2_traj_pkg",
                executable="trajectory_planner_node",
                name="trajectory_planner",
                #parameters=[publisher_params_file],
                output="screen",
                emulate_tty=True,
            )
        ]
    )

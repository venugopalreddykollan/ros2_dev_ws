#!/bin/bash

# ROS2 entrypoint script for trajectory planning package
set -e

# Source ROS2 setup
source /opt/ros/humble/setup.bash

# Source workspace if it exists
if [ -f "/workspace/install/setup.bash" ]; then
    source /ros2_ws_pkg/install/setup.bash
fi

# Execute command
exec "$@"

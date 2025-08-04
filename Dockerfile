# Dockerfile for ROS2 Trajectory Planning Workspace
FROM ros:humble

# Set environment variables
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

#install git and remove cache
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Set workspace directory
RUN mkdir -p /ros2_dev_ws/src
WORKDIR /ros2_dev_ws

#cloning the git repositories
RUN cd src && \
    git clone https://github.com/PickNikRobotics/generate_parameter_library.git && \
    git clone https://github.com/venugopalreddykollan/ros2_dev_ws.git temp_workspace

# Extract your packages from the nested src directory
RUN cd src && \
    # Copy your packages from temp_workspace/src/ to current src/
    cp -r temp_workspace/src/custom_interface . && \
    cp -r temp_workspace/src/ros2_traj_pkg . && \
    # Clean up temporary directory
    rm -rf temp_workspace

# Verify final directory structure
RUN echo "Final Workspace Structure" && \
    ls -la && \
    echo "Source Directory Contents" && \
    ls -la src/ && \
    echo "Package Contents" && \
    ls -la src/custom_interface/ src/ros2_traj_pkg/ src/generate_parameter_library/


# Update rosdep and install dependencies
RUN apt-get update && \
    rosdep update && \
    . /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*


# Build the packages in correct dependency order
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    # Build generate_parameter_library and its dependencies first
    colcon build --packages-select generate_parameter_library generate_parameter_library_py parameter_traits && \
    . install/setup.bash && \
    colcon build --packages-select custom_interface && \
    . install/setup.bash && \
    colcon build --packages-select ros2_traj_pkg && \
    . install/setup.bash


# Source workspace in bashrc for convenience
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_dev_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]

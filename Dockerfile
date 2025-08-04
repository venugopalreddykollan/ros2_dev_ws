# Dockerfile for ROS2 Trajectory Planning Workspace
FROM ros:humble

# Set environment variables
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

#install git and remove cache
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Set workspace directory
RUN mkdir -p /ros2_dev_ws
WORKDIR /ros2_dev_ws

#cloning the git repositories
RUN git clone https://github.com/PickNikRobotics/generate_parameter_library.git
RUN git clone https://github.com/venugopalreddykollan/ros2_dev_ws.git

#Optionally list the contents to verify the clone
# This can help in debugging if the clone fails or if the directory structure is not as expected
RUN ls -la && ls -la src

# Update rosdep and install dependencies
RUN rosdep init || true && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the packages custom_interface and ros2_traj_pkg
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    # Build generate_parameter_library and its dependencies first
    colcon build --packages-select generate_parameter_library generate_parameter_library_py parameter_traits && \
    . install/setup.bash && \
    colcon build --packages-select custom_interface && \
    . install/setup.bash && \
    colcon build --packages-select ros2_traj_pkg"

# Source workspace in bashrc for convenience
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_dev_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]

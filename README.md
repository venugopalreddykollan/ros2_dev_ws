# CONTENTS
  - **Overview**
  - **Features**
  - **Repository Structure**
  - **Velocity visualisation**
  - **Packages Overview**
    - custom_interface
    - ros2_traj_pkg
      - Nodes
      - Topics
  - **Installation**
    - Prerequisites
    - Standard Installation
    - Docker Instsallation
  - **Usage**
    - Clone repo and check launch manually
    - Development
      - Package dependencies
      - Build order
    - Docker
    - Code Quality
      - Install Development Dependencies
      - Run workspace level Linting
  - **License**
  - **Support**


## 1.Overview

A complete ROS2 workspace featuring advanced trajectory planning with 3rd order polynomial generation, real-time velocity filtering, and comprehensive visualization capabilities.


## 2.Features:
-   **3rd Order Polynomial Trajectory Generation** th smooth acceleration profiles
- **Real-time Velocity Filtering** using low-pass RC filters
- **Visualization** using plot_juggler
- **Comprehensive Parameter Management** via YAML configuration
- **Professional Development Tools** with testing


## 3. velocity Visualisation Plots
- **Plot video is uploaded in the doc section of github. Do check it** 
- **Plotting is also shown for raw and filtered speed since speed monitoring is essential for real robots** 


## 4.Repository Structure
This repository follows standard ROS2 workspace conventions:

```
ros2_dev_ws
  |-----src
  |      |---custom_interface
  |      |     |---srv
  |      |     |    | ----- PlanTrajectory.srv
  |      |     |---CMakeLists.txt
  |      |     |---package.xml
  |      |---ros2_traj_pkg
  |      |     |---config
  |      |     |    |---trajectory_params.yaml
  |      |     |    |---velocity_filter_params.yaml
  |      |     |---launch
  |      |     |    |---complete_system_launch.py
  |      |     |    |---trajectory_launch.py
  |      |     |    |---velocity_filter_launch.py
  |      |     |---resource
  |      |     |    |---ros2_traj_pkg
  |      |     |---ros2_traj_pkg
  |      |     |    |---__init__.py
  |      |     |    |---filtervelocity_subscriber_node.py
  |      |     |    |---trajectory_planner_node.py
  |      |     |---test
  |      |          |---test_copyright.py
  |      |          |---test_flake8.py
  |      |          |---test_nodes_comms_and_props.py
  |      |          |---test_peps257.py
  |      |---.pre-commit-config.yaml
  |      |---LICENSE
  |      |---package.xml
  |      |---setup.cfg
  |      |---setup.py
  |---.bandit
  |---.flake8
  |---.gitignore
  |---.pylintrc
  |---Dockerfile
  |---README.md
  |---pyproject.toml
  |---requirements-dev.txt
  |---run_linting.sh
```


## 5.Packages Overview

### 1. custom_interface
-  Defines custom service interfaces for trajectory planning
- **Contains**: 
  - `srv/PlanTrajectory.srv` - Service definition for trajectory planning requests

### 2. ros2_traj_pkg  
- Advanced trajectory planning and velocity filtering
- **Contains**:
  - **Launch Files**: Complete system deployment with parameter loading
  - **Configuration Files**: YAML-based parameter management
  - **Trajectory Planner Node**: 3rd order polynomial generation with parameter validation
  - **Velocity Filter Node**: Low-pass filtering with magnitude publishing for visualization
- **Nodes**
    - trajectory_planner_node- Trajectory generation and publishing
    - filtervelocity_subscriber_node - Velocity computation and filtering and also visualisation using plot juggler
- **Topics**
  - `/current_pose` (geometry_msgs/PoseStamped) - Published trajectory poses
  - `/raw_velocity` (geometry_msgs/Vector3) - Computed raw velocity  
  - `/filtered_velocity` (geometry_msgs/Vector3) - Low-pass filtered velocity

## 6. Installation
  ### Prerequisites--
  - **Operating System Requirements:**
      - Ubuntu 22.04 (Recommended)
      - For Windows Users
        - Check if WSL2 is enabled
        - Install Ubuntu 22.04 LTS app in Microsoft
  ### Standard Installation--
  - **ROS2 Installation:**
    - follow this steps for installing ROS2 Humble [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html]
  ### Requirements--
    - Install colcon
      ```bash
      # Install essential tools
      sudo apt update
      sudo apt upgrade
      sudo apt install python3-colcon-common-extensions

      #Initialize and update rosdep
      sudo rosdep init
      rosdep update

      # If the above commands does not work it means rosdep is not installed
      sudo apt update
      sudo apt install python3-rosdep
      ```
    - **To run the Dockerfile make sure DOcker Desktop is installed**
 

## 7.Usage
  ### Running nodes with repo cloning
   - **Clone and Build**
      ```bash
      # Sourcing the ros humble
      source /opt/ros/humble/setup.bash

      # Clone the workspace
      git clone https://github.com/venugopalreddykollan/ros2_dev_ws.git
      cd ros2_dev_ws

      # Install dependencies
      rosdep update
      rosdep install --from-paths src --ignore-src -r -y

      # Install development dependencies (for code quality tools)
      pip3 install -r requirements-dev.txt

      # Build the workspace (interface package first)
      colcon build --packages-select custom_interface
      source install/setup.bash
      colcon build --packages-select ros2_traj_pkg

      # Source the workspace
      source install/setup.bash
      ```

  #### Launching the nodes
  - **To check the functionalities of the Nodes run the following commands in the bash terminal**
      ```bash
      ros2 launch ros2_traj_pkg complete_system_launch.py
      ```

  #### Service call
  - **Open a new terminal and run the below commands to call the service**
      ```bash
      # Source the environment
      source install/setup.bash

      # Call the service
      ros2 service call /plan_trajectory custom_interface/srv/PlanTrajectory  "{start: {position: {x: 0.0, y: 0.0, z: 0.0},orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},goal: {position: {x: 5.0, y: 3.0, z: 2.0},orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},duration: 10.0}"
      ```
  #### Pose Publishing
  - **Open another terminal to check for the functionalities**
      ```bash
      # Source the environment
      source install/setup.bash

      # To track the nodes
      ros2 node list

      # Return the list of all topics
      ros2 topic list

      # To check the published poses
      ros2 topic echo /current_pose
      ```

  #### Velocity Computation
  - **Open another terminal to check for computed velocities and filtered velocities**
      ```bash
      # Source the environment
      source install/setup.bash

      # To check the velocities published on a seperate topic
      ros2 topic echo /raw_velocity

      # To check the filtered velocities published
      ros2 topic echo /filtered_velocity 
      ```

  ### Build and Run Docker
  - Ensure WSL2 integration is enabled
  - Make sure that Docker Desktop is running 

    ```bash
    # Clone the repository
    git clone https://github.com/venugopalreddykollan/ros2_dev_ws.git
    cd ros2_dev_ws

    # Build Docker image
    docker build -t ros2_dev_ws .

    # To verify the image is build succesfully
    docker image

    # Run the container
    docker run -it ros2_dev_ws
    ```
    #### Run and launch the ros2 nodes
    ```bash
    #Build the packages
    colcon build

    #sourcing 
    source install/setup.py

    #Launching nodes
    ros2 launch ros2_traj_pkg complete_system_launch.py
    ```
    - **To check the topics data** 
    ```bash
    #To list all the nodes
    ros2 node list

    #To list all the topics 
    ros2 topic list -t
  
    #To see the data published on each topic
    ros2 topic echo /current_pose
    ros2 topic echo /raw_velocity
    ros2 topic echo /filetered_velocity
    ```

  ### Code Quality
  - The workspace includes comprehensive linting and formatting tools to maintain professional code standards across all packages.
  - **Install Development Dependencies**
    ```bash
    # Install all linting and formatting tools
    pip3 install -r requirements-dev.txt
    ```
  - **Run Workspace-Level Linting**
    ```bash
    # From workspace root
    cd /home/ros2/ros2_dev_ws

    # Run comprehensive linting on all packages
    ./run_linting.sh
    ```
  ### Run Tests
  - This 
      ```bash
      # Run all tests
      colcon test

      # Run specific test
      colcon test --packages-select ros2_traj_pkg

      # To check test results
      colcon test-result --verbose
      ```

## 8. Troubleshooting
  ### Common Issues
  - **Build fails for ros2_traj_pkg**
      ```bash
      # Solution: Build custom_interface first
      colcon build --packages-select custom_interface
      source install/setup.bash
      colcon build --packages-select ros2_traj_pkg
      ```
  - **Service not found**
    ```bash
    # Solution: Source the workspace
    source install/setup.bash
    ```
## 9. License

This project is licensed under the Apache License 2.0 - see the [LICENSE](src/ros2_traj_pkg/LICENSE) file for details.

## 10. Support
  ### For issues or feature requests
  - Open an issue in my repository
  - Provide detailed information regarding your problem
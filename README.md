# ROS2 Velocity Filtering & Trajectory Planning Workspace

A complete ROS2 workspace featuring advanced trajectory planning with 3rd order polynomial generation, real-time velocity filtering, and comprehensive visualization capabilities.

## 🎯 Overview

This workspace provides a robust ROS2 system for trajectory planning and velocity analysis, featuring:
- **3rd Order Polynomial Trajectory Generation** with smooth acceleration profiles
- **Real-time Velocity Filtering** using low-pass RC filters
- **Multiple Visualization Options** including rqt_plot and terminal-based monitoring
- **Comprehensive Parameter Management** via YAML configuration
- **Professional Development Tools** with testing and quality assurance

## Repository Structure

This repository follows standard ROS2 workspace conventions:

```
ros2_dev_ws/                    # Workspace root
├── src/                        # Source packages directory
│   ├── custom_interface/       # Custom service definitions package
│   └── ros2_traj_pkg/         # Main trajectory planning package
├── build/                      # Build artifacts (excluded from git)
├── install/                    # Install artifacts (excluded from git)
├── log/                        # Log files (excluded from git)
├── .gitignore                  # Git ignore rules
└── README.md                   # This file
```

## Packages Overview

### 1. custom_interface
- **Type**: Interface package (C++)
- **Purpose**: Defines custom service interfaces for trajectory planning
- **Contains**: 
  - `srv/PlanTrajectory.srv` - Service definition for trajectory planning requests

### 2. ros2_traj_pkg  
- **Type**: Python package
- **Purpose**: Advanced trajectory planning and velocity filtering
- **Contains**:
  - **Trajectory Planner Node**: 3rd order polynomial generation with parameter validation
  - **Velocity Filter Node**: Low-pass filtering with magnitude publishing for visualization
  - **Launch Files**: Complete system deployment with parameter loading
  - **Configuration Files**: YAML-based parameter management
  - **Visualization Tools**: Terminal monitor and rqt_plot support

## Quick Start

### Prerequisites
- ROS2 Humble
- Python 3.8+
- colcon build tools

```bash
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions
```

### Clone and Build

```bash
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
colcon build --packages-select ros2_traj_pkg

# Source the workspace
source install/setup.bash
```

### Quick Start Commands

```bash
# Build and run system
colcon build && source install/setup.bash
ros2 launch ros2_traj_pkg complete_system_launch.py

# Run code quality checks
./run_linting.sh

# Auto-format code
black src/ && isort src/
```

### Launch the System

```bash
# Option 1: Launch complete system
ros2 launch ros2_traj_pkg complete_system_launch.py

# Option 2: Launch individual components
ros2 launch ros2_traj_pkg trajectory_launch.py      # Terminal 1
ros2 launch ros2_traj_pkg velocity_filter_launch.py # Terminal 2
```

## Testing the Trajectory Planning

```bash
# Test the trajectory planning service
ros2 service call /plan_trajectory custom_interface/srv/PlanTrajectory "{
  start: {
    position: {x: 0.0, y: 0.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  },
  goal: {
    position: {x: 5.0, y: 3.0, z: 2.0}, 
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  },
  duration: 10.0
}"

# Monitor the published topics
ros2 topic echo /current_pose          # Trajectory poses
ros2 topic echo /velocity              # Computed velocities  
ros2 topic echo /filtered_velocity     # Filtered velocities
```

## Development

### Package Dependencies
- `custom_interface` → No external ROS2 dependencies
- `ros2_traj_pkg` → Depends on `custom_interface`

### Build Order
Always build `custom_interface` before `ros2_traj_pkg`:
```bash
colcon build --packages-select custom_interface
colcon build --packages-select ros2_traj_pkg
```

### Code Quality

The workspace includes comprehensive linting and formatting tools to maintain professional code standards across all packages.

#### Install Development Dependencies
```bash
# Install all linting and formatting tools
pip3 install -r requirements-dev.txt
```

#### Run Workspace-Level Linting
```bash
# From workspace root
cd /home/ros2/ros2_dev_ws

# Run comprehensive linting on all packages
./run_linting.sh

# If missing tools (mypy, etc.), install them by pressing 'y' when prompted
```

#### What Gets Checked
- **Black**: Code formatting and style consistency
- **isort**: Import organization and sorting
- **Flake8**: PEP8 compliance and style violations
- **Pylint**: Advanced code analysis and quality scoring
- **MyPy**: Type checking (excluding launch files)
- **Bandit**: Security analysis (excluding test assertions)

#### Automatic Fixes
```bash
# Auto-format code with Black
black src/

# Sort imports with isort
isort src/

# Or run both automatically during linting
./run_linting.sh  # Will suggest these commands if issues found
```
```

#### Individual Quality Checks
```bash
# Auto-format code (fixes import ordering and code style)
black src/
isort src/

# Check style compliance
flake8 src/

# Advanced code analysis
pylint src/

# Type checking
mypy src/ --ignore-missing-imports

# Security analysis
bandit -r src/
```

#### Run Tests
```bash
# Run tests
colcon test --packages-select ros2_traj_pkg
colcon test-result --verbose
```

## Features

- ✅ 3rd order polynomial trajectory generation with continuous acceleration
- ✅ Service-based trajectory planning interface  
- ✅ Real-time velocity computation and low-pass filtering
- ✅ Configurable parameters via YAML files
- ✅ Comprehensive testing framework
- ✅ Launch files for easy deployment
- ✅ **Workspace-level code quality tools**:
  - **Black**: Automatic code formatting
  - **isort**: Import organization
  - **Flake8**: Style checking and linting
  - **Pylint**: Advanced code analysis
  - **MyPy**: Type checking
  - **Bandit**: Security analysis
- ✅ Professional development workflow with automated quality checks

## Architecture

### Service Interface
- **Service**: `/plan_trajectory` 
- **Type**: `custom_interface/srv/PlanTrajectory`
- **Input**: Start pose, goal pose, duration
- **Output**: Success status and message

### Topics
- `/current_pose` (geometry_msgs/PoseStamped) - Published trajectory poses
- `/velocity` (geometry_msgs/Vector3) - Computed velocity  
- `/filtered_velocity` (geometry_msgs/Vector3) - Low-pass filtered velocity

### Nodes
1. **trajectory_planner_node** - Trajectory generation and publishing
2. **filtervelocity_subscriber_node** - Velocity computation and filtering

## Troubleshooting

### Common Issues

1. **Build fails for ros2_traj_pkg**
   ```bash
   # Solution: Build custom_interface first
   colcon build --packages-select custom_interface
   source install/setup.bash
   colcon build --packages-select ros2_traj_pkg
   ```

2. **Service not found**
   ```bash
   # Solution: Source the workspace
   source install/setup.bash
   ```

3. **Import errors**
   ```bash
   # Solution: Check dependencies
   rosdep install --from-paths src --ignore-src -r -y
   ```

## Contributing

1. Follow ROS2 package conventions
2. Maintain the existing code style
3. Add tests for new functionality  
4. Update documentation as needed

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](src/ros2_traj_pkg/LICENSE) file for details.

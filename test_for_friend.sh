#!/bin/bash

# ROS2 Trajectory Planner Testing Script
# This script provides multiple ways for your friend to test the functionality

echo "🚀 ROS2 Trajectory Planner Testing Options"
echo "==========================================="
echo ""

# Check if Docker is available
if command -v docker &> /dev/null; then
    echo "✅ Docker is available"
    DOCKER_AVAILABLE=true
else
    echo "❌ Docker not found"
    DOCKER_AVAILABLE=false
fi

# Check if Python is available
if command -v python3 &> /dev/null; then
    echo "✅ Python3 is available"
    PYTHON_AVAILABLE=true
else
    echo "❌ Python3 not found"
    PYTHON_AVAILABLE=false
fi

echo ""
echo "Available Testing Options:"
echo ""

# Option 1: Standalone test (no dependencies)
if [ "$PYTHON_AVAILABLE" = true ]; then
    echo "1. 🧪 Standalone Algorithm Test (Recommended)"
    echo "   - Tests core trajectory planning algorithms"
    echo "   - No ROS2 installation required"
    echo "   - Only needs Python3 + NumPy"
    echo "   Command: python3 standalone_test.py"
    echo ""
fi

# Option 2: Docker test (if available)
if [ "$DOCKER_AVAILABLE" = true ]; then
    echo "2. 🐳 Full ROS2 Docker Test"
    echo "   - Complete ROS2 environment in container"
    echo "   - Tests actual ROS2 nodes and services"
    echo "   - Requires Docker installation"
    echo "   Command: docker-compose up test"
    echo ""
fi

# Option 3: Manual inspection
echo "3. 📋 Manual Code Review"
echo "   - Review source code in src/ directory"
echo "   - Check algorithm implementation"
echo "   - No execution required"
echo "   Files: src/ros2_traj_pkg/ and src/custom_interface/"
echo ""

# Interactive selection
echo "Choose testing option:"
echo "Enter 1 for Standalone Test"
if [ "$DOCKER_AVAILABLE" = true ]; then
    echo "Enter 2 for Docker Test"
fi
echo "Enter 3 for Code Review"
echo "Enter q to quit"
echo ""

read -p "Your choice: " choice

case $choice in
    1)
        if [ "$PYTHON_AVAILABLE" = true ]; then
            echo ""
            echo "🧪 Running Standalone Algorithm Test..."
            echo "======================================"
            
            # Check if NumPy is available, install if needed
            python3 -c "import numpy" 2>/dev/null || {
                echo "Installing NumPy (required dependency)..."
                pip3 install numpy --user
            }
            
            python3 standalone_test.py
        else
            echo "❌ Python3 not available for standalone test"
        fi
        ;;
    2)
        if [ "$DOCKER_AVAILABLE" = true ]; then
            echo ""
            echo "🐳 Starting Docker Test Environment..."
            echo "====================================="
            echo "This will:"
            echo "1. Build ROS2 container with your workspace"
            echo "2. Run trajectory planning tests"
            echo "3. Show velocity filtering in action"
            echo ""
            read -p "Continue? (y/n): " confirm
            if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ]; then
                docker-compose up --build test
            fi
        else
            echo "❌ Docker not available for container test"
        fi
        ;;
    3)
        echo ""
        echo "📋 Code Review Guide"
        echo "==================="
        echo ""
        echo "Key files to review:"
        echo ""
        echo "1. Trajectory Planning Service:"
        echo "   📁 src/ros2_traj_pkg/ros2_traj_pkg/trajectory_planner_node.py"
        echo "   - Core planning algorithm using 3rd order polynomials"
        echo "   - Input validation and error handling"
        echo "   - Service interface implementation"
        echo ""
        echo "2. Velocity Filter Node:"
        echo "   📁 src/ros2_traj_pkg/ros2_traj_pkg/filtervelocity_subscriber_node.py"
        echo "   - Real-time velocity computation and filtering"
        echo "   - Low-pass filter implementation"
        echo "   - PlotJuggler visualization support"
        echo ""
        echo "3. Custom Service Interface:"
        echo "   📁 src/custom_interface/srv/PlanTrajectory.srv"
        echo "   - Service request/response definition"
        echo "   - Uses geometry_msgs/Pose for position data"
        echo ""
        echo "4. Package Configuration:"
        echo "   📁 src/ros2_traj_pkg/package.xml"
        echo "   📁 src/ros2_traj_pkg/setup.py"
        echo "   - ROS2 package metadata and dependencies"
        echo ""
        
        # Show file tree if available
        if command -v tree &> /dev/null; then
            echo "Project Structure:"
            tree src/ -I '__pycache__|*.pyc'
        elif command -v find &> /dev/null; then
            echo "Project Files:"
            find src/ -name "*.py" -o -name "*.srv" -o -name "*.xml" | sort
        fi
        ;;
    q|Q)
        echo "Goodbye! 👋"
        exit 0
        ;;
    *)
        echo "❌ Invalid choice. Please run the script again."
        exit 1
        ;;
esac

echo ""
echo "✅ Testing completed! Check the output above for results."
echo ""
echo "📞 Contact your friend if you need help interpreting the results."

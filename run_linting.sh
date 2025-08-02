#!/bin/bash

# Simple linting script for ros2_traj_pkg only
echo "Running Code Quality Checks for ros2_traj_pkg"
echo "=============================================="

# Check if ros2_traj_pkg directory exists
if [[ ! -d "src/ros2_traj_pkg" ]]; then
    echo "Error: src/ros2_traj_pkg directory not found"
    exit 1
fi

# Find Python files in ros2_traj_pkg only
PYTHON_FILES=$(find src/ros2_traj_pkg -name "*.py")

if [[ -z "$PYTHON_FILES" ]]; then
    echo "No Python files found in ros2_traj_pkg"
    exit 0
fi

echo "Found Python files in ros2_traj_pkg:"
echo "$PYTHON_FILES"
echo

# Track status
OVERALL_STATUS=0

# 1. Code formatting with black
echo "1. Checking code formatting (black)..."
if black --check --diff $PYTHON_FILES; then
    echo "Code formatting: PASSED"
else
    echo "Formatting issues found"
    OVERALL_STATUS=1
fi
echo

# 2. Linting with flake8
echo "2. Running flake8 linting..."
if flake8 $PYTHON_FILES --max-line-length=120; then
    echo "Flake8 linting: PASSED"
else
    echo "Flake8 linting: FAILED"
    OVERALL_STATUS=1
fi
echo

# Summary
echo "=============================================="
if [[ $OVERALL_STATUS -eq 0 ]]; then
    echo "All checks PASSED for ros2_traj_pkg!"
else
    echo "Issues found. Fix with:"
    echo "  black $PYTHON_FILES"
fi

exit $OVERALL_STATUS
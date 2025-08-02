#!/bin/bash

# Simple linting script for ros2_traj_pkg only
echo "Running Code Quality Checks for ros2_traj_pkg"
echo "=============================================="

# Find Python files in ros2_traj_pkg
PYTHON_FILES=$(find src/ros2_traj_pkg -name "*.py" 2>/dev/null || echo "")

echo "Found Python files:"
echo "$PYTHON_FILES"
echo

# Run black formatting check
echo "1. Checking code formatting (black)..."
black --check --diff $PYTHON_FILES
echo

# Run flake8 linting
echo "2. Running flake8 linting..."
flake8 $PYTHON_FILES --max-line-length=120
echo

echo "Linting completed!"
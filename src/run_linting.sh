#!/bin/bash

# ROS2 Trajectory Package Linting Script
set -e

echo "🔍 Running linting and formatting checks..."

# Check if virtual environment is activated
if [[ "$VIRTUAL_ENV" != "" ]]; then
    echo "✅ Virtual environment detected: $VIRTUAL_ENV"
else
    echo "⚠️  No virtual environment detected. Consider using one."
fi

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo ""
echo -e "${BLUE}📋 Installing/updating linting tools...${NC}"
pip install -r requirements-dev.txt

echo ""
echo -e "${BLUE}🖤 Running Black formatter...${NC}"
black --check --diff ros2_traj_pkg/ || {
    echo -e "${YELLOW}⚠️  Code formatting issues found. Run 'black ros2_traj_pkg/' to fix them.${NC}"
    BLACK_FAILED=1
}

echo ""
echo -e "${BLUE}🔍 Running Flake8 linter...${NC}"
flake8 ros2_traj_pkg/ || {
    echo -e "${RED}❌ Flake8 linting failed${NC}"
    FLAKE8_FAILED=1
}

echo ""
echo -e "${BLUE}🔎 Running Pylint...${NC}"
pylint ros2_traj_pkg/ || {
    echo -e "${YELLOW}⚠️  Pylint found issues${NC}"
    PYLINT_FAILED=1
}

echo ""
echo -e "${BLUE}📊 Summary:${NC}"
if [[ $BLACK_FAILED ]]; then
    echo -e "${YELLOW}⚠️  Black: Formatting issues found${NC}"
else
    echo -e "${GREEN}✅ Black: Code is properly formatted${NC}"
fi

if [[ $FLAKE8_FAILED ]]; then
    echo -e "${RED}❌ Flake8: Linting errors found${NC}"
else
    echo -e "${GREEN}✅ Flake8: No linting errors${NC}"
fi

if [[ $PYLINT_FAILED ]]; then
    echo -e "${YELLOW}⚠️  Pylint: Issues found${NC}"
else
    echo -e "${GREEN}✅ Pylint: No issues${NC}"
fi

echo ""
echo -e "${BLUE}🛠️  Quick fix commands:${NC}"
echo "Format code:     black ros2_traj_pkg/"
echo "Check format:    black --check ros2_traj_pkg/"
echo "Run flake8:      flake8 ros2_traj_pkg/"
echo "Run pylint:      pylint ros2_traj_pkg/"

if [[ $FLAKE8_FAILED ]]; then
    exit 1
fi

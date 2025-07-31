#!/bin/bash

# ROS2 Workspace - Code Quality Script
# This script runs comprehensive linting and formatting checks across all packages

set -e  # Exit on any error

echo "🔍 Running Code Quality Checks for ROS2 Workspace"
echo "=================================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get script directory (workspace root)
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check if we're in the right directory (workspace root)
if [[ ! -d "$WORKSPACE_DIR/src" ]]; then
    echo -e "${RED}❌ Error: Must be run from workspace root directory${NC}"
    echo -e "${YELLOW}Expected to find 'src/' directory in: $WORKSPACE_DIR${NC}"
    exit 1
fi

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to install missing packages
install_missing_packages() {
    echo -e "${BLUE}📦 Installing missing development packages...${NC}"
    pip3 install --user flake8 black pylint mypy isort bandit
}

# Check for required tools
missing_tools=()
for tool in flake8 black pylint mypy isort bandit; do
    if ! command_exists "$tool"; then
        missing_tools+=("$tool")
    fi
done

if [[ ${#missing_tools[@]} -ne 0 ]]; then
    echo -e "${YELLOW}⚠️  Missing tools: ${missing_tools[*]}${NC}"
    read -p "Install missing tools? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        install_missing_packages
    else
        echo -e "${RED}❌ Cannot proceed without required tools${NC}"
        exit 1
    fi
fi

# Python files to check (across all packages)
PYTHON_FILES=$(find "$WORKSPACE_DIR/src" -name "*.py" -not -path "*/build/*" -not -path "*/install/*" -not -path "*/log/*" | tr '\n' ' ')

if [[ -z "$PYTHON_FILES" ]]; then
    echo -e "${YELLOW}⚠️  No Python files found to check in workspace${NC}"
    exit 0
fi

echo -e "${BLUE}🔍 Found Python files across packages:${NC}"
for file in $PYTHON_FILES; do
    # Show relative path from workspace root
    rel_path=$(realpath --relative-to="$WORKSPACE_DIR" "$file")
    echo "  - $rel_path"
done
echo

# Show package summary
echo -e "${BLUE}📦 Packages being analyzed:${NC}"
find "$WORKSPACE_DIR/src" -name "setup.py" -exec dirname {} \; | while read pkg_dir; do
    pkg_name=$(basename "$pkg_dir")
    python_count=$(find "$pkg_dir" -name "*.py" -not -path "*/build/*" -not -path "*/install/*" | wc -l)
    echo "  - $pkg_name ($python_count Python files)"
done
echo

# Track overall status
OVERALL_STATUS=0

# 1. Import sorting with isort
echo -e "${BLUE}1️⃣  Checking import sorting (isort)...${NC}"
if isort --check-only --diff $PYTHON_FILES; then
    echo -e "${GREEN}✅ Import sorting: PASSED${NC}"
else
    echo -e "${YELLOW}⚠️  Import sorting issues found. Run 'isort $PYTHON_FILES' to fix${NC}"
    OVERALL_STATUS=1
fi
echo

# 2. Code formatting with black
echo -e "${BLUE}2️⃣  Checking code formatting (black)...${NC}"
if black --check --diff $PYTHON_FILES; then
    echo -e "${GREEN}✅ Code formatting: PASSED${NC}"
else
    echo -e "${YELLOW}⚠️  Formatting issues found. Run 'black $PYTHON_FILES' to fix${NC}"
    OVERALL_STATUS=1
fi
echo

# 3. Linting with flake8
echo -e "${BLUE}3️⃣  Running flake8 linting...${NC}"
if flake8 $PYTHON_FILES; then
    echo -e "${GREEN}✅ Flake8 linting: PASSED${NC}"
else
    echo -e "${RED}❌ Flake8 linting: FAILED${NC}"
    OVERALL_STATUS=1
fi
echo

# 4. Advanced linting with pylint
echo -e "${BLUE}4️⃣  Running pylint analysis...${NC}"
pylint_score=$(pylint $PYTHON_FILES --output-format=text --score=yes 2>/dev/null | grep -E "Your code has been rated" | grep -oE '[0-9]+\.[0-9]+' || echo "0.0")
if (( $(echo "$pylint_score >= 8.0" | bc -l) )); then
    echo -e "${GREEN}✅ Pylint score: $pylint_score/10.0 - PASSED${NC}"
else
    echo -e "${YELLOW}⚠️  Pylint score: $pylint_score/10.0 - Consider improvements${NC}"
    OVERALL_STATUS=1
fi
echo

# 5. Type checking with mypy
echo -e "${BLUE}5️⃣  Running type checking (mypy)...${NC}"
if mypy $PYTHON_FILES --ignore-missing-imports; then
    echo -e "${GREEN}✅ Type checking: PASSED${NC}"
else
    echo -e "${YELLOW}⚠️  Type checking issues found${NC}"
    OVERALL_STATUS=1
fi
echo

# 6. Security analysis with bandit
echo -e "${BLUE}6️⃣  Running security analysis (bandit)...${NC}"
if bandit -r $PYTHON_FILES -f txt; then
    echo -e "${GREEN}✅ Security analysis: PASSED${NC}"
else
    echo -e "${YELLOW}⚠️  Security issues found${NC}"
    OVERALL_STATUS=1
fi
echo

# Summary
echo "================================================"
if [[ $OVERALL_STATUS -eq 0 ]]; then
    echo -e "${GREEN}🎉 All code quality checks PASSED!${NC}"
else
    echo -e "${YELLOW}⚠️  Some issues found. Please review and fix.${NC}"
fi

echo
echo -e "${BLUE}💡 Quick fix commands:${NC}"
echo "  Format code:     black $PYTHON_FILES"
echo "  Sort imports:    isort $PYTHON_FILES"
echo "  View full report: pylint $PYTHON_FILES"

exit $OVERALL_STATUS
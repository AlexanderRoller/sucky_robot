#!/bin/bash

# Sucky Robot Setup Script
# This script initializes submodules and sets up the workspace

set -e  # Exit on any error

echo "🤖 Setting up Sucky Robot workspace..."

# Check if we're in the right directory
if [ ! -f "README.md" ] || [ ! -d "src" ]; then
    echo "❌ Error: Please run this script from the sucky_robot root directory"
    exit 1
fi

# Initialize and update submodules if they're not already populated
echo "📦 Checking git submodules..."
if git submodule status | grep -q "^-"; then
    echo "   Initializing submodules..."
    git submodule update --init --recursive
else
    echo "   Submodules already initialized"
fi

# Install system dependencies
echo "🔧 Installing system dependencies..."
if command -v rosdep &> /dev/null; then
    echo "   Running rosdep update..."
    rosdep update
    
    echo "   Installing ROS2 dependencies..."
    rosdep install --from-paths src --ignore-src -r -y --skip-keys="roboclaw_hardware_interface roboclaw_serial"
else
    echo "   ⚠️  rosdep not found. Please install ROS2 first."
    echo "   You can still build the workspace, but some dependencies might be missing."
fi

# Install Python dependencies
echo "🐍 Installing Python dependencies..."
if command -v pip &> /dev/null; then
    pip install pyserial
else
    echo "   ⚠️  pip not found. Please install pyserial manually: pip install pyserial"
fi

# Build the workspace
echo "🔨 Building workspace..."
if command -v colcon &> /dev/null; then
    colcon build --symlink-install
    echo "✅ Workspace built successfully!"
    echo ""
    echo "🚀 To get started:"
    echo "   source install/setup.bash"
    echo "   ros2 launch sucky sucky_launch.py"
else
    echo "   ⚠️  colcon not found. Please install ROS2 colcon tools."
    echo "   You can install with: sudo apt install python3-colcon-common-extensions"
fi

echo ""
echo "🎉 Setup complete!"

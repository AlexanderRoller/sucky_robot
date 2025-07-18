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
else
    echo "   ⚠️  colcon not found. Please install ROS2 colcon tools."
    echo "   You can install with: sudo apt install python3-colcon-common-extensions"
fi

# Setup battery shutdown permissions
echo ""
echo "🔋 Setting up battery shutdown permissions..."
CURRENT_USER=$(whoami)

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    echo "   ⚠️  Cannot setup shutdown permissions as root. Skipping..."
    echo "   Run this script as a regular user to enable automatic shutdown."
else
    # Create sudoers entry for shutdown commands
    SUDOERS_LINE="$CURRENT_USER ALL=(ALL) NOPASSWD: /sbin/shutdown, /sbin/poweroff"
    SUDOERS_FILE="/etc/sudoers.d/battery-monitor-shutdown"
    
    echo "   Creating sudoers entry for user: $CURRENT_USER"
    if echo "$SUDOERS_LINE" | sudo tee "$SUDOERS_FILE" > /dev/null 2>&1; then
        sudo chmod 440 "$SUDOERS_FILE" 2>/dev/null || true
        echo "   ✅ Battery shutdown permissions configured"
        
        # Test the setup
        if sudo -n /sbin/shutdown --help > /dev/null 2>&1; then
            echo "   ✅ Shutdown permission test passed"
        else
            echo "   ⚠️  Shutdown permission test failed - you may need to log out and back in"
        fi
    else
        echo "   ⚠️  Failed to create sudoers file - automatic shutdown will be disabled"
        echo "   You can enable it later by running: sudo visudo -f $SUDOERS_FILE"
    fi
fi

echo ""
echo "🎉 Setup complete!"
echo ""
echo "🚀 To get started:"
echo "   source install/setup.bash"
echo "   ros2 launch sucky sucky.launch.py"
echo ""
echo "🔋 Battery monitoring configured with automatic shutdown protection"
echo "   Configure thresholds in: src/sucky/config/battery_monitor.yaml"

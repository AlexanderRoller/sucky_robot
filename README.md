# 🤖 Sucky Robot

![Sucky Robot](media/sucky.jpeg)

A ROS2-based differential drive robot with advanced sensor integration, autonomous navigation, and teleoperation capabilities.

## 👥 Credits

Developed by:
- **Alexander Roller** ([AlexanderRoller](https://github.com/AlexanderRoller)) - Systems integration
- **Jason Koubi** ([jkoubs](https://github.com/jkoubs)) - Software and simulation
- **Benjamin Cantarero** - Mechanical design
- **George Fox University** - Chassis design and fabrication

This is a continuation of a previous project ([sweepy](https://github.com/AlexanderRoller/sweepy_ws))

## � Overview & Features

The Sucky Robot is a comprehensive ROS2 robotics platform featuring:

### Hardware
- **Differential Drive Base**: Custom roboclaw motor controller integration
- **Sensor Suite**: SICK TiM781 LiDAR, Intel RealSense D455, IMU
- **Power**: 24V battery system with monitoring

### Capabilities
- 🎮 **Joystick Teleoperation**: Real-time control with configurable speed limits
- 🗺️ **SLAM Mapping**: Real-time simultaneous localization and mapping
- 🧭 **Autonomous Navigation**: Path planning and obstacle avoidance
- 📡 **Multi-sensor Fusion**: EKF-based sensor fusion for robust localization
- 🔋 **Battery Monitoring**: Real-time voltage monitoring with low-battery warnings

##  Installation

### Prerequisites
- ROS2 Humble
- Ubuntu 22.04

### Quick Setup
```bash
# Clone and build
git clone --recursive <repository_url> ~/sucky_robot
cd ~/sucky_robot
./setup.sh

# Source the workspace
source install/setup.bash
```

### Manual Dependencies (if needed)
```bash
# Install ROS2 dependencies
rosdep install --from-paths src --ignore-src -r -y --skip-keys="roboclaw_hardware_interface roboclaw_serial"

# Install Python dependencies
pip install pyserial
```

## 🚀 Usage

### Quick Start
```bash
# Build and launch robot system
source build.sh && source launch_sucky.sh

# Launch host system (on separate computer)
source launch_host.sh
```

### Common Commands
```bash
# Launch complete robot system
ros2 launch sucky sucky.launch.py

# SLAM mapping only
ros2 launch sucky slam.launch.py

# RViz visualization (host computer)
ros2 launch sucky host.launch.py

# Manual keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Networking
For multi-machine operation, set the same ROS_DOMAIN_ID on both robot and host:
```bash
export ROS_DOMAIN_ID=1
```

**Note**: Enterprise networks may block multicast traffic. Contact IT to allow multicast on ports 7400-7447.

## 📁 Package Structure

```
sucky_robot/
├── src/
│   ├── sucky/                          # Main robot package
│   │   ├── launch/                     # Launch files
│   │   ├── config/                     # Configuration files
│   │   ├── urdf/                       # Robot description
│   │   ├── meshes/                     # 3D models
│   │   ├── nodes/                      # ROS2 executable nodes
│   │   ├── tools/                      # Utility scripts and tools
│   │   └── test/                       # Test files
│   ├── roboclaw_hardware_interface/    # Motor controller interface
│   └── roboclaw_serial/               # Serial communication
├── build/                              # Build artifacts
├── install/                            # Install space
├── media/                              # Images and documentation
├── build.sh                           # Build script
├── launch_sucky.sh                    # Robot launch script
├── launch_host.sh                     # Host launch script
└── README.md                          # This file
```

## 🔧 Troubleshooting

### Common Issues

**Missing roboclaw packages:**
```bash
git submodule update --init --recursive
```

**Serial port access:**
```bash
sudo usermod -a -G dialout $USER
# Logout and login again
```

**LiDAR connection:**
```bash
ping 192.168.0.1  # Check network connection
```

**Camera issues:**
```bash
lsusb | grep Intel  # Check USB connection
```

## ⚙️ Configuration

- **LiDAR IP**: `192.168.0.1`
- **Serial Port**: `/dev/ttyACM1` (roboclaw)
- **Battery Range**: 22.0V - 29.4V

## 🙏 Acknowledgments

Thanks to the ROS2 community, SICK AG, Intel, and all contributors to the roboclaw driver.

---

For questions or support, please open an issue or contact the maintainers.

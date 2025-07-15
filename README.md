# 🤖 Sucky Robot

![Sucky Robot](media/sucky.jpeg)

A ROS2-based differential drive robot with advanced sensor integration, autonomous navigation, and teleoperation capabilities.

## 👥 Contributors

**Development Team:**
- **Alexander Roller** ([AlexanderRoller](https://github.com/AlexanderRoller)) - Systems Integration
- **Jason Koubi** ([jkoubs](https://github.com/jkoubs)) - Software & Simulation  
- **Benjamin Cantarero** - Mechanical Design
- **George Fox University** - Chassis Design & Fabrication

*This project builds upon the previous [sweepy](https://github.com/AlexanderRoller/sweepy_ws) robot.*

## 🔍 Overview

The Sucky Robot is a comprehensive ROS2 robotics platform designed for autonomous cleaning and debris collection.

### Hardware Components
- **Differential Drive Base**: Custom roboclaw motor controller integration
- **Sensor Suite**: SICK TiM781 LiDAR, Intel RealSense D455, IMU
- **Arduino Control System**: Cyclone vacuum and servo door control
- **Vacuum System**: High-powered cyclone vacuum with ESC control
- **Door Mechanism**: Dual servo-controlled collection doors
- **Power**: 24V battery system with monitoring

### Key Features
- 🎮 **Teleoperation**: PS4 controller support with configurable speed limits
- 🗺️ **SLAM Mapping**: Real-time simultaneous localization and mapping
- 🧭 **Autonomous Navigation**: Path planning and obstacle avoidance
- 📡 **Sensor Fusion**: EKF-based multi-sensor localization
- 🔋 **Battery Monitoring**: Real-time voltage monitoring with warnings
- 🌪️ **Vacuum Control**: Arduino-controlled cyclone with safety timeouts
- 🚪 **Door Control**: Servo-driven collection doors

## 📦 Installation

### Prerequisites
- **OS**: Ubuntu 22.04
- **ROS**: ROS2 Humble
- **Python**: 3.10+

### Quick Setup
```bash
# Clone repository with submodules
git clone --recursive https://github.com/AlexanderRoller/sucky_robot.git ~/sucky_robot
cd ~/sucky_robot

# Run setup script
./setup.sh

# Source the workspace
source install/setup.bash
```

### Manual Installation (if needed)
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

# Launch host system (on remote computer)
source launch_host.sh
```

### Individual Components
```bash
# Complete robot system
ros2 launch sucky sucky.launch.py

# SLAM mapping
ros2 launch sucky slam.launch.py

# RViz visualization (host computer)
ros2 launch sucky host.launch.py

# Manual keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Controller Commands
- **Square**: Toggle vacuum on/off
- **Triangle**: Toggle doors open/close
- **Circle**: Emergency stop
- **Left stick**: Drive control
- **Right stick**: Rotation control

### Multi-Machine Setup
For distributed operation, set matching ROS_DOMAIN_ID:
```bash
export ROS_DOMAIN_ID=1
```

> **Note**: Enterprise networks may block multicast traffic on ports 7400-7447.

## 🔌 Arduino Control

Arduino-based control system for vacuum and door mechanisms.

### Hardware
- **Arduino Uno**: Controls ESC and servo motors
- **ESC**: Electronic speed controller for vacuum motor
- **Servos**: Dual servo motors for collection doors

### ROS2 Integration
- **Topics**: `/vacuum_control`, `/door_control`
- **Serial**: `/dev/ttyACM0` (configurable)
- **Safety**: Automatic timeout and emergency stop

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
│   │   │   ├── arduino_controller.py   # Arduino communication node
│   │   │   ├── battery_monitor.py      # Battery monitoring node
│   │   │   └── sucky_joy.py            # PS4 controller integration
│   │   ├── tools/                      # Utility scripts and tools
│   │   └── test/                       # Test files
│   ├── sucky_arduino/                  # Arduino PlatformIO project
│   │   ├── src/
│   │   │   └── main.cpp                # Arduino vacuum/servo controller
│   │   ├── include/                    # Arduino headers
│   │   ├── lib/                        # Arduino libraries
│   │   └── platformio.ini              # PlatformIO configuration
│   ├── roboclaw_hardware_interface/    # Motor controller interface
│   └── roboclaw_serial/                # Serial communication library
├── media/                              # Images and documentation
├── build.sh                            # Build script
├── setup.sh                            # Initial setup script
├── launch_sucky.sh                     # Robot launch script
├── launch_host.sh                      # Host launch script
└── README.md                           # This file
```

## 🔧 Troubleshooting

### Setup Issues

**Missing submodules:**
```bash
git submodule update --init --recursive
```

**Serial permissions:**
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Hardware Diagnostics

**Check Arduino connection:**
```bash
ls /dev/ttyACM*  # Should show /dev/ttyACM0
```

**Check Roboclaw connection:**
```bash
ls /dev/ttyACM*  # Should show /dev/ttyACM1
```

**Test LiDAR connection:**
```bash
ping 192.168.0.1
```

**Verify camera:**
```bash
lsusb | grep Intel
```

## ⚙️ Configuration

### Hardware Settings
| Component | Configuration | Default |
|-----------|---------------|---------|
| LiDAR | IP Address | `192.168.0.1` |
| Roboclaw | Serial Port | `/dev/ttyACM1` |
| Arduino | Serial Port | `/dev/ttyACM0` |
| Battery | Voltage Range | 22.0V - 29.4V |

### Performance Tuning
- **Max Speed**: Configurable in `config/sucky_controllers.yaml`
- **Safety Limits**: Battery thresholds and timeouts

## 🙏 Acknowledgments

Special thanks to:
- ROS2 Community & Open Source Robotics Foundation
- SICK AG for LiDAR technology
- Intel for RealSense camera support
- Arduino Community for embedded control resources
- Roboclaw driver contributors

---

**Questions or Issues?** Please open an issue or contact the maintainers.

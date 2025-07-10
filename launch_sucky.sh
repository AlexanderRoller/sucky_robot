#!/usr/bin/env bash

colcon build --symlink-install
source install/setup.bash
ros2 launch sucky sucky_launch.py 
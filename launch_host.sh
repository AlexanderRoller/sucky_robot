#!/usr/bin/env bash

colcon build --symlink-install
source install/setup.bash
clear
ros2 launch sucky host.launch.py 
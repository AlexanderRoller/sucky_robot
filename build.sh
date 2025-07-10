#!/usr/bin/env bash

rosdep install --from-paths src --ignore-src -r -y --skip-keys="roboclaw_hardware_interface roboclaw_serial"
colcon build --symlink-install
source install/setup.bash


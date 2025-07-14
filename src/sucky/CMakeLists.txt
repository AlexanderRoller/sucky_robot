cmake_minimum_required(VERSION 3.8)
project(sucky)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(controller_manager REQUIRED)
find_package(xacro REQUIRED)
find_package(sick_scan_xd REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(robot_localization REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# Install directories
install(
  DIRECTORY launch config meshes urdf nodes tools test
  DESTINATION share/${PROJECT_NAME}
)

# Install Python nodes
install(PROGRAMS
  nodes/battery_monitor.py
  nodes/cyclone_controller.py
  nodes/servo_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install utility tools
install(PROGRAMS
  tools/plc/getClock.py
  tools/plc/setClock.py
  tools/plc/troubleshoot_plc.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cd ~/ros2_ws/src/robot_legion_teleop_python

cat << 'EOF' > CMakeLists.txt
cmake_minimum_required(VERSION 3.5)
project(robot_legion_teleop_python)

# Minimal CMake for an ament_python-based package.

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install the Python package directory: robot_legion_teleop_python/
ament_python_install_package(${PROJECT_NAME})

# Install the teleop script into lib/${PROJECT_NAME}/ so that
# "ros2 run robot_legion_teleop_python legion_teleop_key" works.
install(PROGRAMS
  robot_legion_teleop_python/teleop_legion_key.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
EOF

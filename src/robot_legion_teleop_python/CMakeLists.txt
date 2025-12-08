cmake_minimum_required(VERSION 3.5)
project(robot_legion_teleop_python)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Install the Python package directory: robot_legion_teleop_python/
ament_python_install_package(${PROJECT_NAME})

# Install useful Python scripts as executables in lib/${PROJECT_NAME}
install(PROGRAMS
  robot_legion_teleop_python/teleop_legion_key.py
  robot_legion_teleop_python/reset_gz.py
  robot_legion_teleop_python/fpv_camera_mux.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

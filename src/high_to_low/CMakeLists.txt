cmake_minimum_required(VERSION 3.5)
project(high_to_low)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Handler1001.msg"
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/high_to_low/serial_json.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
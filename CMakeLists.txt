cmake_minimum_required(VERSION 3.8)
project(twist_pid_controller)

# Default to C++17 if not specified
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PidDebug.msg"
  DEPENDENCIES geometry_msgs std_msgs
 )

# Export message runtime dependencies
ament_export_dependencies(rosidl_default_runtime)

# Include directories (if needed)
include_directories(
  include
)

# Add executable
add_executable(twist_pid_controller_node src/twist_pid_controller.cpp)

# Add dependencies to the executable
ament_target_dependencies(twist_pid_controller_node
  rclcpp
  geometry_msgs
  std_msgs
)

# Link the generated message interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(twist_pid_controller_node ${cpp_typesupport_target})

# Install targets
install(TARGETS twist_pid_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

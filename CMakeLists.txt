cmake_minimum_required(VERSION 3.8)
project(twist_pid_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PidDebug.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(twist_pid_controller src/twist_pid_controller.cpp)

ament_target_dependencies(twist_pid_controller rclcpp geometry_msgs std_msgs)

# Ensure the generated messages are available before building the executable
add_dependencies(twist_pid_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${rosidl_generated_interfaces})


install(TARGETS twist_pid_controller
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
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

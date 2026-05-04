cmake_minimum_required(VERSION 3.5)
project(amr_sweeper_steadydrive)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  find_library(
    builtin_interfaces__rosidl_generator_c_LIB NAMES builtin_interfaces__rosidl_generator_c
    PATHS "/opt/ros/humble/lib"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(
  steadydrive_hw_interface_lib
  SHARED
  src/robot_hardware_interface_steadydrive.cpp
)

target_include_directories(
  steadydrive_hw_interface_lib
  PRIVATE
  include
)

ament_target_dependencies(
  steadydrive_hw_interface_lib
  hardware_interface
  pluginlib
  rclcpp
  sensor_msgs
  std_msgs
)

pluginlib_export_plugin_description_file(hardware_interface src/hardware.xml)

add_executable(
  steadydrive_can_node
  src/steadydrive_can_node.cpp
)
target_include_directories(
  steadydrive_can_node
  PRIVATE
  include
)
ament_target_dependencies(
  steadydrive_can_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

add_executable(steadydrive_node src/steadydrive_node.cpp)
ament_target_dependencies(steadydrive_node rclcpp)

# INSTALL
install(
  TARGETS steadydrive_hw_interface_lib steadydrive_can_node steadydrive_node
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  steadydrive_hw_interface_lib
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()

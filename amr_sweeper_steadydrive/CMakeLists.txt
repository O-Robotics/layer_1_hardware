cmake_minimum_required(VERSION 3.5)
project(amr_sweeper_steadydrive)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

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
)

pluginlib_export_plugin_description_file(hardware_interface src/hardware.xml)

# INSTALL
install(
  TARGETS steadydrive_hw_interface_lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
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

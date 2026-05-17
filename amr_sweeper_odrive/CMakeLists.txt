cmake_minimum_required(VERSION 3.8)
project(amr_sweeper_odrive)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(
  odrive_ros2_control_plugin
  SHARED
  src/odrive_hardware_interface.cpp
)

target_include_directories(
  odrive_ros2_control_plugin
  PRIVATE
  include
)

ament_target_dependencies(
  odrive_ros2_control_plugin
  ament_index_cpp
  hardware_interface
  pluginlib
  rclcpp
  yaml-cpp
)

target_link_libraries(
  odrive_ros2_control_plugin
  yaml-cpp
)

pluginlib_export_plugin_description_file(hardware_interface src/hardware.xml)

install(
  TARGETS odrive_ros2_control_plugin
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config resources
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES src/hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  odrive_ros2_control_plugin
)
ament_export_dependencies(
  ament_index_cpp
  hardware_interface
  pluginlib
  rclcpp
  yaml-cpp
)

ament_package()

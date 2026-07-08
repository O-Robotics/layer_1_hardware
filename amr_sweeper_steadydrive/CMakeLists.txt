cmake_minimum_required(VERSION 3.5)
project(amr_sweeper_steadydrive)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(amr_sweeper_safety_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(
  steadydrive_hw_interface_lib
  SHARED
  src/steadydrive_node.cpp
)

target_include_directories(
  steadydrive_hw_interface_lib
  PRIVATE
  include
)

ament_target_dependencies(
  steadydrive_hw_interface_lib
  ament_index_cpp
  amr_sweeper_safety_msgs
  hardware_interface
  pluginlib
  rclcpp
  std_srvs
  yaml-cpp
)

target_link_libraries(
  steadydrive_hw_interface_lib
  yaml-cpp
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
install(
  DIRECTORY config resources
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES src/hardware.xml
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
  ament_index_cpp
  amr_sweeper_safety_msgs
  hardware_interface
  pluginlib
  rclcpp
  std_srvs
  yaml-cpp
)

ament_package()

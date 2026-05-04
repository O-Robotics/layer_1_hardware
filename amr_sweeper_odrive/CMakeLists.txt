cmake_minimum_required(VERSION 3.8)
project(amr_sweeper_odrive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlMessage.msg"
  "msg/ControllerStatus.msg"
  "msg/ODriveStatus.msg"
  "srv/AxisState.srv"
)
ament_export_dependencies(rosidl_default_runtime)

include_directories(include)
include_directories(odrive_node/include)
include_directories(odrive_base/include)

add_executable(odrive_node
  odrive_base/src/epoll_event_loop.cpp
  odrive_base/src/socket_can.cpp
  odrive_node/src/odrive_can_node.cpp
  odrive_node/src/main.cpp
  odrive_node/include/odrive_can_node.hpp
)

ament_target_dependencies(odrive_node
  rclcpp
)

target_compile_features(odrive_node PRIVATE cxx_std_20)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(odrive_node "${cpp_typesupport_target}")

ament_auto_add_library(
  odrive_ros2_control_plugin SHARED
  odrive_base/src/epoll_event_loop.cpp
  odrive_base/src/socket_can.cpp
  odrive_ros2_control/src/odrive_hardware_interface.cpp
)

ament_target_dependencies(odrive_ros2_control_plugin
  rclcpp
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface odrive_hardware_interface.xml)

target_compile_features(odrive_ros2_control_plugin PRIVATE cxx_std_20)

install(TARGETS
  odrive_node
  odrive_ros2_control_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

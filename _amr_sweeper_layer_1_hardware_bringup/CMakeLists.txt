cmake_minimum_required(VERSION 3.8)
project(amr_sweeper_layer_1_hardware_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(layer_1_hardware_bringup_node
  src/layer_1_hardware_bringup_node.cpp
)
target_include_directories(layer_1_hardware_bringup_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(layer_1_hardware_bringup_node PUBLIC cxx_std_17)
ament_target_dependencies(
  layer_1_hardware_bringup_node
  rclcpp
  ament_index_cpp
  controller_manager_msgs
)
target_link_libraries(layer_1_hardware_bringup_node yaml-cpp)

install(TARGETS layer_1_hardware_bringup_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

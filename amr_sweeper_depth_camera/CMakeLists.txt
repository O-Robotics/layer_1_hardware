cmake_minimum_required(VERSION 3.8)
project(amr_sweeper_depth_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(domain_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(laserscan_node
  src/laserscan_node.cpp
)

add_executable(depth_camera_node
  src/depth_camera_node.cpp
)

target_include_directories(laserscan_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(depth_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(laserscan_node
  image_geometry::image_geometry
  opencv_core
)

ament_target_dependencies(laserscan_node
  rclcpp
  sensor_msgs
)

ament_target_dependencies(depth_camera_node
  ament_index_cpp
  domain_bridge
  rclcpp
  rcl_interfaces
  realsense2_camera_msgs
  std_msgs
  std_srvs
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS laserscan_node depth_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

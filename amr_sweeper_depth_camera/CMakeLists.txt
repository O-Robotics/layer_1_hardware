cmake_minimum_required(VERSION 3.16)
project(amr_sweeper_depth_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(ExternalProject)

find_package(ament_cmake REQUIRED)

set(LIBREALSENSE_GIT_REPOSITORY
  "https://github.com/realsenseai/librealsense.git"
  CACHE STRING
  "Upstream librealsense Git repository.")
set(LIBREALSENSE_GIT_TAG
  "master"
  CACHE STRING
  "Git ref for librealsense.")
set(REALSENSE_ROS_GIT_REPOSITORY
  "https://github.com/realsenseai/realsense-ros.git"
  CACHE STRING
  "Upstream realsense-ros Git repository.")
set(REALSENSE_ROS_GIT_TAG
  "ros2-master"
  CACHE STRING
  "Git ref for realsense-ros.")
set(DEPTHIMAGE_TO_LASERSCAN_GIT_REPOSITORY
  "https://github.com/ros-perception/depthimage_to_laserscan.git"
  CACHE STRING
  "Upstream depthimage_to_laserscan Git repository.")
set(DEPTHIMAGE_TO_LASERSCAN_GIT_TAG
  "ros2"
  CACHE STRING
  "Git ref for depthimage_to_laserscan.")

set(vendor_root "${CMAKE_CURRENT_BINARY_DIR}/vendor")
set(vendor_src "${vendor_root}/src")
set(vendor_build "${vendor_root}/build")
set(vendor_install "${vendor_root}/install")
set(vendor_dummy_source "${vendor_root}/depth_camera_ros_vendor_dummy")
file(MAKE_DIRECTORY "${vendor_root}" "${vendor_src}" "${vendor_build}" "${vendor_install}" "${vendor_dummy_source}")

set(build_librealsense_script "${CMAKE_CURRENT_BINARY_DIR}/build_librealsense_vendor.sh")
set(build_ros_vendor_script "${CMAKE_CURRENT_BINARY_DIR}/build_ros_vendor_workspace.sh")

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/build_librealsense_vendor.sh.in"
  "${build_librealsense_script}"
  @ONLY
)

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/build_ros_vendor_workspace.sh.in"
  "${build_ros_vendor_script}"
  @ONLY
)

ExternalProject_Add(librealsense_vendor
  PREFIX "${vendor_build}/librealsense_vendor"
  GIT_REPOSITORY "${LIBREALSENSE_GIT_REPOSITORY}"
  GIT_TAG "${LIBREALSENSE_GIT_TAG}"
  SOURCE_DIR "${vendor_src}/librealsense"
  UPDATE_DISCONNECTED 1
  CONFIGURE_COMMAND ""
  BUILD_COMMAND bash "${build_librealsense_script}"
  INSTALL_COMMAND ""
  BUILD_BYPRODUCTS
    "${vendor_install}/lib/librealsense2.so"
)

ExternalProject_Add(depth_camera_ros_vendor
  PREFIX "${vendor_build}/depth_camera_ros_vendor"
  DOWNLOAD_COMMAND ""
  UPDATE_COMMAND ""
  PATCH_COMMAND ""
  SOURCE_DIR "${vendor_dummy_source}"
  DEPENDS librealsense_vendor
  CONFIGURE_COMMAND ""
  BUILD_COMMAND bash "${build_ros_vendor_script}"
  INSTALL_COMMAND ""
  BUILD_BYPRODUCTS
    "${vendor_install}/share/realsense2_camera/package.xml"
    "${vendor_install}/share/depthimage_to_laserscan/package.xml"
)

add_custom_target(${PROJECT_NAME}_vendor ALL
  DEPENDS depth_camera_ros_vendor
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY "${vendor_install}/"
  DESTINATION "."
  USE_SOURCE_PERMISSIONS
)

ament_package()

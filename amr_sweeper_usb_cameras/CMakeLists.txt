cmake_minimum_required(VERSION 3.5)
project(amr_sweeper_usb_cameras)

# The package is small and modern C++17 is sufficient for its needs.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Keep warnings strict because this package touches low-level device and memory code.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec REQUIRED libavcodec)
pkg_check_modules(avutil REQUIRED libavutil)
pkg_check_modules(swscale REQUIRED libswscale)

add_library(${PROJECT_NAME} SHARED
  src/amr_sweeper_usb_camera.cpp
)

# Export the public headers for the reusable low-level camera driver library.
target_include_directories(${PROJECT_NAME} PUBLIC
  "include"
  ${camera_info_manager_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${avutil_LIBRARIES}
  ${swscale_LIBRARIES})

ament_export_libraries(${PROJECT_NAME})

# Build the ROS 2 component wrapper separately from the low-level driver library.
ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/amr_sweeper_usb_camera_node.cpp
)

target_link_libraries(${PROJECT_NAME}_component
  ${PROJECT_NAME})

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "amr_sweeper_usb_cameras::UsbCameraNode"
  EXECUTABLE amr_sweeper_usb_cameras_node
)

# The generated executable target needs the C++ standard library explicitly linked here.
target_link_libraries(amr_sweeper_usb_cameras_node stdc++)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
    config
)

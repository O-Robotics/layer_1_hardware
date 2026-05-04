cmake_minimum_required(VERSION 3.8)
project(amr_sweeper_gnss)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_test REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(CURL REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb REQUIRED libusb-1.0)

file(GLOB GNSS_MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")
file(GLOB GNSS_SRV_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${GNSS_MSG_FILES}
  ${GNSS_SRV_FILES}
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

include_directories(
  include
  ntrip_client_node/include
  ublox_dgnss_node/include
  ublox_nav_sat_fix_hp_node/include
)

add_library(ntrip_client_components SHARED
  ntrip_client_node/src/ntrip_client_node.cpp
)

target_compile_definitions(ntrip_client_components
  PRIVATE "NTRIP_CLIENT_NODE_BUILDING_DLL"
)

ament_target_dependencies(ntrip_client_components
  rclcpp
  rclcpp_components
  std_msgs
)

target_link_libraries(ntrip_client_components
  ${cpp_typesupport_target}
  CURL::libcurl
)

rclcpp_components_register_node(ntrip_client_components
  PLUGIN "amr_sweeper_ublox_dgnss::NTRIPClientNode"
  EXECUTABLE amr_sweeper_ntrip_client_node
)

add_library(ublox_dgnss_components SHARED
  ublox_dgnss_node/src/usb.cpp
  ublox_dgnss_node/src/ublox_dgnss_node.cpp
)

target_compile_definitions(ublox_dgnss_components
  PRIVATE "UBLOX_DGNSS_NODE_BUILDING_DLL"
)

ament_target_dependencies(ublox_dgnss_components
  rclcpp
  rclcpp_components
  std_msgs
)

target_link_libraries(ublox_dgnss_components
  usb-1.0
  ${cpp_typesupport_target}
)

rclcpp_components_register_node(ublox_dgnss_components
  PLUGIN "amr_sweeper_ublox_dgnss::UbloxDGNSSNode"
  EXECUTABLE amr_sweeper_ublox_dgnss_node
)

add_library(ublox_nav_sat_fix_hp_components SHARED
  ublox_nav_sat_fix_hp_node/src/ublox_nav_sat_fix_hp_node.cpp
)

ament_target_dependencies(ublox_nav_sat_fix_hp_components
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
)

target_link_libraries(ublox_nav_sat_fix_hp_components
  ${cpp_typesupport_target}
)

rclcpp_components_register_node(ublox_nav_sat_fix_hp_components
  PLUGIN "ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode"
  EXECUTABLE ublox_nav_sat_fix_hp
)

install(TARGETS
  ntrip_client_components
  ublox_dgnss_components
  ublox_nav_sat_fix_hp_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

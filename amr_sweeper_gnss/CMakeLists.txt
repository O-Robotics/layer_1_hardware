cmake_minimum_required(VERSION 3.8)
project(amr_sweeper_gnss)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rtcm_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(amr_sweeper_gnss_ublox_node
  src/amr_sweeper_gnss_node.cpp
)
set_target_properties(amr_sweeper_gnss_ublox_node PROPERTIES OUTPUT_NAME gnss_ublox_node)

target_compile_features(amr_sweeper_gnss_ublox_node PUBLIC cxx_std_17)
target_include_directories(amr_sweeper_gnss_ublox_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(amr_sweeper_gnss_ublox_node
  rclcpp
  rtcm_msgs
  sensor_msgs
)

add_executable(ntrip_client
  src/ntrip_client_node.cpp
)

target_compile_features(ntrip_client PUBLIC cxx_std_17)
target_include_directories(ntrip_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ntrip_client
  OpenSSL::SSL
  OpenSSL::Crypto
  yaml-cpp
)
ament_target_dependencies(ntrip_client
  rclcpp
  rtcm_msgs
  sensor_msgs
)

install(TARGETS ntrip_client
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS amr_sweeper_gnss_ublox_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()

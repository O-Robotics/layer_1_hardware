cmake_minimum_required(VERSION 3.16)
project(amr_sweeper_microros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(ExternalProject)

find_package(ament_cmake REQUIRED)

set(MICRO_XRCE_DDS_AGENT_GIT_REPOSITORY
  "https://github.com/eProsima/Micro-XRCE-DDS-Agent.git"
  CACHE STRING
  "Upstream Micro XRCE-DDS Agent Git repository.")
set(MICRO_XRCE_DDS_AGENT_GIT_TAG
  "v3.0.1"
  CACHE STRING
  "Git ref for Micro XRCE-DDS Agent.")

set(vendor_root "${CMAKE_CURRENT_BINARY_DIR}/vendor")
set(vendor_src "${vendor_root}/src")
set(vendor_build "${vendor_root}/build")
set(vendor_install "${vendor_root}/install")
set(vendor_dummy_source "${vendor_root}/microros_vendor_dummy")
file(MAKE_DIRECTORY "${vendor_root}" "${vendor_src}" "${vendor_build}" "${vendor_install}" "${vendor_dummy_source}")

set(build_microros_script "${CMAKE_CURRENT_BINARY_DIR}/build_microros_vendor.sh")

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/build_microros_vendor.sh.in"
  "${build_microros_script}"
  @ONLY
)

ExternalProject_Add(microros_vendor
  PREFIX "${vendor_build}/microros_vendor"
  DOWNLOAD_COMMAND ""
  UPDATE_COMMAND ""
  PATCH_COMMAND ""
  SOURCE_DIR "${vendor_dummy_source}"
  CONFIGURE_COMMAND ""
  BUILD_COMMAND bash "${build_microros_script}"
  INSTALL_COMMAND ""
  BUILD_BYPRODUCTS
    "${vendor_install}/lib/${PROJECT_NAME}/amr_sweeper_microros_classic_can_agent"
    "${vendor_install}/share/microxrcedds_agent/cmake/microxrcedds_agentConfig.cmake"
)

add_custom_target(${PROJECT_NAME}_vendor ALL
  DEPENDS microros_vendor
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY "${vendor_install}/"
  DESTINATION "."
  USE_SOURCE_PERMISSIONS
)

ament_package()

cmake_minimum_required(VERSION 3.16)
project(amr_sweeper_microros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(microxrcedds_agent CONFIG REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(amr_sweeper_microros_classic_can_agent
  src/classic_can_agent.cpp
)

target_link_libraries(amr_sweeper_microros_classic_can_agent
  PRIVATE
    microxrcedds_agent
)

ament_target_dependencies(amr_sweeper_microros_classic_can_agent
  rclcpp
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  amr_sweeper_microros_classic_can_agent
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

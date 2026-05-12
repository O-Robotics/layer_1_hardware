cmake_minimum_required(VERSION 3.16)
project(amr_sweeper_microros_classic_can_agent LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(microxrcedds_agent CONFIG REQUIRED)

add_executable(amr_sweeper_microros_classic_can_agent
  ../../src/classic_can_agent.cpp
)

target_link_libraries(amr_sweeper_microros_classic_can_agent
  PRIVATE
    microxrcedds_agent
)

install(TARGETS amr_sweeper_microros_classic_can_agent
  DESTINATION lib/amr_sweeper_microros
)

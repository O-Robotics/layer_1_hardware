cmake_minimum_required(VERSION 3.16)
project(amr_sweeper_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  launch/gz_pose_to_gps.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gz_pose_to_gps
)

install(PROGRAMS
  scripts/simulate_programmed_missions.sh
  scripts/set_gazebo_simulation_speed.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

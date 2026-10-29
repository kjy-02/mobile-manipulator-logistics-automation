cmake_minimum_required(VERSION 3.0.2)
project(scout_dwa)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  message_generation
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf
  visualization_msgs
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

add_message_files(FILES IntegrationHealth.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS diagnostic_msgs geometry_msgs message_runtime nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_msgs std_srvs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(scout_improved_dwa
  src/improved_dwa/improved_dwa_planner.cpp
  src/improved_dwa/parameters.cpp
  src/improved_dwa/node.cpp)
target_compile_options(scout_improved_dwa PRIVATE -std=c++17 -O2)
target_link_libraries(scout_improved_dwa ${catkin_LIBRARIES})

add_executable(cloud_obstacle_adapter src/cloud_obstacle_adapter.cpp)
add_dependencies(cloud_obstacle_adapter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cloud_obstacle_adapter ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(static_footprint_publisher src/static_footprint_publisher.cpp)
target_link_libraries(static_footprint_publisher ${catkin_LIBRARIES})

add_executable(cmd_vel_guard src/cmd_vel_guard.cpp)
add_dependencies(cmd_vel_guard ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cmd_vel_guard ${catkin_LIBRARIES})

install(TARGETS cloud_obstacle_adapter static_footprint_publisher cmd_vel_guard scout_improved_dwa
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/command_watchdog.py
  scripts/run_goal_navigation.py
  scripts/run_short_trial.py
  scripts/bounded_drive_test.py
  scripts/trajectory_display.py
  scripts/input_gateway.py
  scripts/planner_preflight.py
  scripts/analyze_sampling_limits.py
  scripts/read_only_preflight.py
  scripts/localization_monitor.py
  scripts/localization_session.py
  scripts/synthetic_inputs.py
  scripts/validate_integration_config.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/run_synthetic_tests.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES src/improved_dwa/LICENSE src/improved_dwa/README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/improved_dwa)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_config_validator.py)
  add_rostest(test/synthetic_pipeline.test)
  add_rostest(test/synthetic_scan_pipeline.test)
  add_rostest(test/synthetic_collision_pipeline.test)
  add_rostest(test/guard_failsafe.test)
  add_rostest(test/guard_freshness.test)
  add_rostest(test/health_guard.test)
  add_rostest(test/localization_monitor.test)
  add_rostest(test/input_gateway.test)
  add_rostest(test/cloud_faults.test)
  add_rostest(test/guard_fault_matrix.test)
  add_rostest(test/integration_preflight.test)
  add_rostest(test/planner_lifecycle.test)
  add_rostest(test/planner_circle.test)
  add_rostest(test/guard_sigkill.test)
  add_rostest(test/guard_shutdown.test)
  add_rostest(test/localization_session.test)
  add_rostest(test/localization_jump.test)
  add_rostest(test/path_cost_selection.test)
  add_rostest(test/thin_scan_limit.test)
endif()

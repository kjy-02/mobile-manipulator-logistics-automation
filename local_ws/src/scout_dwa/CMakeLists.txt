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

add_executable(cloud_obstacle_adapter src/cloud_obstacle_adapter.cpp)
add_dependencies(cloud_obstacle_adapter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cloud_obstacle_adapter ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(static_footprint_publisher src/static_footprint_publisher.cpp)
target_link_libraries(static_footprint_publisher ${catkin_LIBRARIES})

install(TARGETS cloud_obstacle_adapter static_footprint_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/command_watchdog.py
  scripts/run_goal_navigation.py
  scripts/trajectory_display.py
  scripts/localization_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

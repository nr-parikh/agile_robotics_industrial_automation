cmake_minimum_required(VERSION 2.8.3)
project(agile_robotics_industrial_automation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  osrf_gear
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
  moveit_ros_planning_interface
  tf
  geometry_msgs
  control_msgs
)

catkin_package()
# catkin_package(
#  INCLUDE_DIRS include
# #  LIBRARIES agile_robotics_industrial_automation
#  CATKIN_DEPENDS osrf_gear rocpp sensor_msgs std_msgs std_srvs trajectory_msgs tf moveit_ros_planning_interface geometry_msgs
# #  DEPENDS system_lib
# )

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(qual1_node src/qual1.cpp src/ur10_controller.cpp src/sensor.cpp src/order_manager.cpp)
# add_executable(qual1_node src/qual1.cpp src/ur10_controller.cpp src/sensor_2b.cpp src/order_manager.cpp)
# add_executable(qual1_node src/qual1.cpp src/ur10_controller.cpp src/sensor_2b.cpp src/order_manager_2b.cpp)
add_executable(pose_node src/pose_broadcaster.cpp src/sensor.cpp)

add_dependencies(qual1_node ${catkin_EXPORTED_TARGETS})
add_dependencies(pose_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(qual1_node
  ${catkin_LIBRARIES}
)
target_link_libraries(pose_node
  ${catkin_LIBRARIES}
)

install(PROGRAMS
  scripts/qual1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS qual1_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
 config/team_config_2b.yaml
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

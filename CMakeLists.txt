cmake_minimum_required(VERSION 3.0.2)
project(ur5e_tutorials)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  tf
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    moveit_core
    moveit_ros_planning_interface
    tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/wiggle.py scripts/pick_and_place.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

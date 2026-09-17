cmake_minimum_required(VERSION 3.0.2)
project(amr_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  move_base
  costmap_2d
  dwa_local_planner
  teb_local_planner
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    tf2_ros
    move_base
    costmap_2d
    dwa_local_planner
    teb_local_planner
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

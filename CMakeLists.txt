cmake_minimum_required(VERSION 3.0.2)
project(me48b_library_robot)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
)

catkin_package()

install(DIRECTORY launch models worlds maps routes config scripts urdf rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

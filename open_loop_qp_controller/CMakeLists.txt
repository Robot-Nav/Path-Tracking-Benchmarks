cmake_minimum_required(VERSION 3.1)
project(open_loop_qp_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs nav_msgs rospy std_msgs)

catkin_install_python(PROGRAMS
  scripts/global_open_loop_qp_controller.py
  scripts/geometric_tracking_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES LICENSE DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

cmake_minimum_required(VERSION 3.16)



find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   sensor_msgs
   geometry_msgs
   visualization_msgs

   
)
project( motor_controller )
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs visualization_msgs
)

catkin_install_python(PROGRAMS scripts/motor_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
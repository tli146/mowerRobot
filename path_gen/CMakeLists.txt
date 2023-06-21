cmake_minimum_required(VERSION 3.16)



find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   sensor_msgs
   geometry_msgs
   visualization_msgs

   
)
project( path_gen )
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs visualization_msgs
)

catkin_install_python(PROGRAMS scripts/path_gen.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
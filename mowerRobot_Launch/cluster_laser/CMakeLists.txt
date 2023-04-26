cmake_minimum_required(VERSION 3.16)



find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   sensor_msgs

   
)
project( metr4202_transforms )
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
)

catkin_install_python(PROGRAMS scripts/cluster_laser.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
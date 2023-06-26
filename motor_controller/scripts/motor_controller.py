#!/usr/bin/env python

#this class will process trees and obstacles to generate a path for the robotd

import rospy


import numpy as np
import subprocess as p

from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point, Point32, Pose, Quaternion
from std_msgs.msg import ColorRGBA, Bool, Float32, Int16
from visualization_msgs.msg import  Marker

#global variables
ros_rate = 5

#mower variables
wb = 1.3
min_turn_radius = 1.5
max_wheel_angle = np.pi/4
min_wheel_angle = -np.pi/4

#feedback variables
sensor_right = 3540
sensor_left = 571
sensor_forward = 2108



#tuning variables

class motor_controller:
    def __init__(self) -> None:
        self.curvature = 0
        self.x_target = 0
        self.y_target = 0
        self.motor_status = 0
        self.active = False
        
        self.sub_heading= rospy.Subscriber(
            #subscribe to cluster output
            "path",
            Marker,
            self.heading_callback       
        )

        self.publish_curv = rospy.Publisher(
            #publishes desired turning radius
            'curvature',
            Float32
        )

        self.publish_wheel_angle = rospy.Publisher(
            #publishes desired wheel turn angle
            'wheel_angle',
            Float32
        )

        self.publish_motor_value = rospy.Publisher(
            #publishes output to wheel
            'wheel_angle',
            Int16
        )

    
    def check_motor_status(self):
        x = p.run(["jrk2cmd", "--status"])
        print(x)


    def motor_value_calc(self, theta):
        if np.abs(theta) > max_wheel_angle*1.2:
            return -1
        if np.abs(theta) < 0.1:
            return sensor_forward
        if theta> 0:
            #right turn
            motor_value = theta * (sensor_right-sensor_forward)/max_wheel_angle + sensor_forward
            return int(motor_value)
        if theta< 0:
            #left turn
            motor_value = theta * (sensor_forward - sensor_left)/max_wheel_angle + sensor_forward
            return int(motor_value)

    
    def heading_callback(self, marker):
        self.active = True
        point = marker.points[1]
        x = point.x
        y = point.y
        dist2 = np.square(x)+np.square(y)
        curv = 2*y/dist2
        theta = np.arctan(wb*curv)

        
        motor_value = self.motor_value_calc(theta)
        print(motor_value)
        curv_msg = Float32()
        curv_msg.data = curv
        self.publish_curv.publish(curv_msg)

        theta_msg = Float32()
        theta_msg.data = theta
        self.publish_wheel_angle.publish(theta_msg)

        motor_msg = Int16()
        motor_msg.data = motor_value
        self.publish_motor_value.publish(motor_msg)
        self.set_motor(motor_value)

        

    def set_motor(self, motor_value):
        
        p.run(["jrk2cmd", "--target", str(motor_value)])






if __name__ == '__main__':

    rospy.init_node('motor_controller')
    motor_controller = motor_controller()
    #set frequency to increase performance
    rate = rospy.Rate(ros_rate)
    


    while not rospy.is_shutdown():

            
        rate.sleep()



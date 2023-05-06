#!/usr/bin/env python

#this class will take in lidar msgs and return: list of tree matching obstacles and whether there is an obstacle within collision distance of the robot

from select import select
import rospy
import math
import heapq
from scipy.spatial.transform import Rotation as R
import numpy as np

from sensor_msgs import LaserScan, PointCloud
from geometry_msgs import Point, Point32, Pose, Quaternion
from std_msgs.msg import Header, String, Int16, Bool, Float32, ColorRGBA
from visualization_msgs import  Marker

#constants

#ros constants
ros_rate = 5

#detected object data
bollard_diameter = 90/1000 #meter

diameter_max = bollard_diameter*1.5
diameter_min = bollard_diameter*0.5

obj_link_min_length_wo_distance_adjust = 50/1000 #m

wallLength_max = 0.5 #meter

#safety data
safety_cone = np.deg2rad(15) #rad
safety_base = 1 #meter

safety_distance = 2 #meter
#distance to apply brake

safety_warning = 5 #meter
#distance to apply slow

lidar_heading_modifier = np.deg2rad(0) #rad
#lidar heading tuning


#degrees

class obstacleList:
    #takes in laserscan results and generates obstacle list and bollards list
    def __init__(self, angle_min, angle_max, angle_increment, range_min, range_max, ranges):
        self.location = [0,0]
        currentAngle = angle_min
        x_last_obj = ranges[0]* np.sin(currentAngle)
        y_last_obj = ranges[0]* np.cos(currentAngle)
        x_last = x_last_obj
        y_last = y_last_obj
        new_obj = True

        obstacle_list = []
        obstacle_index = 0
        for laser in ranges:
            if(laser > range_min and laser < range_max):
                
                x = laser* np.sin(currentAngle)
                y = laser* np.cos(currentAngle)

                if(new_obj):
                    
                    obstacle_list.append(obstacle((x,y)))
                    obstacle_index = obstacle_index + 1

                else:

                    dis_from_last = np.sqrt((x-x_last)^2+(y-y_last)^2)
                    dis_from_last_obj = np.sqrt((x-x_last_obj)^2+(y-y_last_obj)^2)
                    linear_scaled_obj_min_length = (0.7+ (laser/15)*0.8)* obj_link_min_length_wo_distance_adjust
                    #scale by resolution at distance

                    if(dis_from_last < linear_scaled_obj_min_length and dis_from_last_obj < wallLength_max):

                        obstacle_list[obstacle_index].newScan(x, y)


                    else:

                        obstacle_list[obstacle_index].finalise()

                        obstacle_list.append(obstacle((x,y)))
                        obstacle_index = obstacle_index + 1

                new_obj = False
            else:
                obstacle_list[obstacle_index].finalise()
                new_obj = True
            currentAngle = currentAngle + angle_increment

        #connect front and back(if connecting)
        if obstacle_list[1].mergeFrom(obstacle_list[obstacle_index -1]):
            obstacle_index = obstacle_index - 1
            obstacle_list.pop(obstacle_index)

        #for debug    
        print(obstacle_index)



    def filter_bollards(self):
        bollard_list = []
        for obstacle in self.obstacle_list:
            if obstacle.length < diameter_max and obstacle.length > diameter_min:
                bollard_list.append(obstacle)
                obstacle.type = 1
        return bollard_list
    






#location
#      -x
#       ^
#       |
#       |
#-y <---R-----> y --->angle_min--->front
#       |
#       |
#       x
#

class obstacle:
    def __init__(self, x , y) -> None:
        self.size = 1
        self.length = 0
        self.locations_x = []
        self.locations_y = []
        self.locations_x.append(x)
        self.locations_y.append(y)
        self.x = x
        self.y = y
        self.type = 0


    def newScan(self, x, y):
        self.size = self.size + 1
        self.locations_x.append(x)
        self.locations_y.append(y)

    def finalise(self):
        #find average location
        self.x = sum(self.locations_x) / len(self.locations_x)
        self.y = sum(self.locations_y) / len(self.locations_y)
        self.length = np.sqrt((max(self.locations_x) - min(self.locations_x))^2 + (max(self.locations_y) - min(self.locations_y))^2)

    def mergeFrom(self, obstacle):
        merged_x = self.locations_x + obstacle.locations_x
        merged_y = self.locations_y + obstacle.locations_y
        new_length = np.sqrt((max(merged_x) - min(merged_x))^2 + (max(merged_y) - min(merged_y))^2)
        if new_length > wallLength_max:
            return False
        else:
            self.size = self.size + obstacle.size
            self.locations_x = merged_x
            self.locations_y = merged_y
            self.x = sum(self.locations_x) / len(self.locations_x)
            self.y = sum(self.locations_y) / len(self.locations_y)
            return True
        
    def to_point_32(self):
        point = Point32()
        point.x = self.x
        point.y = self.y
        point.z = self.length
        return point
    
    def to_point(self):
        point = Point()
        point.x = self.x
        point.y = self.y
        point.z = self.length
        return point
    
    
        




class processor:
#class of detected obstacles and bollards
    def detection_callback(self, LaserScan):
        #synchronous update on receiving new transform information
        self.laserScan = LaserScan
        obstacleList = obstacleList(LaserScan.angle_min, LaserScan.angle_max, LaserScan.angle_increment, LaserScan.range_min, LaserScan.range_max, LaserScan.ranges)
        bollard_list = obstacleList.filter_bollards()


    def publish_markers(self, obstacle_list):
        point_list = []
        color_list = []
        for obstacle in obstacle_list:
            point_list.append(obstacle.to_point())
            if(obstacle.type == 0):
                color_list.append(ColorRGBA(1.0,0.0,0.0,1.0))
            else:
                color_list.append(ColorRGBA(0.0,1.0,0.0,1.0))
        marker = Marker()
        marker.type = 8 #points type
        marker.points = point_list
        marker.pose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
        marker.colors = color_list
        self.publish_obstacle_visual(marker)


    def __init__(self):


        self.sub_laserScan= rospy.Subscriber(
            #subscribe to lidar output
            "LaserScan",
            LaserScan,
            self.detection_callback       
        )

        self.publish_obstacles = rospy.Publisher(
            #publishes list of all obstacles. z value is length
            'obstacle_list',
            PointCloud
        )

        self.publish_bollards = rospy.Publisher(
            #publishes list of all bollards. z value is length
            'bollards_list',
            PointCloud
        )

        self.publish_obstacle_visual = rospy.Publisher(
            #published list of markers for rviz
            'obstacle_list_rviz',
            Marker
        )



    
            



if __name__ == '__main__':

    rospy.init_node('cluster_laser')

    #set frequency to increase performance
    rate = rospy.Rate(ros_rate)
    scan_processor = processor()
    #instantiate object
    
    
        
    
    while not rospy.is_shutdown():
        
        rate.sleep()



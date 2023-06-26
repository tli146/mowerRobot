#!/usr/bin/env python

#this class will process trees and obstacles to generate a path for the robotd

import rospy


import numpy as np

from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point, Point32, Pose, Quaternion
from std_msgs.msg import ColorRGBA, Bool, Float32
from visualization_msgs.msg import  Marker

#global variables
ros_rate = 5



#tuning variables
MinRange = 0.1

MidRange = 20
midWidth = 8

closeRange = 10
closeWidth = 5

detectAngle = 45

biasWeight = 100
biasWeightFront = biasWeight
bollardFilterMinDist = 8

robotWidth = 1
safeStopDistance = 2
initialHeading = 0

angleForward = np.pi/2

class obstacle:
    #location
#      -y
#       ^
#       |
#       |
# y <---R-----> -x --->angle 0--->front
#       |
#       |
#       y
#

    def __init__(self, x , y) -> None:
        self.x = x
        self.y = y
        self.distance = np.sqrt(np.square(x) + np.square(y))
        self.heading = np.arccos(y/self.distance)
        self.behind = (x>= 0)
        self.priorityScore = -1
    
    def isImmediate(self) -> bool:
        if np.abs(self.y) < robotWidth:
            if self.x >0 and self.x < safeStopDistance:
                return True
        return False
    
    def update_priority_score(self, angle) -> float:
        #lower better
        priorityScore = self.distance*(1 + biasWeight*np.abs(angle - self.heading) + biasWeightFront*np.abs(angle - angleForward))
        self.priorityScore = priorityScore
        return priorityScore
    
    def inMaxBound(self):
        if np.abs(self.y) > midWidth/2:
            return False
        if np.abs(self.x) > MidRange or np.abs(self.x) < MinRange:
            return False
        if self.behind:
            return False
        return True
        
    def __lt__(self, other):
        if self.priorityScore == -1:
            return False
        if self.priorityScore < other.priorityScore:
            return True
        return False
        
    



    
        



class filter:
    def __init__(self):
        self.receiving_obstacles = False
        self.receiving_bollards = False
        self.bollard_list = []
        self.obstacle_list = []
        self.headingAngle = initialHeading   
        self.no_more_path = True       

        self.sub_bollards= rospy.Subscriber(
            #subscribe to cluster output
            "bollards_list",
            PointCloud,
            self.bollard_callback       
        )

        self.sub_obstacles = rospy.Subscriber(
            #subscribe to cluster output
            "obstacle_list",
            PointCloud,
            self.obstacle_callback    
        )

        self.publish_path = rospy.Publisher(
            #publishes next few points of navigation
            'path',
            Marker
        )

        self.publish_heading1 = rospy.Publisher(
            #publishes front of robot
            'heading1',
            Marker
        )

        self.publish_heading2 = rospy.Publisher(
            #publishes robot desired heading
            'heading2',
            Marker
        )

        self.publish_stop = rospy.Publisher(
            #publishes if going to run into obstacle
            'stop',
            Bool
        )
        self.publish_Turn = rospy.Publisher(
            #publishes if going to run into obstacle
            'turnRadius',
            Float32
        )

        self.publish_filtered = rospy.Publisher(
            #publishes filtered bollards
            'filtered',
            Marker
        )

    def obstacle_callback(self, PointCloud):
        #async
       
        self.obstacle_list = []
        for point in PointCloud.points:
            self.obstacle_list.append(obstacle(point.x, point.y))
        self.receiving_obstacles = True


    def bollard_callback(self, PointCloud):
        #async

        self.bollard_list = []
        for point in PointCloud.points:
            self.bollard_list.append(obstacle(point.x, point.y))
        self.receiving_bollards = True

    def stopSignal(self):
        stop = False
        if not self.receiving_bollards:
            return True
        
        for obstacles in self.obstacle_list:
            if obstacles.isImmediate:
                stop = True
        
        return stop
    
    def filter(self) -> list:
        filtered_list = []
        for bollard in self.bollard_list:
            if bollard.inMaxBound():
                filtered_list.append(bollard)
        self.bollard_list = filtered_list
        closeList = []
        for bollard in self.bollard_list:
            minDist2 = bollardFilterMinDist*bollardFilterMinDist*2
            for bollard2 in self.bollard_list:
                dist2 = np.square(bollard.x-bollard2.x)+np.square(bollard.y-bollard2.y)
                if dist2 < minDist2:
                    minDist2 = dist2
            if minDist2 < bollardFilterMinDist:
                closeList.append(bollard)
        
        self.bollard_list = closeList    
        return closeList
    
    def publish_filter_results(self, bollard_list):
        points = []
        for bollard in bollard_list:
            point = Point()
            point.x = bollard.x
            point.y = bollard.y
            point.z = 0
            points.append(point)
        marker = Marker()
        marker.header.frame_id = "laser"
        marker.type = 8 #points type
        marker.points = points
        marker.pose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
        marker.color = ColorRGBA(1.0,1.0,1.0,1.0)
        marker.scale.x = 0.06
        marker.scale.y = 0.06
        marker.scale.z = 1.5
        self.publish_filtered.publish(marker)
        


    
    def findCenter(self, list:list) -> list:
        x_sum = 0
        y_sum = 0
        for item in list:
            x_sum = x_sum + item.x
            y_sum = y_sum + item.y
        x = x_sum/len(list)
        y = y_sum/len(list)
        distance = np.sqrt(np.square(x) + np.square(y))
        heading = np.arccos(y/distance)
        return [x, y, distance, heading]


    def main_loop(self):
        #filter bollards
        self.filter()
        for bollard in self.bollard_list:
            bollard.update_priority_score(self.headingAngle)

        self.bollard_list.sort()
        immediate_list = []
        beyond = []
        

        if len(self.bollard_list) <4:
            return False
        if len(self.bollard_list) == 4:
            self.headingAngle = 0
            immediate_list = self.bollard_list
            self.no_more_path = True
            beyond = False
        else:   
            self.no_more_path = False
            immediate_list = self.bollard_list[0:4]
            beyond_list = self.bollard_list[2:]
            beyond = self.findCenter(beyond_list)
            self.headingAngle = beyond[3]

        self.publish_filter_results(self.bollard_list)
        prior = [0,0, 0, 0]
        next =self.findCenter(immediate_list)
        #print(next[3])
        self.publish_next(self.next_to_points(next, beyond))


        #sepperate into front 2, next 2 and others
        #update heading angle
        #find next and beyond points
        #find arc of best fit and corresponding radius
        #draw arc on rviz

    
        
    def next_to_points(self, next, beyond):
        points_list = []
        point1 = Point()
        point1.x = next[0]
        point1.y = next[1]
        point1.z = 0
        points_list.append(point1)
        if beyond == False:
            return points_list
        point2 = Point()
        point2.x = beyond[0]
        point2.y = beyond[1]
        point2.z = 0
        points_list.append(point2)
        return points_list
    
    def publish_next(self, points_list):
        #white for path

        marker = Marker()
        marker.header.frame_id = "laser"
        marker.type = 8 #points type
        marker.points = points_list
        marker.pose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
        marker.colors = [ColorRGBA(1.0,1.0,0.5,1.0),ColorRGBA(1.0,1.0,0.0,1.0)]
        marker.scale.x = 0.06
        marker.scale.y = 0.06
        marker.scale.z = 1.5
        self.publish_path.publish(marker)
    
    def publish_headings(self):
        point0 = Point()
        point0.x = 0
        point0.y = 0
        point0.z = 0

        point1 = Point()
        point1.x = 0
        point1.y = 1
        point1.z = 0

        point2 = Point()
        point2.x = -np.sin(self.headingAngle)*1.2
        point2.y = np.cos(self.headingAngle)*1.2
        point2.z = 0

        marker1 = Marker()
        marker1.header.frame_id = "laser"
        marker1.type = 0 #arrow type
        marker1.pose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
        marker1.scale.x = 0.1
        marker1.scale.y = 0.2
        marker1.scale.z = 0
        marker1.points = [point0, point1]
        marker1.color = ColorRGBA(1.0,1.0,1.0,1.0)

        marker2 = Marker()
        marker2.header.frame_id = "laser"
        marker2.type = 0 #arrow type
        marker2.pose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
        marker2.scale.x = 0.1
        marker2.scale.y = 0.2
        marker2.scale.z = 0
        marker2.points = [point0, point2]
        marker2.color = ColorRGBA(1.0,1.0,1.0,1.0)

        self.publish_heading1.publish(marker1)
        self.publish_heading2.publish(marker2)





    




if __name__ == '__main__':

    rospy.init_node('path_gen')

    #set frequency to increase performance
    rate = rospy.Rate(ros_rate)
    bollard_filter = filter()

    #instantiate object
    
    
        
    


    while not rospy.is_shutdown():
        
        if bollard_filter.receiving_bollards and bollard_filter.receiving_obstacles:
            bollard_filter.main_loop()
            bollard_filter.publish_headings()
            
        rate.sleep()



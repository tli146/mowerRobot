#!/usr/bin/env python

from select import select
import rospy
import math
import heapq
from scipy.spatial.transform import Rotation as R
import numpy as np
import modern_robotics as mr

from metr4202_msgs.msg import block as Block
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from geometry_msgs.msg import Transform
from std_msgs.msg import Header, String, Int16, Bool

#constants
calibration_ID = 2
ros_rate = 5
rotation_theta_threshold = 5 
#degrees

class DetectedBlock:
#class of detected aruco code blocks
    def __init__(self, id, Transf, Toc) -> None:
        rotQ = Transf.rotation
        transQ = Transf.translation
        rotM = R.from_quat([rotQ.x,rotQ.y,rotQ.z,rotQ.w] )
        rotM = R.as_matrix(rotM)
        transM = np.array([transQ.x, transQ.y, transQ.z])
        Tca = mr.RpToTrans(rotM, transM)
        #calculate transformation matrices of the block in camera frame


        Toa = np.dot(Toc,Tca)
        #calculate transformation matrices of the block in robot frame

        r, p = mr.TransToRp(Toa)

        self.id = id
        self.coordinate = np.multiply(p, 1000)
        #convert to mm

        theta_1 = int(np.rad2deg(np.arctan2(r[0][0], r[0][1])))
        theta_1 = theta_1%90


        if(theta_1 >45):
            theta_1 = 90 - abs(theta_1)
        theta_2 = int(np.rad2deg(np.arctan2(p[1], p[0])))

        self.theta = np.abs(theta_1 - theta_2)
        #find difference in angle between block and gripper

        self.absTheta = int(np.rad2deg(theta_1)%45)
        #find actual angle of the block to robot axis

        self.priority = 0




    def setColor(self, color:int):
        self.color = color
    
    def setPriority(self, priority: int):
        if self.id == calibration_ID:
            return False
        self.priority = priority 

    def toMsg(self) -> Block:
        #convert block to block message
        msg = Block()
        msg.id = self.id
        msg.x = int(self.coordinate[0])
        msg.y = int(self.coordinate[1])
        msg.z = int(self.coordinate[2])
        msg.theta = self.theta
        
        return msg


    #priority comparators for block
    def _is_valid_operand(self, other):
        return (hasattr(other, "priority"))

    def __eq__(self, other):
        if not self._is_valid_operation(other):
            return NotImplemented
        return self.priority == other.priority

    
    def __lt__(self, other):
        if not self._is_valid_operation(other):
            return NotImplemented
        return self.priority < other.priority

    def __gt__(self, other):
        if not self._is_valid_operation(other):
            return NotImplemented
        return self.priority > other.priority

    def _is_valid_operation(self, other):
        return type(self) == type(other)



class DetectBlock:
    def detection_callback(self, fiducialTransformArray: FiducialTransformArray):
        #async update on receiving new transform information
        self.transformList = fiducialTransformArray.transforms

    
    
                
    def stateUpdater(self, data):
        #async update on receiving new state information
        self.state = data.data

    


    def __init__(self):


        self.pub =rospy.Publisher(
        #publishes the next block to pick up
        'priority_block', # Topic name
        Block, # Message type
        )

        self.publish_block = rospy.Publisher(
            #debug publisher to publish amount of blocks detected
        'block_detect',
        String
        )

        self.publish_raw = rospy.Publisher(
            #debug publisher to fiducials transforms of blocks detected
        'theta_raw',
        String
        )  

        self.publish_message = rospy.Publisher(
            #debug publisher for message outputs of the detect_block node
        'metr4202_message',
        String
        )  


        self.publish_state = rospy.Publisher(
            #state machine publisher
        'metr4202_state',
        Int16
        )  

        self.sub_state = rospy.Subscriber(
            #state machine subscriber
        "metr4202_state",
        Int16,
        self.stateUpdater
        )


        self.sub_fiducials_transform = rospy.Subscriber(
            #subscribe to aruco tag transforms from aruco_detect
        "fiducial_transforms",
        FiducialTransformArray,
        self.detection_callback       
        )


        self.calibrated = False

        #set calibration aruco code Transformation, T origin to T calibration code location
        self.Tox = np.array([
            [1,0,0,0],
            [0,-1,0,-0.215],
            [0,0,-1,0.117],
            [0,0,0,1]
        ])

        #backup calibration Transformation for flat arm pose calibration
        self.ToxFlat = np.array([
            [1,0,0,0],
            [0,-1,0,-0.190],
            [0,0,-1,0.015],
            [0,0,0,1]
        ])


        #init object variables to avoid NoneType error
        self.Toc = []
        self.state = 0
        self.transformList = []
        self.blockList = []
        self.oldBlockList = []
        self.rotating = False



    def find_transM(self, Transf:Transform):
        #converts fiducial transform to 4x4 transformation matrix
        rotQ = Transf.rotation
        transQ = Transf.translation
        rotM = R.from_quat([rotQ.x,rotQ.y,rotQ.z,rotQ.w] )
        rotM = R.as_matrix(rotM)
        transM = np.array([transQ.x, transQ.y, transQ.z])
        return mr.RpToTrans(rotM, transM)



    def calibrate(self, Transf: Transform):
        #performs final calibration based on aruco transform and publishes next state command
        Tcx = self.find_transM(Transf)
        self.Toc = np.dot(self.Tox, mr.TransInv(Tcx))
        self.calibrated = True
        self.publish_state.publish(1)
        
        

        
    def initialCalibration(self):
        #attempts calibration of system  
        listID = []
        if self.transformList == None:
            return "No aruco cubes detected"
        for fiducial in self.transformList:
            if(fiducial.fiducial_id == calibration_ID):
                    self.calibrate(fiducial.transform)
                    self.state = 1
                    self.publish_state.publish(self.state)
                    return str(self.Toc)
            listID.append(fiducial.fiducial_id)
        return "calibration id not found" 




    def track_fiducial(self, id):
        #trac single fiducial based on ID and outputing 2D coordinate information
        for fiducial in self.transformList:
            if(fiducial.fiducial_id == id):
                block = DetectedBlock(id, fiducial.transform, self.Toc)

                detectBlock.publish_block.publish(str(block.coordinate))    
                detectBlock.publish_raw.publish(str(block.absTheta))    
            



    def findPriorityBlock(self):
        #require system to be calibrated
        #finds the next block to be picked up


        newBlocks = []
        for fiducial in self.transformList:
            newBlock = DetectedBlock(fiducial.fiducial_id, fiducial.transform, self.Toc)
            newBlocks.append(newBlock)
        #add blocks

        self.rotating = self.rotationDetect(newBlocks, self.blockList, self.oldBlockList)
        #check for rotation with theta
        self.oldBlockList = self.blockList
        self.blockList = newBlocks
        #updating blocklist


        pubEmpty = False
        wait = False

        
        numBlocks = len(self.blockList)
        if numBlocks ==0:
            pubEmpty = True
            #self.publish_message.publish("no blocks")
            
        elif numBlocks == 1 and self.blockList[0].id == calibration_ID:
            pubEmpty = True
            #self.publish_message.publish("only calibration cube found")
            
        elif self.rotating:
            
            pubEmpty = True
            if self.state == 22:
                pubEmpty = False
            #self.publish_message.publish("rotating")

        if pubEmpty:
            #publish wait command to block
            emptyBlock = Block()
            emptyBlock.wait = True
            self.pub.publish(emptyBlock)
            return
            #publish empty wait if no tags detected or only calibration is detected

        for i in self.blockList:
            sumDis = 0
            for j in self.blockList:

                #find distance to other blocks
                deltaX = i.coordinate[0] - j.coordinate[0]
                deltaY = i.coordinate[1] - j.coordinate[1]
                dist = np.sqrt( np.square(deltaX) + np.square(deltaY))
                if(dist > 70):
                    dist = 0
                if(dist < 65):
                    dist = 100
                #calculating weight of distances on the priority calculation
                #Logic: avoid block clusters when picking up 
                
                sumDis += dist
            distWeight = sumDis/numBlocks

            L = np.sqrt( np.square(i.coordinate[0]) + np.square(i.coordinate[1]))
            #finding weight of distance from robot on the priority calculation
            #Logic: pick up further one away to avoid knocking down adjacent blocks
            #Logic: pick up closer block when block is hard to reach

            yWeight = np.abs(L/ 2)
            if L > 230 :
                yWeight = -yWeight


            priority = i.theta*2 + distWeight - yWeight
            #finding weight of combined function including theta
            #Logic: pick up block aligning with robot arm first
            i.setPriority(priority)

            

        currentBlock = self.blockList[0]
        if currentBlock.id == calibration_ID and len(self.blockList)>1:
            currentBlock = self.blockList[1]
            
        for block in self.blockList:
            if not block.id == calibration_ID:
                if block < currentBlock:
                    currentBlock = block
        #find highest priority (lower better)
            
        blockMsg = currentBlock.toMsg()
        blockMsg.wait = wait
        self.pub.publish(blockMsg)
        #publish block to ROS topic
        
        
        



            
                
       

    def rotationDetect(self, newBlocks, blockList, oldBlockList):
        #detects rotation to pause robot when rotating
        if oldBlockList == []:
            return True
        
        rot = 0
        for i in newBlocks:
            for j in blockList:
                if i.id == j.id:
                    rot += np.abs(i.absTheta - j.absTheta)
                    detectBlock.publish_message.publish(str(rot))
        
        for j in blockList:
            for k in oldBlockList:
                if j.id == k.id:
                    rot += np.abs(j.absTheta - k.absTheta)
                    detectBlock.publish_message.publish(str(rot))
        #find rotation in the last two frames. If both are below threshold the conveyor belt is stopped

                    
        
        if rot > rotation_theta_threshold:
            detectBlock.publish_message.publish("rotating")
            return True

        return False
                
                        


            



if __name__ == '__main__':

    rospy.init_node('detect_block')

    #set frequency to increase performance
    rate = rospy.Rate(ros_rate)
    detectBlock = DetectBlock()
    #instantiate object
    
    
        
    
    while not rospy.is_shutdown():
        

        if not detectBlock.calibrated:
            #calibrate when not calibrated
            if detectBlock.state == 10:
                detectBlock.initialCalibration()
                
            
        else:
            #publish detected priority block
            if detectBlock.state == 1 or detectBlock.state == 22:
                detectBlock.findPriorityBlock()
                #detectBlock.publish_message.publish("finding priority")
            

        rate.sleep()



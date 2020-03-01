#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is TofuBot node.
2020.3.2 DO NOT USE THIS NODE NOW.
by Fujiwara-Tofu.
'''

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import PoseWithCovarianceStamped


import cv2
import numpy as np
import random
import sendIdToJudge
import math


import tf



class State():
    '''
    now:current state
    before:last state
    done:a flag which changes to True when the sequence of current state done.
         False True
    target_x,target_y:target position
    '''
    STATES = (
        NONE,         #0
        TRACE_WALL,   #1
        ROTATE,       #2
        AIM_MARKER,   #3
        STOP,         #4
     ) = range(0, 5)


    def __init__(self):
        self.current=State.NONE
        self.last=State.NONE
        self.done= False
    def Change(self,newstate):
        self.last=self.current
        self.current=newstate
        self.done=False



class TofuBot():
    def __init__(self, bot_name="NoName"):
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        # lidar scan subscriber
        self.scan = LaserScan()
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)
        self.laser = self.scan.ranges

        # camera subscribver
        # for convert image topic to opencv obj
        self.img = None
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)

        # imu subscriber
        self.imu_sub = rospy.Subscriber('imu', Imu, self.imuCallback)

        # odom subscriber
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)

        # joint_states subscriber
        self.odom_sub = rospy.Subscriber('joint_states', JointState, self.jointstateCallback)

        # initialize state
        self.state=State()

        self.wheel_rot_r = 0
        self.wheel_rot_l = 0
        self.pose_x = 0
        self.pose_y = 0

        self.k = 0.5

        # speed [m/s]
        self.speed = 0.07

        # subscriber
        self.pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.poseCallback)

        self.twist = Twist()
        self.twist.linear.x = self.speed; self.twist.linear.y = 0.; self.twist.linear.z = 0.
        self.twist.angular.x = 0.; self.twist.angular.y = 0.; self.twist.angular.z = 0.
 
    def poseCallback(self, data):
        '''
        pose topic from amcl localizer
        update robot twist
        '''
        pose_x = data.pose.pose.position.x
        pose_y = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        rpy = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        th = rpy[2]

        th_xy = self.calcTargetTheta(pose_x,pose_y)
        
        th_diff = th_xy - th
        while not PI >= th_diff >= -PI:
            if th_diff > 0:
                th_diff -= 2*PI
            elif th_diff < 0:
                th_diff += 2*PI
        new_twist_ang_z = th_diff * self.k
        
        self.twist.angular.z = new_twist_ang_z
        print(pose_x, pose_y, th, th_xy, new_twist_ang_z)
    def calcTargetTheta(self, pose_x, pose_y):
        th =math.atan2(pose_y,pose_x) #-3.14159/2
        return th





    def calcTwist(self,x,th):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = th
        return twist

    def extentFlag(self, x, lower_x, upper_x):
        if lower_x < x and x < upper_x:
            return True
        return False

    def GetArea(self):
        hsv = cv2.cvtColor(self.img,cv2.COLOR_BGR2HSV)

        hsvLower = np.array([0, 128, 0])
        hsvUpper = np.array([30, 255, 255])
        mask1 = cv2.inRange(hsv, hsvLower, hsvUpper)

        hsvLower = np.array([150, 128, 0])
        hsvUpper = np.array([179, 255, 255])
        mask2 = cv2.inRange(hsv, hsvLower, hsvUpper)
	
        mask = mask1 + mask2

        masked_hsv = cv2.bitwise_and(self.img, self.img, mask=mask)
        gray = cv2.cvtColor(masked_hsv,cv2.COLOR_BGR2GRAY)

        ret,thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY)
        image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        area=0
        if len(contours)>=1:
            area=cv2.contourArea(contours[0])
        return area

    def StopBot(self):
        twist = self.calcTwist(0,0)
        self.vel_pub.publish(twist)

    def TraceWall(self): 
        
        rdif =  self.scan.ranges[360-55] - self.scan.ranges[360-124]
        rave = (self.scan.ranges[360-55] + self.scan.ranges[360-124])/2

        th=0.2-rave
        if th>0.2 :
            th=0.2
        if th<-0.2 :
            th=-0.2

        twist = self.calcTwist(0.05,th)
        self.vel_pub.publish(twist)

    def strategy(self):

        r = rospy.Rate(1) # change speed 1fps
        r.sleep()

        while not rospy.is_shutdown():
            '''
            x = self.pose_x; y = self.pose_y
            area = self.GetArea()
            
            # when find enemy
            if area > 2000:
			    self.state.Change(State.STOP)
            else:
                self.state.Change(self.state.last) # back to last state
            
            s= self.state.current
            
            #switch state
            if s == State.TRACE_WALL:
                # ToDo
                # if get near by the aiming position
                # self.state.Change(State.AIM_MARKER)
                
                # ToDo          
                # if get near by the wall
                # self.state.Change(State.ROTATE)
                s=State.TRACE_WALL #dummy

            elif s == State.AIM_MARKER or s == State.ROTATE:
                if self.state.done:
                    self.state.Change(State.TRACE_WALL)
                        

            #switch task and call once

            if s == State.TRACE_WALL:
                self.TraceWall()
            elif s == State.ROTATE:
                self.OrgSet()
            elif s == State.AIM_MARKER:
                self.AimMarker()
            elif s == State.STOP:
                self.StopBot()
            else:
                r.sleep()
            '''
            self.vel_pub.publish(self.twist)

        #end while

    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data
        self.laser = self.scan.ranges
        is_near_wall = self.isNearWall(data.ranges)
        if is_near_wall:
            self.twist.linear.x = -self.speed
        else:
            self.twist.linear.x = self.speed

    def isNearWall(self, scan):
        if not len(scan) == 360:
            return False
        forword_scan = scan[:10] + scan[-10:]
        # drop too small value ex) 0.0
        forword_scan = [x for x in forword_scan if x > 0.1]
        if min(forword_scan) < 0.2:
            return True
        return False


    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        cv2.imshow("Image window", self.img)
        cv2.waitKey(1)

    # imu call back sample
    # update imu state
    def imuCallback(self, data):
        self.imu = data
        rospy.loginfo(self.imu)

    # odom call back sample
    # update odometry state
    def odomCallback(self, data):
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y


    # jointstate call back sample
    # update joint state
    def jointstateCallback(self, data):
        self.wheel_rot_r = data.position[0]
        self.wheel_rot_l = data.position[1]




        

if __name__ == '__main__':
    rospy.init_node('tofubot')
    bot = TofuBot('Tofu')
    bot.strategy()


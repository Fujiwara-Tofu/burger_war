#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is TofuBot node.

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
import cv2
import numpy as np
import random
import sendIdToJudge
import tf
#from tf.transformations import euler_from_quaternion

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

#
PI = 3.14152

#Set Marker definition 
CENTER_BLOCK = 0
CORNER_BLOCK_1 = 1
CORNER_BLOCK_2 = 2
CORNER_BLOCK_3 = 3
CORNER_BLOCK_4 = 4

CENTER_BLOCK_WIDTH = 0.350
CORNER_BLOCK_WIDTH_X = 0.200
CORNER_BLOCK_WIDTH_Y = 0.150

CENTER_BLOCK_POS_X = 0
CENTER_BLOCK_POS_Y = 0

CORNER_BLOCK_POS = 0.530
CORNER_BLOCK_1_POS_X =  CORNER_BLOCK_POS
CORNER_BLOCK_1_POS_Y =  CORNER_BLOCK_POS
CORNER_BLOCK_2_POS_X = -CORNER_BLOCK_POS
CORNER_BLOCK_2_POS_Y =  CORNER_BLOCK_POS
CORNER_BLOCK_3_POS_X = -CORNER_BLOCK_POS
CORNER_BLOCK_3_POS_Y = -CORNER_BLOCK_POS
CORNER_BLOCK_4_POS_X =  CORNER_BLOCK_POS
CORNER_BLOCK_4_POS_Y = -CORNER_BLOCK_POS

MARKER_UP    = 1
MARKER_DOWN  = 2
MARKER_RIGHT = 3
MARKER_LEFT  = 4
#/Set Marker definition 

SIMPLE_GOAL_STATE_PENDING = 0
SIMPLE_GOAL_STATE_ACTIVE = 1
SIMPLE_GOAL_STATE_DONE = 2


class TofuBot():
    def __init__(self, bot_name="NoName",use_lidar=False, use_camera=False, use_imu=False,
                 use_odom=False, use_joint_states=False):
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        # lidar scan subscriber
        if use_lidar:
            self.scan = LaserScan()
            self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)
            self.laser = self.scan.ranges

        # camera subscribver
        # please uncoment out if you use camera
        if use_camera:
            # for convert image topic to opencv obj
            self.img = None
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)

        # imu subscriber
        if use_imu:
            self.imu_sub = rospy.Subscriber('imu', Imu, self.imuCallback)

        # odom subscriber
        if use_odom:
            self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)

        # joint_states subscriber
        if use_joint_states:
            self.odom_sub = rospy.Subscriber('joint_states', JointState, self.jointstateCallback)
     
    #initialize parameter
        self.odom_pose_x = 0
        self.odom_pose_y = 0
        self.odom_orientation_yaw = PI/2
        self.imu_orientation_yaw = PI/2
        self.S = 0
    #/initialize parameter

    # lidar scan topic call back sample
    # update lidar scan state    
    def lidarCallback(self, data):
        self.scan = data
        #rospy.loginfo(self.scan)

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        self.S = self.area(self.img) #
        cv2.imshow("Image window", self.img)
        cv2.waitKey(1)


    # imu call back sample
    # update imu state
    def imuCallback(self, data):
        self.imu = data
        #rospy.loginfo(data)
        self.imu_q = (data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w)
    
        imu_euler = tf.transformations.euler_from_quaternion(self.imu_q)
        self.imu_orientation_yaw = imu_euler[2]

        '''
        if imu_euler[2] > PI:
            self.orientation_yaw -= 2 * PI
        elif imu_euler[2] < -PI:
            self.orientation_yaw += 2 * PI
        '''
        #rospy.loginfo("imu orientation_yaw: {}".format(self.imu_orientation_yaw))

    #odomCallback
    def odomCallback(self, data):
        self.odom_pose_x = data.pose.pose.position.x
        self.odom_pose_y = data.pose.pose.position.y

        odom_q = data.pose.pose.orientation
        odom_e = tf.transformations.euler_from_quaternion((odom_q.x,odom_q.y,odom_q.z,odom_q.w)) 
        self.odom_orientation_yaw = odom_e[2]
        
        #rospy.loginfo("odom pose_x: {}".format(self.odom_pose_x))
        #rospy.loginfo("odom pose_y: {}".format(self.odom_pose_y))
        #rospy.loginfo("odom orientation_yaw: {}".format(self.odom_orientation_yaw))
        
    #/odomCallback

    # jointstate call back sample
    # update joint state
    def jointstateCallback(self, data):
        self.wheel_rot_r = data.position[0]
        self.wheel_rot_l = data.position[1]
        #rospy.loginfo("joint_state R: {}".format(self.wheel_rot_r))
        #rospy.loginfo("joint_state L: {}".format(self.wheel_rot_l))


    def area(self,img):
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

        hsvLower = np.array([0, 128, 0])
        hsvUpper = np.array([30, 255, 255])
        mask1 = cv2.inRange(hsv, hsvLower, hsvUpper)

        hsvLower = np.array([150, 128, 0])
        hsvUpper = np.array([179, 255, 255])
        mask2 = cv2.inRange(hsv, hsvLower, hsvUpper)
    
        mask = mask1 + mask2

        masked_hsv = cv2.bitwise_and(img, img, mask=mask)
        gray = cv2.cvtColor(masked_hsv,cv2.COLOR_BGR2GRAY)

        ret,thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY)
        image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        count_area = contours[0]
        return cv2.contourArea(count_area)
 
    def stop(self):
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        self.vel_pub.publish(twist)


    def setGoal(self,x,y,yaw):
        if self.goal_set_flag == 0:
            self.client.wait_for_server()
    
            #Coordinate transformation (Odom -> Move Base)
            temp_x = x
            x = y
            y = -temp_x
            yaw -= PI/2
            #/Coordinate transformation (Odom -> Move Base)

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "/map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y

            # Euler to Quartanion
            q=tf.transformations.quaternion_from_euler(0,0,yaw)        
            goal.target_pose.pose.orientation.x = q[0]
            goal.target_pose.pose.orientation.y = q[1]
            goal.target_pose.pose.orientation.z = q[2]
            goal.target_pose.pose.orientation.w = q[3]

            self.client.send_goal(goal)
            self.goal_set_flag = 1

        if not self.client.simple_state == SIMPLE_GOAL_STATE_DONE:
            if not self.client.gh:
                rospy.logerr("Called wait_for_goal_to_finish when no goal exists")
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
                return -1
            else:
                return 1
        else:        
            self.client.get_result()
            return 0
      

    #Move to field field marker
    def MoveToFieldMarker(self,block_name, marker_name, shooting_distance = 0.300):
        if self.goal_set_flag == 0:
            if block_name == CENTER_BLOCK:
                if marker_name == MARKER_UP:
                    self.goal_pos_x = CENTER_BLOCK_POS_X
                    self.goal_pos_y = CENTER_BLOCK_POS_Y + (CENTER_BLOCK_WIDTH/2 + shooting_distance)
                    self.goal_angle = -PI/2        
                elif marker_name == MARKER_DOWN:
                    self.goal_pos_x = CENTER_BLOCK_POS_X
                    self.goal_pos_y = CENTER_BLOCK_POS_Y - (CENTER_BLOCK_WIDTH/2 + shooting_distance)
                    self.goal_angle = PI/2    
                elif marker_name == MARKER_RIGHT:
                    self.goal_pos_x = CENTER_BLOCK_POS_X + (CENTER_BLOCK_WIDTH/2 + shooting_distance)
                    self.goal_pos_y = CENTER_BLOCK_POS_Y
                    self.goal_angle = PI
                elif marker_name == MARKER_LEFT:
                    self.goal_pos_x = CENTER_BLOCK_POS_X - (CENTER_BLOCK_WIDTH/2 + shooting_distance)
                    self.goal_pos_y = CENTER_BLOCK_POS_Y    
                    self.goal_angle = 0
                else:
                    rospy.loginfo("Invalid_MARKER_NAME")
                    return -1
    
            elif block_name == CORNER_BLOCK_1:
                if marker_name == MARKER_UP:
                    self.goal_pos_x = CORNER_BLOCK_1_POS_X
                    self.goal_pos_y = CORNER_BLOCK_1_POS_Y + (CORNER_BLOCK_WIDTH_Y/2 + shooting_distance)
                    self.goal_angle = -PI/2            
                elif marker_name == MARKER_DOWN:
                    self.goal_pos_x = CORNER_BLOCK_1_POS_X    
                    self.goal_pos_y = CORNER_BLOCK_1_POS_Y - (CORNER_BLOCK_WIDTH_Y/2 + shooting_distance)    
                    self.goal_angle = PI/2        
                else:
                    rospy.loginfo("Invalid_MARKER_NAME")    
                    return -1

            elif block_name == CORNER_BLOCK_2:
                if marker_name == MARKER_UP:    
                    self.goal_pos_x = CORNER_BLOCK_2_POS_X    
                    self.goal_pos_y = CORNER_BLOCK_2_POS_Y + (CORNER_BLOCK_WIDTH_Y/2 + shooting_distance)
                    self.goal_angle = -PI/2        
                elif marker_name == MARKER_DOWN:
                    self.goal_pos_x = CORNER_BLOCK_2_POS_X
                    self.goal_pos_y = CORNER_BLOCK_2_POS_Y - (CORNER_BLOCK_WIDTH_Y/2 + shooting_distance)
                    self.goal_angle = PI/2    
                else:
                    rospy.loginfo("Invalid_MARKER_NAME")
                    return -1

            elif block_name == CORNER_BLOCK_3:
                if marker_name == MARKER_UP:
                    self.goal_pos_x = CORNER_BLOCK_3_POS_X
                    self.goal_pos_y = CORNER_BLOCK_3_POS_Y + (CORNER_BLOCK_WIDTH_Y/2 + shooting_distance)
                    self.goal_angle = -PI/2            
                elif marker_name == MARKER_DOWN:
                    self.goal_pos_x = CORNER_BLOCK_3_POS_X    
                    self.goal_pos_y = CORNER_BLOCK_3_POS_Y - (CORNER_BLOCK_WIDTH_Y/2 + shooting_distance)        
                    self.goal_angle = PI/2    
                else:
                    rospy.loginfo("Invalid_MARKER_NAME")    
                    return -1

            elif block_name == CORNER_BLOCK_4:
                if marker_name == MARKER_UP:    
                    self.goal_pos_x = CORNER_BLOCK_4_POS_X
                    self.goal_pos_y = CORNER_BLOCK_4_POS_Y + (CORNER_BLOCK_WIDTH_Y/2 + shooting_distance)
                    self.goal_angle = -PI/2        
                elif marker_name == MARKER_DOWN:
                    self.goal_pos_x = CORNER_BLOCK_4_POS_X
                    self.goal_pos_y = CORNER_BLOCK_4_POS_Y - (CORNER_BLOCK_WIDTH_Y/2 + shooting_distance)
                    self.goal_angle = PI/2    
                else:
                    rospy.loginfo("Invalid_MARKER_NAME")
                    return -1
        
            else:
                rospy.loginfo("Invalid_BLOCK_NAME")
                return -1
        
        return self.setGoal(self.goal_pos_x,self.goal_pos_y,self.goal_angle)
    
    #/Move to field field marker

    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps
        r.sleep()
        self.init_x = self.odom_pose_x
        self.init_y = self.odom_pose_y
        self.init_odom_yaw = self.odom_orientation_yaw
        self.init_imu_yaw = self.imu_orientation_yaw
        rospy.loginfo("init_x: {}[m]".format(self.init_x))
        rospy.loginfo("init_y: {}[m]".format(self.init_y))
        rospy.loginfo("init_yaw(odom): {}[rad]".format(self.init_odom_yaw))
        rospy.loginfo("init_yaw(imu): {}[rad]".format(self.init_imu_yaw))
       
        route_state = 0
        self.goal_set_flag = 0
        active_flag = 0
        while not rospy.is_shutdown():
            #割込みでカメラ情報を読んで止まる
            if self.S > 2000:
                twist = self.stop()
                rospy.loginfo("--STOP Moving! Enemy Ahead!--")
                if active_flag == 1:
                    self.client.cancel_all_goals()
                    self.goal_set_flag = 0

            else:
                if route_state == 0:
                    active_flag = self.setGoal(0.2,-0.9,PI/4)
                elif route_state == 1:
                    active_flag = self.MoveToFieldMarker(CORNER_BLOCK_4, MARKER_DOWN)
                elif route_state == 2:
                    active_flag = self.MoveToFieldMarker(CORNER_BLOCK_4, MARKER_DOWN)
                elif route_state == 3:
                    active_flag = self.setGoal(self.odom_pose_x, self.odom_pose_y,PI)
                elif route_state == 4:
                    active_flag = self.setGoal(0,-0.53,PI/2)
                elif route_state == 5:
                    active_flag = self.MoveToFieldMarker(CENTER_BLOCK, MARKER_DOWN)
                elif route_state == 6:
                    active_flag = self.setGoal(0.53,0,PI/2)
                elif route_state == 7:        
                    active_flag = self.MoveToFieldMarker(CORNER_BLOCK_1, MARKER_DOWN)
                elif route_state == 8:
                    active_flag = self.MoveToFieldMarker(CENTER_BLOCK, MARKER_RIGHT)
                elif route_state == 9:
                    active_flag = self.MoveToFieldMarker(CORNER_BLOCK_4, MARKER_UP)
                elif route_state == 10:
                    active_flag = self.MoveToFieldMarker(CORNER_BLOCK_3, MARKER_UP)
                elif route_state == 11:
                    active_flag = self.MoveToFieldMarker(CENTER_BLOCK, MARKER_LEFT)
                elif route_state == 12:
                    active_flag = self.MoveToFieldMarker(CORNER_BLOCK_2, MARKER_DOWN)
                elif route_state == 13:
                    active_flag = self.setGoal(-0.53,0,-PI/2)
                elif route_state == 14:    
                    active_flag = self.setGoal(0,-0.53,-PI/2)
                elif route_state == 15:
                    active_flag = self.setGoal(0,-0.90,-PI/2)
                elif route_state == 16:
                    active_flag = self.MoveToFieldMarker(CORNER_BLOCK_3, MARKER_DOWN)
                elif route_state == 17:
                    active_flag = self.setGoal(self.odom_pose_x, self.odom_pose_y,0)
                
                if active_flag == 0:
                    self.goal_set_flag = 0
                    route_state += 1
                    if route_state > 17:
                        route_state = 0

            rospy.loginfo("odom pose_x: {}[m]".format(self.odom_pose_x))
            rospy.loginfo("odom pose_y: {}[m]".format(self.odom_pose_y))
            rospy.loginfo("odom orientation_yaw: {}[rad]".format(self.odom_orientation_yaw))
            rospy.loginfo("imu orientation_yaw: {}[rad]".format(self.imu_orientation_yaw))
            rospy.loginfo("route_state: {}".format(route_state))
            rospy.loginfo("goal_set_flag: {}".format(self.goal_set_flag))
            rospy.loginfo("active_flag: {}".format(active_flag))

        #self.setGoal(self.init_x,self.init_y,self.init_odom_yaw)    #


if __name__ == '__main__':
    rospy.init_node('tofubot')
    bot = TofuBot('Tofu',True,True,True,True,True)
    bot.strategy()

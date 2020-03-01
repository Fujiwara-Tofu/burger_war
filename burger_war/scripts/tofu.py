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
import random
import sendIdToJudge

class state():
	def __init__(self, now=2, before=3):
		self.now = now; self.before = before
	def both_sub(self, x, y):#代入
		self.now = x; self.before = y
	def both_return(self):
		return self.now, self.before


class TofuBot():
    def __init__(self, bot_name="NoName",use_lidar=False, use_camera=False, use_imu=False,
                 use_odom=False, use_joint_states=False):
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
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

    def stop(self):
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0


    def strategy(self):
        r = rospy.Rate(1) # change speed 1fps

        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0
        r.sleep()

        while not rospy.is_shutdown():
            twist = self.walltrace(1)
            #twist = self.calcTwist()
            #print(twist)
            #print(self.laser)

            self.vel_pub.publish(twist)
            
            r.sleep()

    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data
        #rospy.loginfo(self.scan.ranges)
	    #print(self.scan.ranges[10])
        self.laser = self.scan.ranges

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        #cv2.imshow("Image window", self.img)
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
        rospy.loginfo("odom pose_x: {}".format(self.pose_x))
        rospy.loginfo("odom pose_y: {}".format(self.pose_y))

    # jointstate call back sample
    # update joint state
    def jointstateCallback(self, data):
        self.wheel_rot_r = data.position[0]
        self.wheel_rot_l = data.position[1]
        rospy.loginfo("joint_state R: {}".format(self.wheel_rot_r))
        rospy.loginfo("joint_state L: {}".format(self.wheel_rot_l))

    def walltrace(self, data): 
        
        r = self.scan.ranges[55] - self.scan.ranges[124]
        l = self.scan.ranges[-124] - self.scan.ranges[-55]
        x = 0.0
        th = 0.0
        kp = 0.1
        twist = Twist()
        if data == 0:
		    if self.scan.ranges[55] > 0.2:
			    wallrad = 0.2
		    else:
			    wallrad = 0

		    if 0.1 > r + wallrad and -0.1 < r + wallrad:
			    x = 0.1
			    th = 0
			    print("a")
		    elif 0.1 <= r + wallrad:
			    x = 0.05
			    th = kp*r
			    print("b")
		    else: 
			    x = 0.05
			    th = kp*r
			    print("c")
        else:
            if self.scan.ranges[-55] > 0.2:
                wallrad = 0.2
            else:
                wallrad = 0

            if 0.2 > l and -0.2 < l:
                x = 0.1
                th = 0
                print("d")
            elif 0.2 <= l:
                x = 0.05
                th = kp*l
                print("e")
            else: 
                x = 0.05
                th = kp*l
                print("f")
        
        twist.linear.x = x
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = th
        return twist
        

    def odmreset():
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y

        del_range = 0
        distance = 0.3
        turn_angle = 90
        end = 0
        self.turn_count = 0
        self.coner_posi = self.turn_count % 4
        #self.posi1 = [,]
        #self.posi2 = [,]
        #self.posi3 = [,]
        #self.posi4 = [,]

        while end > 0:
            old_lider = self.scan.ranges[0]
            if old_lider - self.scan.ranges[0] > del_range:
                x = 1
            else:
                #offset_x = xxx - self.pose_x
                #offset_y = yyy - self.pose_y
                #self.offset_posi = self.posi"self.coner_posi" - [self.pose_x,self.pose_y]


                while self.scan.ranges[0]-old_lider < distance:
                    x = -1
                    twist = Twist()
                    twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
                    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
	            self.vel_pub.publish(twist)

                while self.scan.ranges[55] - self.scan.ranges[124] > -0.1 or self.scan.ranges[55] - self.scan.ranges[124] > 0.1:
                    th = 1
                    twist = Twist()
                    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
                    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
	            self.vel_pub.publish(twist)
                end = 1
                self.turn_count += 1


        return (offset_x, offset_y)

if __name__ == '__main__':
    rospy.init_node('tofubot')
    bot = TofuBot('Tofu')
    bot.strategy()


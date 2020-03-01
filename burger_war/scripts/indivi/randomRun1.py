#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is rumdom run node.
subscribe No topcs.
Publish 'cmd_vel' topic. 
mainly use for simple sample program

by Takuya Yamaguhi.
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

class RandomBot():
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


    def calcTwist(self):
        value = random.randint(1,1000)
        if value < 250:
            x = 0.2
            th = 0
        elif value < 500:
            x = -0.2
            th = 0
        elif value < 750:
            x = 0
            th = 1
        elif value < 1000:
            x = 0
            th = -1
        else:
            x = 0
            th = 0
	x = 0
	th = 0
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

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
            '''
            if self.laser.sizes() != 0:
	            print(self.laser[10])
            '''
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
	r = self.scan.ranges[49] - self.scan.ranges[130]
	l = self.scan.ranges[-130] - self.scan.ranges[-49]
	kp1 = 0.3
	kp2 = 0.1
	twist = Twist()
	if data == 0:
		if self.scan.ranges[49] < 0.1:
			if 0.2 > r and -0.2 < r:
				x = 0.1
				th = 0
				print("a")
			elif 0.2 <= r:
				x = 0.05
				th = kp1*r
				print("b")
			else: 
				x = 0.05
				th = kp1*r
				print("c")
		else:
			if 0.2 <= r:
				x = 0.05
				th = kp1*r
				print("b")
			elif -0.2 < : 
				x = 0.05
				th = kp1*r
				print("c")
	else:
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

        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
	return twist


if __name__ == '__main__':
    rospy.init_node('random_run')
    bot = RandomBot('Random',use_lidar=True, use_camera=True, use_imu=False,
                       use_odom=False, use_joint_states=False)
    bot.strategy()


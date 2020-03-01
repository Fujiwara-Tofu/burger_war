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
import numpy as np

class state():
	'''
	now:現在の状態
	before:過去の状態
	done:現在の状態が完了の有無のフラグ
	     False True
	target_x,target_y:ターゲット座標
	'''
	def __init__(self, now=2, before=3, x, y):
		self.now = now; self.before = before; self.done= False;
		self.target_x = x; self.target_y = y
	def both_sub(self, x, y, done):#代入
		self.now = x; self.before = y; state.done = False
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
	self.vel_pub.publish(twist)


    def calcTwist(self):
	x = 1
	th = 0
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def extentFlag(self, x, lower_x, upper_x):
	if lower_x < x and x < upper_x
		return True
	return False

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
	gray = cv2.cvtColor(masked_img,cv2.COLOR_BGR2GRAY)

	ret,thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY)
	image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	return cv2.contourArea(contours[0])


    def strategy(self):
	state = state()
	position = data.pose.pose.position
	errbar = 0.2	#[m]
	ex_state = [1,2,3,4]	#existing state
	
	'''
	state = 1 壁沿い
	state = 2 原点復帰
	state = 3 マーカに向かう+戻る
	state = 4 止まる（enemyがcameraで見えている）
	'''


        r = rospy.Rate(1) # change speed 1fps

        while not rospy.is_shutdown():
		x = position.x; y = position.y
		area = area(self.img)
		if area > 2000:
			state.both_sub(ex_state[3],state.now,False)

		if state.now == ex_state[0]:
			tracewall()
			#的の近く
			if extentFlag(x, x-errbar,x+errbar) and extentFlag(y, y-errbar,y+errbar):
				aimmarker()
				#元の位置に戻る
			elif self.scan.ranges[0] < errbar:
				orgset()
#			else:
			if state.done == True:
				if state.before == ex_state[1]:
					state.both_sub(ex_state[2], ex_state[1], False)
				elif state.before == ex_state[2]:
					#前状態3->現在状態1
					state.both_sub(ex_state[1], ex_state[2], False)
#				else:

		elif state.now == ex_state[1]:
			orgset()
			if state.done == True:
				state.both_sub(ex_state[2], ex_state[1], False)
		elif state.now == ex_state[2]:
			aimmarker()
			if state.done ==True:
				state.both_sub(ex_state[0], ex_state[2], False)

		#割込みでカメラ情報を読んで止まる
		elif state.now == ex_state[3]:
			stop()
			if state.done == True:
				state.both_sub(state.now, ex_state[3], False)
#		else:
		r.sleep()
#end while

    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data
        rospy.loginfo(self.scan)
        #rospy.loginfo(self.scan.ranges)

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
        rospy.loginfo("odom pose_x: {}".format(self.pose_x))
        rospy.loginfo("odom pose_y: {}".format(self.pose_y))

    # jointstate call back sample
    # update joint state
    def jointstateCallback(self, data):
        self.wheel_rot_r = data.position[0]
        self.wheel_rot_l = data.position[1]
        rospy.loginfo("joint_state R: {}".format(self.wheel_rot_r))
        rospy.loginfo("joint_state L: {}".format(self.wheel_rot_l))


if __name__ == '__main__':
    rospy.init_node('random_run')
    bot = RandomBot('Random',use_lidar=True, use_camera=True, use_imu=False,
                       use_odom=True, use_joint_states=False)
    bot.strategy()

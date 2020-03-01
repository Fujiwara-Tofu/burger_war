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
from tf.transformations import euler_from_quaternion

class State():
    '''
    now:state of now
    before:state of before
    done:
         True:  if state is done
         False: if state is not done
    target_x,target_y:position of target
    '''
    def __init__(self, now=2, before=3, x=0, y=0):
        self.now = now; self.before = before; self.done= False;
        self.target_x = x; self.target_y = y
    def both_sub(self, x, y, done):#:substitution
        self.now = x; self.before = y; state.done = False
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
        self.vel_pub.publish(twist)


    def calcTwist(self):
        x = 1
        th = 0
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def extentFlag(self, x, lower_x, upper_x):
        if lower_x < x and x < upper_x:
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
        state = State()
        errbar = 0.2    #[m]
        ex_state = [1,2,3,4]    #existing state
        RIGHT = 0; LEFT = 1     # when state = 1 trace wall by left hand or right hand
        '''
        state = 1 壁沿い       
        state = 2 原点復帰
        state = 3 マーカに向かう+戻る
        state = 4 止まる（enemyがcameraで見えている）
        '''


        r = rospy.Rate(1) # change speed 1fps

        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0
        r.sleep()

        while not rospy.is_shutdown():
            if self.S > 2000:
                state.both_sub(ex_state[3],state.now,False)

            if state.now == ex_state[0]:
                #calc from target position (not yet) 
                if migi:
                    twist = walltrace(RIGHT)
                elif hidari:
                    twist = walltrace(LEFT)
                self.vel_pub.publish(twist)

                #的の近く
                if extentFlag(x, x-errbar,x+errbar) and extentFlag(y, y-errbar,y+errbar):
                    aimmarker()
                    #元の位置に戻る
                elif self.scan.ranges[0] < errbar:
                    orgset()

                if state.done == True:
                    if state.before == ex_state[1]:
                        state.both_sub(ex_state[2], ex_state[1], False)
                    elif state.before == ex_state[2]:
                        #前状態3->現在状態1
                        state.both_sub(ex_state[1], ex_state[2], False)

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
                twist = stop()
                if state.done == True:
                    state.both_sub(state.now, ex_state[3], False)
#        else:
            r.sleep()
#end while

    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data
        rospy.loginfo(self.scan.ranges)
        #print(self.scan.ranges[10])
        self.laser = self.scan.ranges
        self.old_lider = 

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        self.S = self.area(self.img)
        cv2.imshow("Image window", self.img)
        cv2.waitKey(1)

    # imu call back sample
    # update imu state
    def imuCallback(self, data):
        self.imu = data
        rospy.loginfo(self.imu)
        euler = euler_from_quaternion(self.test)
        #roll_rad = euler[0]
        #pitch_rad = euler[1]
        if euler[2] <0:
            self.yaw_rad = 6.2831853072 + euler[2] 
        else:
            self.yaw_rad = euler[2]
#        print(yaw_rad*180/3.14)
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
        
        r = self.scan.ranges[45] - self.scan.ranges[135]
        l = self.scan.ranges[225] - self.scan.ranges[315]
        x = 0.0
        th = 0.0
        kp = 0.1
        twist = Twist()
        if data == 0:
            if self.scan.ranges[45] > 0.2:
                wallrad = 0.2
            else:
                wallrad = 0

            if 0.1 > r + wallrad and -0.1 < r + wallrad:
                x = 0.1;th = 0
                #print("a")
            elif 0.1 <= r + wallrad:
                x = 0.05; th = kp*r
                #print("b")
            else: 
                x = 0.05; th = kp*r
                #print("c")
        else:
            if self.scan.ranges[315] > 0.2:
                wallrad = 0.2
            else:
                wallrad = 0

            if 0.2 > l and -0.2 < l:
                x = 0.1; th = 0
                #print("d")
            elif 0.2 <= l:
                x = 0.05; th = kp*l
                #print("e")
            else: 
                x = 0.05; th = kp*l
                #print("f")
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist
        

    def odmreset():

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

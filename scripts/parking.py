#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 wechange tech.
# Developer: FuZhi Liu 
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from nav_msgs.msg import Odometry
import tf
from enum import Enum
import math

class parking:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.twist = Twist()
        self.logmark = True
        self.counter = 1
        self.flag = False
        self.notsingcounter = 0
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.image_gray_pub = rospy.Publisher("gray_image_image", Image, queue_size=1)
        # publishes traffic sign image in raw type
        self.pub_image_traffic_sign = rospy.Publisher('/detect/image_output', Image, queue_size = 1)
        # self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.searchparklot, queue_size=1)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom, queue_size=1)

        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        self.img3 = cv2.imread('/home/nanorobot/catkin_ws/src/robot_vision/scripts/parking.jpg')      # trainImage3
        # Initiate SIFT detector
        self.sift = cv2.xfeatures2d.SIFT_create()
        self.kp3, self.des3 = self.sift.detectAndCompute(self.img3,None)
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

        self.StepOfParking = Enum('StepOfParking', 'idle parking_lot parking_lot_entry slot_entry turn_first turn_second turn_third  no_slot_r no_slot_l park parked')
        self.current_step_of_parking = self.StepOfParking.parking_lot.value
        self.theta = 0.0
        self.last_current_theta = 0.0
        
        self.is_step_start = False
        
        self.lastError = 0.0
        self.is_step_parking = False

        loop_rate = rospy.Rate(10) # 10hz
        
        while not rospy.is_shutdown():
            # print 'while 11'
            if self.is_step_parking == True:
                self.fnParking()
            
            loop_rate.sleep()

    def fnCalcMSE(self, arr1, arr2):
            squared_diff = (arr1 - arr2) ** 2
            sum = np.sum(squared_diff)
            num_all = arr1.shape[0] * arr1.shape[1] #cv_image_input and 2 should have same shape
            err = sum / num_all
            return err

    def searchparklot(self, image_msg):
        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1
        
        cv_image_input = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        MIN_MATCH_COUNT = 9
        MIN_MSE_DECISION = 50000
        # find the keypoints and descriptors with SIFT
        kp1, des1 = self.sift.detectAndCompute(cv_image_input,None)
        matches3 = self.flann.knnMatch(des1,self.des3,k=2)
        image_out_num = 1
        good3 = []
        for m,n in matches3:
            if m.distance < 0.7*n.distance:
                good3.append(m)
        if len(good3)>MIN_MATCH_COUNT:
            dst_pts = np.float32([ kp1[m.queryIdx].pt for m in good3 ]).reshape(-1,1,2)
            src_pts = np.float32([ self.kp3[m.trainIdx].pt for m in good3 ]).reshape(-1,1,2)
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask3 = mask.ravel().tolist()
            h,w,_ = self.img3.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,M)
            
            # mse = self.fnCalcMSE(src_pts, dst_pts)
            # if mse < MIN_MSE_DECISION:
            if True:
                msg_sign = UInt8()
                image_out_num = 3

        else:
            matchesMask3 = None

        if image_out_num == 1:
            if self.current_step_of_parking == self.StepOfParking.turn_first.value:
                self.notsingcounter += 1
                if self.notsingcounter > 5:
                    self.current_step_of_parking = self.StepOfParking.no_slot_r.value
            if self.current_step_of_parking == self.StepOfParking.turn_second.value:
                self.notsingcounter += 1
                if self.notsingcounter > 5:
                    self.current_step_of_parking = self.StepOfParking.no_slot_l.value
                
            print self.current_step_of_parking
            self.pub_image_traffic_sign.publish(self.bridge.cv2_to_imgmsg(cv_image_input, "bgr8"))
        elif image_out_num == 3:
            cv2.polylines(cv_image_input,[np.int32(dst)],True,(0,255,0),5,cv2.LINE_AA)
            width = np.int32(-dst[0][0][0]-dst[1][0][0]+dst[2][0][0]+dst[3][0][0])
            circle = sum(np.int32(dst)/4)
            cv2.circle(cv_image_input,(circle[0][0],circle[0][1]), 20, (0,0,255), -1)
            if width > 200 and width < 400:
                draw_params3 = dict(matchColor = (255,0,0), # draw matches in green color
                                singlePointColor = None,
                                matchesMask = matchesMask3, # draw only inliers
                                flags = 2)

                final3 = cv2.drawMatches(cv_image_input,kp1,self.img3,self.kp3,good3,None,**draw_params3)
                # publishes traffic sign image in raw type
                self.pub_image_traffic_sign.publish(self.bridge.cv2_to_imgmsg(final3, "bgr8")) 
                # print width,circle

                # print self.current_step_of_parking
                if self.current_step_of_parking == self.StepOfParking.parking_lot.value:
                    self.distance = (480 - circle[0][1]) * 0.09 + 18.0
                    print self.distance
                    self.current_step_of_parking = self.StepOfParking.parking_lot_entry.value
                    self.is_step_parking = True
                if self.current_step_of_parking == self.StepOfParking.turn_first.value or self.current_step_of_parking == self.StepOfParking.turn_second.value:
                    self.distance = (480 - circle[0][1]) * 0.09 + 18.0
                    print self.distance
                    self.current_step_of_parking = self.StepOfParking.park.value
                    self.is_step_parking = True
                    self.notsingcounter = 0

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()


    def euler_from_quaternion(self, quaternion):
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        return theta

    def cbOdom(self, odom_msg):
        quaternion = (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
        self.current_theta = self.euler_from_quaternion(quaternion)
        if (self.current_theta - self.last_current_theta) < -math.pi:
            self.current_theta = 2. * math.pi + self.current_theta
            self.last_current_theta = math.pi
        elif (self.current_theta - self.last_current_theta) > math.pi:
            self.current_theta = -2. * math.pi + self.current_theta
            self.last_current_theta = -math.pi
        else:
            self.last_current_theta = self.current_theta
        self.current_pos_x = odom_msg.pose.pose.position.x
        self.current_pos_y = odom_msg.pose.pose.position.y

    def fnTurn(self):
        err_theta = self.current_theta - self.desired_theta      
        # rospy.loginfo("Parking_Turn")
        # rospy.loginfo("err_theta  desired_theta  current_theta : %f  %f  %f", err_theta, self.desired_theta, self.current_theta)
        Kp = 0.6
        Kd = 0.03
        angular_z = Kp * err_theta + Kd * (err_theta - self.lastError)
        self.lastError = err_theta
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -angular_z
        self.pub_cmd.publish(twist)

        # rospy.loginfo("angular_z : %f", angular_z)

        return err_theta


    def fnStraight(self, desired_dist):
        err_pos = math.sqrt((self.current_pos_x - self.start_pos_x) ** 2 + (self.current_pos_y - self.start_pos_y) ** 2) - desired_dist
        # rospy.loginfo("Parking_Straight")
        Kp = 0.4
        Kd = 0.05
        angular_z = Kp * err_pos + Kd * (err_pos - self.lastError)
        self.lastError = err_pos

        twist = Twist()
        twist.linear.x = 0.07
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd.publish(twist)

        return err_pos
        
    def fnParking(self):
        if self.current_step_of_parking == self.StepOfParking.parking_lot_entry.value:
            # rospy.loginfo("parking_lot_entry")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.start_pos_x = self.current_pos_x
                self.start_pos_y = self.current_pos_y
                self.is_step_start = True
            error = self.fnStraight(self.distance/100)  #cm to m unit convert

            if math.fabs(error) < 0.005:
                rospy.loginfo("parking_lot_entry finished")
                twist = Twist()
                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0
                self.pub_cmd.publish(twist)
                rospy.sleep(1)
                self.current_step_of_parking = self.StepOfParking.slot_entry.value
                # self.current_step_of_parking = self.StepOfParking.idle.value
                self.is_step_start = False 
            # rospy.loginfo("outer_turn_first")
        if self.current_step_of_parking == self.StepOfParking.slot_entry.value:
            if self.is_step_start == False:
                self.lastError = 0.0
                self.desired_theta = self.current_theta - 1.57
                self.is_step_start = True
            error = self.fnTurn()
            if math.fabs(error) < 0.05:
                rospy.loginfo("outer_turn_first finished")
                self.current_step_of_parking = self.StepOfParking.turn_first.value
                self.is_step_start = False
        if self.current_step_of_parking == self.StepOfParking.no_slot_r.value:
            if self.is_step_start == False:
                self.lastError = 0.0
                self.desired_theta = self.current_theta - 3.14
                self.is_step_start = True
            error = self.fnTurn()
            if math.fabs(error) < 0.05:
                rospy.loginfo("outer_turn_first finished")
                self.current_step_of_parking = self.StepOfParking.turn_second.value
                self.is_step_start = False
        if self.current_step_of_parking == self.StepOfParking.no_slot_l.value:
            if self.is_step_start == False:
                self.lastError = 0.0
                self.desired_theta = self.current_theta - 1.57
                self.is_step_start = True
            error = self.fnTurn()
            if math.fabs(error) < 0.05:
                rospy.loginfo("outer_turn_first finished")
                self.current_step_of_parking = self.StepOfParking.parking_lot.value
                self.is_step_start = False

        if self.current_step_of_parking == self.StepOfParking.park.value:
            if self.is_step_start == False:
                self.lastError = 0.0
                self.start_pos_x = self.current_pos_x
                self.start_pos_y = self.current_pos_y
                self.is_step_start = True
            error = self.fnStraight(self.distance/100)  #cm to m unit convert
            if math.fabs(error) < 0.005:
                rospy.loginfo("slot_entry finished")
                twist = Twist()
                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0
                self.pub_cmd.publish(twist)
                rospy.sleep(1)
                self.current_step_of_parking = self.StepOfParking.parked.value
                # self.current_step_of_parking = self.StepOfParking.idle.value
                self.is_step_start = False 
            # rospy.loginfo("outer_turn_first")

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("parking")
        parking()
        rospy.loginfo("parking is started..")
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down line follow node."
        cv2.destroyAllWindows()
#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import numpy as np
import time
import math
import tf
from aerial_robot_msgs.msg import FlightNav
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Empty, UInt8
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CameraInfo
import tf.transformations as tft
from drone_basic_function import DroneBasic
from dog_basic_function import DogBasic

# It is for ground robot used to align with aerial robot

class AprilmoveqilinNode:

    def __init__(self):  # This part will work when this node is used.
        print(f'Hi, I am Cloud Cube')
        rospy.init_node('Aprilmoveqilin', anonymous=True)

        self.lx, self.ly, self.lz = 0, 0, 0
        self.target_x, self.target_y, self.target_z = 0.0, 0.0, 0.0
        self.target_qx,self.target_qy, self.target_qz, self.target_qw = 0.0, 0.0, 0.0, 0.0

        self.time_rece = rospy.Time()


        # Subscribe and publish.
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)

        self.pub_qilin_vel= rospy.Publisher('/go1/cmd_vel', Twist, queue_size=10)

        # Wait the service of ground robot
        rospy.wait_for_service('/go1/sit')
        rospy.wait_for_service('/go1/stand')
        self.service_client_sit = rospy.ServiceProxy('/go1/sit', Trigger)
        self.service_client_stand = rospy.ServiceProxy('/go1/stand', Trigger)

        # Load the parameter
        self.converge_interval = rospy.get_param("/converge_interval")
        self.above_z = rospy.get_param("/above_z")
        self.move_param = rospy.get_param("/move_parameter")
        self.rotate_param = rospy.get_param("/rotate_parameter")
        # self.pose_parameter = rospy.get_param("/pose_parameter")


    def _callback_apriltag(self, data):
        current_time = rospy.Time.now()
        # print(f'apriltag:{current_time.to_sec()}')

        # get the apriltag`s position information compare with camera coordination
        if data.detections:
            #rospy.loginfo("latest arigtarg timestamp: {}".format(data.header.stamp.to_sec()))
            target = data.detections[0]
            self.target_x = target.pose.pose.pose.position.x
            self.target_y = target.pose.pose.pose.position.y
            self.target_qx = target.pose.pose.pose.orientation.x
            self.target_qy = target.pose.pose.pose.orientation.y
            self.target_qz = target.pose.pose.pose.orientation.z
            self.target_qw = target.pose.pose.pose.orientation.w
            self.target_roll = tft.euler_from_quaternion([self.target_qx, self.target_qy, self.target_qz, self.target_qw])[0]
            self.target_pitch = tft.euler_from_quaternion([self.target_qx, self.target_qy, self.target_qz, self.target_qw])[1]
            self.target_yaw = tft.euler_from_quaternion([self.target_qx, self.target_qy, self.target_qz, self.target_qw])[2]
            self.lx = - self.move_param * self.target_y
            self.ly = self.move_param * self.target_x
            self.ryaw = self.rotate_param * self.target_yaw
            apriltag_time = rospy.Time.now()
            print("euler_z (degree):", self.lx, self.ly)
            #print(f'apriltag_time:{apriltag_time.to_sec()}')
            if abs(self.lx) < 5 and abs(self.ly) < 5:
                navigation_time = rospy.Time.now()
                print("enter")
                #print(f'navigation_time:{navigation_time.to_sec()}')
                self.qilin_cmd_vel(self.lx, self.ly, 0, 0, self.ryaw)
                # self.qilin_body_pose(self.qx, self.qy, self.qz, self.qw)
                # self.qilin_body_pose(self.april_qx, self.april_qy, self.april_qz, self.april_qw)
        else:
            self.qilin_cmd_vel(0, 0, 0, 0, 0)
            # self.qilin_body_pose(0, 0, 0, 1)
    def sit(self):
        try:
            response = self.service_client_sit()
            if response.success:
                rospy.loginfo('Sit command executed successfully')
            else:
                rospy.logwarn('Stand command failed: %s', response.message)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)

    def stand(self):
        try:
            response = self.service_client_stand()
            if response.success:
                rospy.loginfo('Stand command executed successfully')
            else:
                rospy.logwarn('Stand command failed: %s', response.message)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)

    def qilin_cmd_vel(self, lx, ly, ax, ay, az):
        qilin_cmd_vel = Twist()
        qilin_cmd_vel.linear.x = lx
        qilin_cmd_vel.linear.y = ly
        qilin_cmd_vel.angular.x = ax
        qilin_cmd_vel.angular.y = ay
        qilin_cmd_vel.angular.z = az

        self.pub_qilin_vel.publish(qilin_cmd_vel)


if __name__ == '__main__':
    node = AprilmoveqilinNode()
    node.stand()
    time.sleep(1)

    while not rospy.is_shutdown():
        rospy.spin()

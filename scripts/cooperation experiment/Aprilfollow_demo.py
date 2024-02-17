#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import time
import math
import tf2_ros
import tf
from std_msgs.msg import Empty, UInt8
from aerial_robot_msgs.msg import FlightNav
from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo
from dynamic_reconfigure.server import Server
# It is for Apirltag following demo for MyAGV

# use the class to create a node


class AprilfollowNode:

    def __init__(self):  # This part will work when this node is used.
        print(f'Hi, I am Cloud Cube')
        rospy.init_node('Aprilfollow', anonymous=True)

        self.agv_x, self.agv_y, self.agv_z = 0.0, 0.0, 0.0
        self.lvol_x, self.lvol_y, self.lvol_z = 0.0, 0.0, 0.0
        self.anglevol_x, self.anglevol_y, self.anglevol_z = 0.0, 0.0, 0.0
        self.april_x, self.april_y, self.april_z = 0.0, 0.0, 0.0
        self._seq = 0
        self.D = 0

        self.time_rece = rospy.Time()

        # Subscribe and publish.
        # rospy.Subscriber('/odom', Odometry, self._callback_position)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)
        rospy.Subscriber('/land_camera_upward/camera_info', CameraInfo, self._callback_camera_raw)
        # rospy.Subscriber('/usb_cam/camera_info', CameraInfo, self._callback_camera_raw)
        self.pub_nav = rospy.Publisher('/cmd_vel', Twist, queue_size=10, tcp_nodelay=True)
        # transport_hint='tcpNoDelay'
        # self.tf_buffer = tf2_ros.Buffer()
        # Initialize a TransformListener
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.set_param('/move_parameter', 0.8)
        self.move_parameter = rospy.get_param("/move_parameter")

    def _callback_apriltag(self, data):
        current_time = rospy.Time.now()
        # print(f'apriltag:{current_time.to_sec()}')

        # get the apriltag`s position information compare with camera coordination
        if data.detections:
            #rospy.loginfo("latest arigtarg timestamp: {}".format(data.header.stamp.to_sec()))
            a = data.detections[0]
            self.april_x = a.pose.pose.pose.position.x
            self.april_y = a.pose.pose.pose.position.y
            self.D = 1
            self.lvol_x = self.move_parameter * self.april_y
            self.lvol_y = - self.move_parameter * self.april_x
            apriltag_time = rospy.Time.now()

            #print(f'apriltag_time:{apriltag_time.to_sec()}')
            if abs(self.lvol_x) < 0.5 and abs(self.lvol_y) < 0.5:
                navigation_time = rospy.Time.now()

                #print(f'navigation_time:{navigation_time.to_sec()}')
                self.agv_nav_info(self.lvol_x, self.lvol_y, 0)
        else:
            self.D = 0
            self.agv_nav_info(0, 0, 0)

        # orientation = detection.pose.pose.pose.orientation

    def _callback_camera_raw(self, data):
        current_time = rospy.Time.now()

        # print(f'camera_info:{current_time.to_sec()}')
        # rospy.loginfo("------------------------------------camera!")
    def agv_nav_info(self, lx, ly, az):
        agv_nav_msg = Twist()
        agv_nav_msg.linear.x = lx
        agv_nav_msg.linear.y = ly
        agv_nav_msg.angular.z = az

        self.pub_nav.publish(agv_nav_msg)

        #rospy.loginfo("pub vel cmd!")

    def dddprint(self):
        while not rospy.is_shutdown():
            rate = rospy.Rate(1)

            print(f'x = {self.april_x},y = {self.april_y}')
            rate.sleep()

    # def follow_camera(self, ctl_rate: int):
    #     rate = rospy.Rate(ctl_rate)
    #     while not rospy.is_shutdown():
    #         if self.D:
    #             self.lvol_x =  self.move_parameter * self.april_y
    #             self.lvol_y =  - self.move_parameter * self.april_x
    #
    #             if -0.5 < self.lvol_x < 0.5 and -0.5< self.lvol_y < 0.5:
    #                 self.agv_nav_info(self.lvol_x, self.lvol_y, 0)
    #
    #             #     if (rospy.Time.now() - self.time_rece).to_sec() > 0.5:
    #             #         self.D = 0
    #
    #         else:
    #             self.agv_nav_info(0, 0, 0)
    #
    #         rate.sleep()


if __name__ == '__main__':
    node = AprilfollowNode()
    # node.follow_camera(10)
    # node.dddprint()
    # while not rospy.is_shutdown():
    rospy.spin()

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

# It is for Apirltag following demo for MyAGV

# use the class to create a node

class AprilfollowNode:

    def __init__(self):  # This part will work when this node is used.
        print(f'Hi, I am Cloud Cube')
        rospy.init_node('Aprilfollow', anonymous=True)

        self.agv_x, self.agv_y, self.agv_z = 0.0, 0.0, 0.0
        self.lvol_x, self.lvol_y, self.lvol_z = 0.0, 0.0, 0.0
        self.avol_x, self.avol_y, self.avol_z = 0.0, 0.0, 0.0
        self.april_x, self.april_y, self.april_z = 0.0, 0.0, 0.0
        self._seq = 0

        # Subscribe and publish.
        #rospy.Subscriber('/odom', Odometry, self._callback_position)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)
        self.pub_nav = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        # Initialize a TransformListener
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
	
      
    def _callback_apriltag(self, data):
        # get the apriltag`s position information compare with camera coordination
	    # use transform to gain the
        transform = self.tf_buffer.lookup_transform('land_camera_upward_frame', 'Target', rospy.Time.now(),rospy.Duration(0.1))
        self.april_x = transform.transform.translation.x
        self.april_y = transform.transform.translation.y
        #self.april_z = transform.transform.translation.z


    # write a AGV nav function
    def agv_nav_info(self, lx, ly, az):
        agv_nav_msg = Twist()
        agv_nav_msg.linear.x = lx
        agv_nav_msg.linear.y = ly
        agv_nav_msg.angular.z = az

        self.pub_nav.publish(agv_nav_msg)

    def dddprint(self):
        while not rospy.is_shutdown():
            rate = rospy.Rate(1)

            print(f'x = {self.april_x},y = {self.april_y}')
            rate.sleep()


    def follow_camera(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.lvol_x = - 0.8 * self.april_x
            self.lvol_y = - 0.8 * self.april_y
            self.agv_nav_info(self.lvol_x, self.lvol_y, 0)

            rate.sleep()


if __name__ == '__main__':
    node = AprilfollowNode()
    node.follow_camera()
    #node.dddprint()
    while not rospy.is_shutdown():
        rospy.spin()

#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import numpy as np
import time
import math
from aerial_robot_msgs.msg import FlightNav
from apriltag_ros.msg import AprilTagDetectionArray
from pandas import set_eng_float_format
from std_msgs.msg import Empty, UInt8
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

# It is for  the Coopration for Mini_Quadrotor and Qilin

# use the class to create a node


class SubscribeeventNode:

    def __init__(self):  # This part will work when this node is used.
        print(f'Hi, I am Cloud Cube')
        rospy.init_node('Aprillandqilin', anonymous=True)

        self._seq = 0
        # Subscribe and publish.
        rospy.Subscriber('/quadrotor/uav/nav/trigger', Empty, self._callback_nav_trigger)
        rospy.Subscriber('/quadrotor/uav/nav/info', FlightNav, self._callback_nav_info)


        # self.pub_event = rospy.Publisher('/uavandgr/event', UInt8, queue_size=10)

        self.pub_drone_nav = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)
        self.pub_takeoff = rospy.Publisher('/quadrotor/teleop_command/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('/quadrotor/teleop_command/land', Empty, queue_size=10)

        self.target_x, self.target_y, self.target_z= 0.0, 0.0, 0.0
        self.yaw_nav_mode, self.target_omega_z, self.target_yaw = 0.0, 0.0, 0.0
    def _callback_nav_info(self, msg):
        self.target_x= msg.target_pos_x
        self.target_y= msg.target_pos_y
        self.target_z= msg.target_pos_z
        self.yaw_nav_mode = msg.yaw_nav_mode
        self.target_omega_z = msg.target_omega_z
        self.target_yaw = msg.target_yaw

    def _callback_nav_trigger(self, msg):
        self.drone_nav_info(self.target_x,self.target_y,self.target_z,self.yaw_nav_mode, self.target_omega_z, self.target_yaw)
        print(f"pub")

    def takeoff(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing takeoff command...")
        empty_msg = Empty()
        self.pub_takeoff.publish(empty_msg)

    # drone land
    def land(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing land command...")
        empty_msg = Empty()
        self.pub_land.publish(empty_msg)

    def drone_nav_info(self, x, y, z, yaw_mode, omega_z, yaw):
        flight_nav_msg = FlightNav()
        flight_nav_msg.header.seq = self._seq
        self._seq += 1
        flight_nav_msg.header.stamp = rospy.Time.now()
        flight_nav_msg.header.frame_id = 'world'

        flight_nav_msg.control_frame = 0
        flight_nav_msg.target = 0
        flight_nav_msg.pos_xy_nav_mode = 2
        flight_nav_msg.target_pos_x = x
        flight_nav_msg.target_vel_x = 0.0
        flight_nav_msg.target_acc_x = 0.0
        flight_nav_msg.target_pos_y = y
        flight_nav_msg.target_vel_y = 0.0
        flight_nav_msg.target_acc_y = 0.0
        flight_nav_msg.yaw_nav_mode = yaw_mode
        flight_nav_msg.target_omega_z = omega_z
        flight_nav_msg.target_yaw = yaw
        flight_nav_msg.pos_z_nav_mode = 2
        flight_nav_msg.target_pos_z = z
        flight_nav_msg.target_vel_z = 0.0
        flight_nav_msg.target_pos_diff_z = 0.0

        self.pub_drone_nav.publish(flight_nav_msg)





if __name__ == '__main__':
    node = SubscribeeventNode()
    time.sleep(1)

    while not rospy.is_shutdown():
        rospy.spin()

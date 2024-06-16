#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import numpy as np
import time
import math
from aerial_robot_msgs.msg import FlightNav
from apriltag_ros.msg import AprilTagDetectionArray
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

        self.lx, self.ly, self.lz = 0, 0, 0
        self.qx, self.qy, self.qz, self.qw = 0, 0, 0, 0
        self.april_x, self.april_y, self.april_z = 0.0, 0.0, 0.0
        self.april_qx,self.april_qy, self.april_qz, self.april_qw = 0.0, 0.0, 0.0, 0.0

        self.drone_x, self.drone_y, self.drone_z = 0.0, 0.0, 0.0
        self.takeoff_x, self.takeoff_y, self.takeoff_z = 0.0, 0.0, 0.0

        self.D = 0
        self.time_rece = rospy.Time()
        self._seq = 0
        self.state = 0
        self.flag_takeoff = 1
        self.flag_landon = 1
        self.flag_nav = 1
        # Subscribe and publish.
        rospy.Subscriber('/uavandgr/event', UInt8, self._callback_event)
        rospy.Subscriber('/quadrotor/uav/cog/odom', Odometry, self._callback_position)
        rospy.Subscriber('/quadrotor/flight_state', UInt8, self._callback_state)

        # self.pub_event = rospy.Publisher('/uavandgr/event', UInt8, queue_size=10)

        self.pub_drone_nav = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)
        self.pub_takeoff = rospy.Publisher('/quadrotor/teleop_command/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('/quadrotor/teleop_command/land', Empty, queue_size=10)

        rospy.set_param('/converge_interval', 0.05)
        self.converge_interval = rospy.get_param("/converge_interval")
        rospy.set_param('/above_z', 0.4)
        self.above_z = rospy.get_param("/above_z")

        rospy.set_param('/move_parameter', 2)
        self.move_parameter = rospy.get_param("/move_parameter")
        rospy.set_param('/pose_parameter', 0.05)
        self.pose_parameter = rospy.get_param("/pose_parameter")

    def _callback_position(self, odom_msg):
        self.drone_x = odom_msg.pose.pose.position.x
        self.drone_y = odom_msg.pose.pose.position.y
        self.drone_z = odom_msg.pose.pose.position.z

    def _callback_state(self, msg):
        self.state = msg.data

    def _callback_event(self,msg):
        self.event = msg.data
        if self.flag_takeoff == 1 and self.event == 1 :
            self.flag_takeoff = 0
            self.takeoff()
            print('take off')
        if self.flag_nav == 1 and self.event == 2:
            self.flag_nav = 0
            tz = self.takeoff_z + self.above_z
            self.drone_nav_info(self.takeoff_x + 0.2, self.takeoff_y + 0.1, tz + 0.5)
            print(f'move to {self.takeoff_x + 0.3}, {self.takeoff_y + 0.1}, {self.takeoff_z + 0.5}' )
            time.sleep(4)
            self.drone_nav_info(self.takeoff_x, self.takeoff_y, tz)
            time.sleep(1)
        if self.flag_landon == 1 and self.event == 3:
            self.flag_landon = 0
            self.land()
            print(f'land on')

    def event(self, x):
        event_msgs = UInt8()
        event_msgs.data = x
        self.pub_event.publish(event_msgs)

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

    def converge(self, x, y, z):
        while not rospy.is_shutdown():
            if abs(self.drone_x - x) < self.converge_interval and abs(self.drone_y - y) < self.converge_interval and abs(self.drone_z - z) < self.converge_interval:  # reach hover state
                break
            time.sleep(0.1)

    # record the take off position
    def record_takeoff_position(self):
        self.takeoff_x = self.drone_x
        self.takeoff_y = self.drone_y
        self.takeoff_z = self.drone_z
        print(self.takeoff_x, self.takeoff_y)

    def drone_nav_info(self, x, y, z):
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
        flight_nav_msg.yaw_nav_mode = 0
        flight_nav_msg.target_omega_z = 0.0
        flight_nav_msg.target_yaw = 0.0
        flight_nav_msg.pos_z_nav_mode = 2
        flight_nav_msg.target_pos_z = z
        flight_nav_msg.target_vel_z = 0.0
        flight_nav_msg.target_pos_diff_z = 0.0

        self.pub_drone_nav.publish(flight_nav_msg)





if __name__ == '__main__':
    node = SubscribeeventNode()
    time.sleep(1)
    node.record_takeoff_position()
    while not rospy.is_shutdown():
        rospy.spin()

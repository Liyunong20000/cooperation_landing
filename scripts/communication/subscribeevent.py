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
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

# It is for  the Coopration for Mini_Quadrotor and Qilin

# use the class to create a node


class SubscribeeventNode:

    def __init__(self):  # This part will work when this node is used.
        print(f'Hi, I am Cloud Cube')
        rospy.init_node('Aprillandqilin', anonymous=True)

        self._seq = 0
        # Subscribe and publish.
        rospy.Subscriber('/quadrotor/target_pose/info', PoseStamped, self._callback_target_pose_info)
        rospy.Subscriber('/quadrotor/target_pose/trigger', Empty, self._callback_target_pose_trigger)
        # rospy.Subscriber('/quadrotor/uav/nav/info', FlightNav, self._callback_nav_info)


        # self.pub_event = rospy.Publisher('/uavandgr/event', UInt8, queue_size=10)
        self.pub_drone_target = rospy.Publisher('/quadrotor/target_pose', PoseStamped, queue_size=10)
        # self.pub_drone_nav = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)
        self.pub_takeoff = rospy.Publisher('/quadrotor/teleop_command/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('/quadrotor/teleop_command/land', Empty, queue_size=10)

        self.x_y_mode, self.z_mode = 0, 0
        self.target_x, self.target_y, self.target_z= 0.0, 0.0, 0.0
        self.target_ox, self.target_oy, self.target_oz, self.target_ow = 0.0, 0.0, 0.0, 0.0
        self.yaw_nav_mode, self.target_omega_z, self.target_yaw = 0, 0.0, 0.0
    # def _callback_nav_info(self, msg):
    #     self.x_y_mode= msg.pos_xy_nav_mode
    #     self.target_x= msg.target_pos_x
    #     self.target_y= msg.target_pos_y
    #     self.z_mode= msg.pos_z_nav_mode
    #     self.target_z= msg.target_pos_z
    #     self.yaw_nav_mode = msg.yaw_nav_mode
    #     self.target_omega_z = msg.target_omega_z
    #     self.target_yaw = msg.target_yaw

    def _callback_target_pose_info(self, msg):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.target_z = msg.pose.position.z
        self.target_ox = msg.pose.orientation.x
        self.target_oy = msg.pose.orientation.y
        self.target_oz = msg.pose.orientation.z
        self.target_ow = msg.pose.orientation.w

    def _callback_target_pose_trigger(self,msg):
        self.drone_target_pose(self.target_x, self.target_y, self.target_z, self.target_ox, self.target_oy, self.target_oz, self.target_ow)
        print(f"pub target pose")

    def _callback_nav_trigger(self, msg):
        self.drone_nav_info(self.x_y_mode, self.target_x, self.target_y, self.z_mode, self.target_z,self.yaw_nav_mode, self.target_omega_z, self.target_yaw)
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

    def drone_nav_info(self, x_y_mode, x, y, z_mode, z, yaw_mode, omega_z, yaw):
        flight_nav_msg = FlightNav()
        flight_nav_msg.header.seq = self._seq
        self._seq += 1
        flight_nav_msg.header.stamp = rospy.Time.now()
        flight_nav_msg.header.frame_id = 'world'

        flight_nav_msg.control_frame = 0
        flight_nav_msg.target = 0
        flight_nav_msg.pos_xy_nav_mode = x_y_mode
        flight_nav_msg.target_pos_x = x
        flight_nav_msg.target_vel_x = 0.0
        flight_nav_msg.target_acc_x = 0.0
        flight_nav_msg.target_pos_y = y
        flight_nav_msg.target_vel_y = 0.0
        flight_nav_msg.target_acc_y = 0.0
        flight_nav_msg.yaw_nav_mode = yaw_mode
        flight_nav_msg.target_omega_z = omega_z
        flight_nav_msg.target_yaw = yaw
        flight_nav_msg.pos_z_nav_mode = z_mode
        flight_nav_msg.target_pos_z = z
        flight_nav_msg.target_vel_z = 0.0
        flight_nav_msg.target_pos_diff_z = 0.0

        self.pub_drone_nav.publish(flight_nav_msg)

    def drone_target_pose(self, x, y, z, ox, oy, oz, ow):
        drone_target_pose = PoseStamped()
        drone_target_pose.header.frame_id = 'world'
        drone_target_pose.pose.position.x = x
        drone_target_pose.pose.position.y = y
        drone_target_pose.pose.position.z = z
        drone_target_pose.pose.orientation.x = ox
        drone_target_pose.pose.orientation.y = oy
        drone_target_pose.pose.orientation.z = oz
        drone_target_pose.pose.orientation.w = ow

        self.pub_drone_target.publish(drone_target_pose)




if __name__ == '__main__':
    node = SubscribeeventNode()
    time.sleep(1)

    while not rospy.is_shutdown():
        rospy.spin()

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
import numpy as np

# It is for  the first drone BoomBoom
# use the class to create a node

class AprillandNode:

    def __init__(self):  # This part will work when this node is used.
        print(f'Hi, I am Cloud Cube')
        rospy.init_node('Aprilland', anonymous=True)

        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.Px, self.Py, self.Pz = 0.0, 0.0, 0.0
        self.Rx, self.Ry, self.Rz = 0.0, 0.0, 0.0
        self.Tx, self.Ty, self.Tz= 0.0, 0.0, 0.0
        self.I = 2
        self._seq = 0
        self.state = 0

        # Subscribe and publish.
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)
        rospy.Subscriber('/quadrotor/uav/cog/odom', Odometry, self._callback_position)
        rospy.Subscriber('/quadrotor/flight_state', UInt8, self._callback_state)
        self.pub_nav = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)
        self.pub_takeoff = rospy.Publisher('/quadrotor/teleop_command/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('/quadrotor/teleop_command/land', Empty, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer()
        # Initialize a TransformListener
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # transformation

    def get_user_input(self):  # Use to input the number
        self.x = float(input("Enter x value: "))
        self.y = float(input("Enter y value: "))
        self.z = float(input("Enter z value: "))

    def takeoff(self):  # Use to land
        time.sleep(0.5)
        rospy.loginfo("Publishing takeoff command...")
        empty_msg = Empty()
        self.pub_takeoff.publish(empty_msg)

    def land(self):  # Use to land
        time.sleep(0.5)
        rospy.loginfo("Publishing land command...")
        empty_msg = Empty()
        self.pub_land.publish(empty_msg)

    def _callback_apriltag(self, data):
        # Get the information of the distance between the drone and the apriltag
        transform = self.tf_buffer.lookup_transform('world', 'Target', rospy.Time.now(),rospy.Duration(0.1))
        self.Rx = transform.transform.translation.x
        self.Ry = transform.transform.translation.y
        self.Rz = transform.transform.translation.z
        #print(f'Apriltag position')
        #print( self.Rx, self.Ry )
        # print(f'{self.Rx}  {self.Ry}  {self.Rz}')


    def _callback_position(self, odom_msg):  # Get the position information of the drone
        self.Px = odom_msg.pose.pose.position.x
        self.Py = odom_msg.pose.pose.position.y
        self.Pz = odom_msg.pose.pose.position.z

    def _callback_state(self, msg):
        self.state = msg.data

    def record_takeoff_position(self):
        self.Tx = self.Px
        self.Ty = self.Py
        self.Tz = self.Pz
        #print(self.Px, self.Py)
        print(self.Tx, self.Ty)

    def nav_info(self, x, y, z):
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

        self.pub_nav.publish(flight_nav_msg)

    def publish_flight_nav(self):  # Set a beginning point
        # print(f'Type the begin point')
        # Type the number
        # self.get_user_input()
        # self.nav_info(1, 0, 1.0)
        # time.sleep(3)
        # while not rospy.is_shutdown():
        # if -0.05 < self.Px < 0.05:
        # if -0.05 < self.Py - 0 < 0.05:
        # print(f'I have arrived!')
        # break
        self.nav_info(0.6, 0, 1.0)
        time.sleep(3)
        # self.nav_info(0.3, 0, 0.7)
        # time.sleep(3)

    def land_camera(self):
        # print(f'Type the goal point')
        # self.get_user_input()
        while not rospy.is_shutdown():
            if self.state == 5: # reach hover state
                break
            time.sleep(0.1)

        tz = self.Tz + 0.3
        self.nav_info(self.Tx, self.Ty, tz)
        print(f'Move to higher Z')
        while not rospy.is_shutdown():
            if self.Pz - tz < 0.03:
                break
            time.sleep(0.1)
        print(f'Wait for 3 seconds!')
        time.sleep(3)

        while not rospy.is_shutdown():
            x_values = []
            y_values = []
            for i in range (5):
                x_values.append(self.Rx)
                y_values.append(self.Ry)
                time.sleep(1)

            x_data = np.array(x_values)
            y_data = np.array(y_values)
            print(f'{x_data}, {y_data}')
            # 定义移动平均的窗口大小（你可以根据需要调整）
            window_size = 3

            # 计算 x 和 y 的移动平均值
            middle_rx = np.convolve(x_data, np.ones(window_size) / window_size, mode='valid')
            middle_ry = np.convolve(y_data, np.ones(window_size) / window_size, mode='valid')
            print(f'the apriltags position is rx ={middle_rx}, ry = {middle_ry}')
            smoothed_rx = np.convolve(middle_rx, np.ones(window_size) / window_size, mode='valid')
            smoothed_ry = np.convolve(middle_ry, np.ones(window_size) / window_size, mode='valid')
            print(f'the apriltags position is rx ={smoothed_rx}, ry = {smoothed_ry}')
            if smoothed_rx !=0 and smoothed_ry != 0:
                self.nav_info(smoothed_rx, smoothed_ry, tz)
                while not rospy.is_shutdown():
                    if (self.Px - smoothed_rx) < 0.03 and (self.Py - smoothed_ry) < 0.03:
                        break
                    time.sleep(0.1)
                break
            #break

        while not rospy.is_shutdown():
            if math.sqrt((self.Px - smoothed_rx) ** 2 + (self.Py - smoothed_ry) ** 2) < 0.01:
                print(f'Safe landing!')
                print(f'Before landing on, the position is X = {self.Px}, Y = {self.Py}')
                self.land()
                time.sleep(1)
                print(f'After landing on, the position is X = {self.Px}, Y = {self.Py}')
                sys.exit()



if __name__ == '__main__':
    node = AprillandNode()
    time.sleep(3)
    # node.publish_flight_nav()
    node.record_takeoff_position()
    node.takeoff()
    node.land_camera()

    while not rospy.is_shutdown():
        rospy.spin()

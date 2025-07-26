#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import numpy as np
import time
from aerial_robot_msgs.msg import FlightNav
from std_msgs.msg import Empty, UInt8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelState, ModelStates

import tf.transformations as tft

# It is for  the Coopration for Mini_Quadrotor and Qilin

# use the class to create a node

class DroneBasic:

    def __init__(self):  # This part will work when this node is used.
        print(f'This is the basic class for drone')

        self.drone_x, self.drone_y, self.drone_z = 0.0, 0.0, 0.0
        self.drone_qx, self.drone_qy, self.drone_qz, self.drone_qw = 0.0, 0.0, 0.0, 0.0
        self.drone_roll, self.drone_pitch, self.drone_yaw  = 0.0, 0.0, 0.0

        self.drone_state = 0

        self.robot_ns = rospy.get_param("~robot_ns", "xuanwu")

        # Subscribe and publish.

        rospy.Subscriber(self.robot_ns + '/uav/cog/odom', Odometry, self._callback_drone_position)
        rospy.Subscriber(self.robot_ns + '/flight_state', UInt8, self._callback_drone_state)

        self.pub_drone_nav = rospy.Publisher(self.robot_ns + '/uav/nav', FlightNav, queue_size=10)
        self.pub_drone_target = rospy.Publisher(self.robot_ns + '/uav/target_pose', FlightNav, queue_size=10)
        self.pub_drone_start = rospy.Publisher(self.robot_ns + '/teleop_command/start', Empty, queue_size=10)

        self.pub_drone_takeoff = rospy.Publisher(self.robot_ns + '/teleop_command/takeoff', Empty, queue_size=10)
        self.pub_drone_land = rospy.Publisher(self.robot_ns + '/teleop_command/land', Empty, queue_size=10)

    def _callback_drone_position(self, msg):
        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y
        self.drone_z = msg.pose.pose.position.z
        self.drone_qx = msg.pose.pose.orientation.x
        self.drone_qy = msg.pose.pose.orientation.y
        self.drone_qz = msg.pose.pose.orientation.z
        self.drone_qw = msg.pose.pose.orientation.w
        self.drone_roll = tft.euler_from_quaternion([self.drone_qx, self.drone_qy, self.drone_qz, self.drone_qw])[0]
        self.drone_pitch = tft.euler_from_quaternion([self.drone_qx, self.drone_qy, self.drone_qz, self.drone_qw])[1]
        self.drone_yaw = tft.euler_from_quaternion([self.drone_qx, self.drone_qy, self.drone_qz, self.drone_qw])[2]

    def _callback_drone_state(self, msg):
        self.drone_state = msg.data

    def drone_start(self):  # Use to takeoff
        time.sleep(0.5)
        rospy.loginfo("Publishing start command...")
        empty_msg = Empty()
        self.pub_drone_start.publish(empty_msg)

    def drone_takeoff(self):  # Use to takeoff
        time.sleep(0.5)
        rospy.loginfo("Publishing takeoff command...")
        empty_msg = Empty()
        self.pub_drone_takeoff.publish(empty_msg)

    def drone_land(self):  # Use to land
        time.sleep(0.5)
        rospy.loginfo("Publishing land command...")
        empty_msg = Empty()
        self.pub_drone_land.publish(empty_msg)

    def drone_nav(self, x, y, z):
        flight_nav_msg = FlightNav()
        # flight_nav_msg.header.seq = self._seq
        # self._seq += 1
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

    def drone_target(self,x,y,z,ox,oy,oz,ow):
        drone_target_pose = PoseStamped()
        drone_target_pose.header.frame_id = 'world'
        drone_target_pose.pose.position.x = x
        drone_target_pose.pose.position.y = y
        drone_target_pose.pose.position.z = z
        drone_target_pose.pose.orientation.x = ox
        drone_target_pose.pose.orientation.y = oy
        drone_target_pose.pose.orientation.z = oz
        drone_target_pose.pose.et.publish(drone_target_pose)

    def record_takeoff_position(self):
        self.takeoff_x = self.drone_x
        self.takeoff_y = self.drone_y
        self.takeoff_z = self.drone_z
        self.takeoff_yaw = self.drone_yaw
        print(f'takeoff position:{self.takeoff_x}, {self.takeoff_y},{self.takeoff_z}, {self.takeoff_yaw}')

class DroneBasicSim:
    def __init__(self):
        rospy.Subscriber('/gazebo/model_states', ModelStates, self._callback_aerial_robot_pose)

        self.sim_drone_x, self.sim_drone_y, self.sim_drone_z = 0.0, 0.0, 0.0
        self.sim_drone_qx, self.sim_drone_qy, self.sim_drone_qz, self.sim_drone_qw = 0.0, 0.0, 0.0, 0.0
        self.sim_drone_roll, self.sim_drone_pitch, self.sim_drone_yaw  = 0.0, 0.0, 0.0

    def _callback_aerial_robot_pose(self, msg):

            index = msg.name.index("xuanwu")  # Find index of the robot
            pose = msg.pose[index]
            self.sim_drone_x = pose.position.x
            self.sim_drone_y = pose.position.y
            self.sim_drone_z = pose.position.z
            self.sim_drone_qx = pose.orientation.x
            self.sim_drone_qy = pose.orientation.y
            self.sim_drone_qz = pose.orientation.z
            self.sim_drone_qw = pose.orientation.w

            self.sim_drone_roll = tft.euler_from_quaternion([self.sim_drone_qx, self.sim_drone_qy, self.sim_drone_qz, self.sim_drone_qw])[0]
            self.sim_drone_pitch = \
        tft.euler_from_quaternion([self.sim_drone_qx, self.sim_drone_qy, self.sim_drone_qz, self.sim_drone_qw])[1]
            self.sim_drone_yaw = \
        tft.euler_from_quaternion([self.sim_drone_qx, self.sim_drone_qy, self.sim_drone_qz, self.sim_drone_qw])[2]
            print(f'{self.sim_drone_x},{self.sim_drone_y}')
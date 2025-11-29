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
from spinal.msg import ServoStates, ServoControlCmd
from gripper.gripper_move import GripperMoveNode
from drone_basic_function import DroneBasic


# It is for  the Coopration for Mini_Quadrotor and Qilin

# use the class to create a node


class EventtriggerNode:

    def __init__(self):  # This part will work when this node is used.
        print(f'Hi, I am Cloud Cube')
        rospy.init_node('eventtrigger', anonymous=True)

        # Subscribe and publish.
        rospy.Subscriber('/xuanwu/target_pose/info', PoseStamped, self._callback_target_pose_info)
        rospy.Subscriber('/xuanwu/target_pose/trigger', Empty, self._callback_target_pose_trigger)
        # rospy.Subscriber('/quadrotor/uav/nav/info', FlightNav, self._callback_nav_info)
        rospy.Subscriber('/xuanwu/servo/target_states/info', ServoControlCmd, self._callback_servo_target_states_info)
        rospy.Subscriber('/xuanwu/servo/target_states/trigger', Empty, self._callback_servo_target_states_trigger)
        rospy.Subscriber('/xuanwu/servo/return/trigger', Empty, self._callback_servo_return_trigger)
        rospy.Subscriber('/xuanwu/add_extra_module/trigger', Empty, self._callback_add_module_trigger)
        rospy.Subscriber('/xuanwu/remove_extra_module/trigger', Empty, self._callback_remove_module_trigger)

        # self.pub_event = rospy.Publisher('/uavandgr/event', UInt8, queue_size=10)
        self.pub_drone_target = rospy.Publisher('/xuanwu/target_pose', PoseStamped, queue_size=10)
        self.pub_servo_target = rospy.Publisher('/xuanwu/servo/target_states', ServoControlCmd, queue_size=10)

        # self.pub_drone_nav = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)
        # self.pub_takeoff = rospy.Publisher('/quadrotor/teleop_command/takeoff', Empty, queue_size=10)
        # self.pub_land = rospy.Publisher('/quadrotor/teleop_command/land', Empty, queue_size=10)

        self.gripper_move = GripperMoveNode()
        self.drone_basic = DroneBasic()

        self.x_y_mode, self.z_mode = 0, 0
        self.target_x, self.target_y, self.target_z= 0.0, 0.0, 0.0
        self.target_ox, self.target_oy, self.target_oz, self.target_ow = 0.0, 0.0, 0.0, 0.0
        self.yaw_nav_mode, self.target_omega_z, self.target_yaw = 0, 0.0, 0.0
        self.servo_index = []
        self.servo_angle = []
    # def _callback_nav_info(self, msg):
    #     self.x_y_mode= msg.pos_xy_nav_mode
    #     self.target_x= msg.target_pos_x
    #     self.target_y= msg.target_pos_y
    #     self.z_mode= msg.pos_z_nav_mode
    #     self.target_z= msg.target_pos_z
    #     self.yaw_nav_mode = msg.yaw_nav_mode
    #     self.target_omega_z = msg.target_omega_z
    #     self.target_yaw = msg.target_yaw
    # Capture the navigation information from ~drone_ns/target_pose topic
    def _callback_target_pose_info(self, msg):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.target_z = msg.pose.position.z
        self.target_ox = msg.pose.orientation.x
        self.target_oy = msg.pose.orientation.y
        self.target_oz = msg.pose.orientation.z
        self.target_ow = msg.pose.orientation.w
    # Trigger the target_pose navigation topic of drone
    def _callback_target_pose_trigger(self, msg):
        if self.target_x == 0 and self.target_y == 0 and self.target_z == 0:
            return
        self.drone_target_pose(self.target_x, self.target_y, self.target_z, self.target_ox, self.target_oy, self.target_oz, self.target_ow)
        print(f"pub target pose")

    def _callback_nav_trigger(self, msg):
        self.drone_nav_info(self.x_y_mode, self.target_x, self.target_y, self.z_mode, self.target_z,self.yaw_nav_mode, self.target_omega_z, self.target_yaw)
        print(f"pub")
    # Get the /target_state_info from dog side
    def _callback_servo_target_states_info(self, msg):
        self.servo_index = msg.index
        self.servo_angle = msg.angles
        # print(f'{self.servo_index} {self.servo_angle}')
    # If the trigger from dog side published, the trigger for drone side will respond
    def _callback_servo_target_states_trigger(self, msg):
        rospy.sleep(0.1)
        self.gripper_move.servo_target_cmd(0, 1150)
    # Trigger for gripper to return zero point
    def _callback_servo_return_trigger(self, msg):
        rospy.sleep(0.2)
        self.gripper_move.return_zero()
    # trigger for add extra module
    def _callback_add_module_trigger(self, msg):
        rospy.sleep(0.1)
        self.drone_basic.call_add_extra_module(1, 'brick', 'main_body')
    # trigger for remove extra module
    def _callback_remove_module_trigger(self, msg):
        rospy.sleep(0.1)
        self.drone_basic.call_add_extra_module(-1, 'brick', 'main_body')
    # def gripper_move_event(self, servo_index, servo_angle):
    #     servo_target_cmd = ServoControlCmd()
    #     servo_target_cmd.index = self.servo_index
    #     servo_target_cmd.angles = self.servo_angle
    #     time.sleep(0.1)
    #     self.pub_servo_target.publish(servo_target_cmd)
    #     print(f'servo_target_cmd:{servo_target_cmd}')
    #

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
        time.sleep(0.5)
        self.pub_drone_target.publish(drone_target_pose)




if __name__ == '__main__':
    node = EventtriggerNode()
    time.sleep(1)

    while not rospy.is_shutdown():
        rospy.spin()

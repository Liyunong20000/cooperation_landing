#! /usr/bin/env python
# -*- coding: utf-8 -*-
from logging import shutdown

import rospy, sys
import numpy as np
import time, math, threading
import smach
import smach_ros
import tf.transformations as tft
from appdirs import user_data_dir
from lxml.html import Classes
from std_msgs.msg import Empty, UInt8
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist, PoseStamped
from gazebo_msgs.msg import ModelState, ModelStates
from tf.tfwtf import rostime_delta
from aerial_robot_msgs.msg import FlightNav
from apriltag_ros.msg import AprilTagDetectionArray

from basic_function import *
from dog_basic_function import *
from drone_basic_function import *
from gripper.gripper_move import *
# from simulation.sim_basic import *
# from simulation.sim_links_attachment import *

# The Cooperative inspection system by mini_quadrotor and Qilin
class Start(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['rm'])
        self.dog_basic = DogBasic()

        self.rm = 0

    def execute(self, userdata):
        self.rm = userdata.rm
        if self.rm == 1:
            self.dog_basic.stand()
        print(f'pass begin.')
        return 'succeeded'

class TargetSearch(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['rm'])
        self.dog_basic = DogBasic()
        self.drone_basic = DroneBasic()
        self.gripper_move = GripperMoveNode
        # self.drone_basic_sim = DroneBasicSim()
        # self.sim_basic = SimbasicNode()

        self.pick_position = []
        self.rm = 0
        self.sim_dog_x, self.sim_dog_y, self.sim_dog_z = 0.0, 0.0, 0.0
        self.sim_dog_roll, self.sim_dog_pitch, self.sim_dog_yaw= 0.0, 0.0, 0.0
        self.sim_target_x, self.sim_target_y, self.sim_target_z = 0.0, 0.0, 0.0
        self.sim_target_roll, self.sim_target_pitch, self.sim_target_yaw= 0.0, 0.0, 0.0
        self.msg_apriltag = None



        rospy.Subscriber('/xuanwu/tag_detections', AprilTagDetectionArray, self._callback_simulation_apriltag)

    def _callback_simulation_apriltag(self, msg):
        self.msg_apriltag = msg

    def sim_apriltag_position(self,data,target_id):
        for det in data.detections:
            # print(f'{det}')
            # print(f'{det.id}')
            if target_id in det.id:
                pose = det.pose.pose.pose
                self.sim_target_x = pose.position.x
                self.sim_target_y = pose.position.y
                self.sim_target_z = pose.position.z
                qx = pose.orientation.x
                qy = pose.orientation.y
                qz = pose.orientation.z
                qw = pose.orientation.w
                self.sim_target_roll = \
                tft.euler_from_quaternion([qx, qy, qz, qw])[0]
                self.sim_target_pitch = \
                tft.euler_from_quaternion([qx, qy, qz, qw])[1]
                self.sim_target_yaw = \
                tft.euler_from_quaternion([qx, qy, qz, qw])[2]

        print(f'{self.sim_target_x}, {self.sim_target_y}, {self.sim_target_z}')

    def execute(self, userdata):
        self.rm = userdata.rm

        if self.rm == 1:

            while self.drone_basic.drone_x < 0:
                self.dog_basic.qilin_cmd_vel(0.3, 0, 0, 0, 0)
                time.sleep(0.1)
                print(f'{self.drone_basic.drone_x}')
            self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)

            while abs(self.drone_basic.drone_yaw - 1.57) > 0.05:
                self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, -(self.drone_basic.drone_yaw - 1.57))
                time.sleep(0.1)
                if abs(self.drone_basic.drone_yaw - 1.57) < 0.02:
                    break
            self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
            rospy.loginfo(f'enter y')
            while abs(self.drone_basic.drone_y - 1) > 0.1:
                self.dog_basic.qilin_cmd_vel(- (self.drone_basic.drone_y - 1),0, 0, 0, 0)
                time.sleep(0.1)
                if abs(self.drone_basic.drone_y - 1) < 0.05:
                    break
            self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
            rospy.loginfo(f'out y')

            self.drone_basic.tag_position(self.drone_basic.tag_info, 11)
            rospy.loginfo(f'find tag')

            while abs(self.drone_basic.tag_target_z - 0.87) > 0.01 or abs(self.drone_basic.tag_target_x + 0.03) > 0.01 or abs(self.drone_basic.tag_target_pitch) > 0.1:
                self.drone_basic.tag_position(self.drone_basic.tag_info, 11)
                # if self.drone_basic.tag_info == 0:
                #     rospy.logwarn("No tag detections.")
                #     self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
                #     return
                self.dog_basic.qilin_cmd_vel(0.2 * (self.drone_basic.tag_target_z - 0.87), - 0.8 * (self.drone_basic.tag_target_x + 0.03), 0, 0, - 0.5* self.drone_basic.tag_target_pitch)
                time.sleep(0.1)
                if (0.2 * (self.drone_basic.tag_target_z - 0.87) < 0.02) and (abs(-0.8 * (self.drone_basic.tag_target_x + 0.03)) < 0.03) and ((- 0.5* self.drone_basic.tag_target_pitch) < 0.05):
                    break
            # while abs(self.drone_basic.tag_target_x + 0.15) > 0.01:
            #     self.drone_basic.tag_position(self.drone_basic.tag_info, 11)
            #     # if self.drone_basic.tag_info == 0:
            #     #     rospy.logwarn("No tag detections.")
            #     #     self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
            #     #     return
            #     self.dog_basic.qilin_cmd_vel(0, 1 * -(self.drone_basic.tag_target_x + 0.15), 0, 0, 0)
            #     print(f'I got it!')
            time.sleep(0.1)
            self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
            time.sleep(1)
            self.dog_basic.sit()
            time.sleep(3)
            # while abs(self.drone_basic.tag_target_pitch) > 0.05:
            #     self.drone_basic.tag_position(self.drone_basic.tag_info, 11)
            #     if self.drone_basic.tag_info == 0:
            #         rospy.logwarn("No tag detections.")
            #         self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
            #         return
            #     self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, - self.drone_basic.tag_target_pitch
            #     time.sleep(0.1)
            # self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)

            # while abs(self.drone_basic.tag_target_z - 1.27) > 0.01:
            #     self.drone_basic.tag_position(self.drone_basic.tag_info, 11)
            #     # if self.drone_basic.tag_info == 0:
            #     #     rospy.logwarn("No tag detections.")
            #     #     self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
            #     #     return
            #     self.dog_basic.qilin_cmd_vel(0.6 * (self.drone_basic.tag_target_z - 1.27), 0, 0, 0, 0)
            #     time.sleep(0.1)
            # self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)

            return 'succeeded'

        else:
            while self.drone_basic_sim.sim_drone_x < 2.5:
                self.sim_dog_x = 0.5
                self.sim_basic.sim_robot_twist('go1_gazebo', 0.5, 0, 0, 0, 0, 0)
                time.sleep(0.5)
            while abs(self.drone_basic_sim.sim_drone_yaw + 1.57) > (3.14/8):
                self.sim_dog_yaw = 10
                self.sim_basic.sim_robot_twist('go1_gazebo', 0, 0, 0, 0, 0, self.sim_dog_yaw)
                time.sleep(0.5)
            while self.drone_basic_sim.sim_drone_y < 1.3:
                self.sim_dog_y = 0.5
                self.sim_basic.sim_robot_twist('go1_gazebo', 0.5, 0, 0, 0, 0, 0)
                time.sleep(0.5)
                self.sim_apriltag_position(self.msg_apriltag, 11)
                if 1.0 < self.drone_basic_sim.sim_drone_y < 1.5:
                    if self.sim_target_x != 0:
                        print(f'I got it!')
                        break

            while abs(self.sim_target_z) > 1.5 :
                self.sim_apriltag_position(self.msg_apriltag, 11)
                self.sim_basic.sim_robot_twist('go1_gazebo', 0.5 * self.sim_target_z, 0.5 * self.sim_target_x, 0, 0, 0, 0)
                time.sleep(0.5)
            while abs(self.sim_target_x) > 0.1:
                self.sim_apriltag_position(self.msg_apriltag, 11)
                self.sim_basic.sim_robot_twist('go1_gazebo', 0, 0.5 * self.sim_target_x, 0, 0, 0, 0.85 * abs(self.drone_basic_sim.sim_drone_yaw - self.sim_target_roll))
                time.sleep(0.5)
            return 'succeeded'

class Pick(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['rm'])
        # self.sim_basic = SimbasicNode()
        # self.link_attachment = AttachlinksNode()
        self.user_input = 0

        self.gripper_move = GripperMoveNode()
        self.dog_basic = DogBasic()
        self.drone_basic = DroneBasic()
    def execute(self, userdata):
        self.rm = userdata.rm
        if self.rm == 0:
            self.link_attachment.sim_attach_links("xuanwu", "root", "brick", "brick")
            self.sim_basic.call_add_extra_module(1, "brick", "main_body")

        if self.rm == 1:
            self.gripper_move.servo_target_cmd_qilin(0, -10)
            #
            # self.user_input = input()
            # if self.user_input == 'y':
            #     print(f'yes')
            #     self.gripper_move.servo_target_cmd_qilin(0,1150)
            #     rospy.sleep(5)
            #     # self.drone_basic.call_add_extra_module(1,"brick", "main_body")
            self.dog_basic.stand()

        return 'succeeded'

class MoveDestination(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['rm'], output_keys=['put_position'])
        self.TargetSearch = TargetSearch()
        self.rm = 0
        self.put_position = [0.0, 0.0, 0.0]
        self.desk_dimension = [0.0, 0.0, 0.0]
        self.drone_basic = DroneBasic()
        self.dog_basic = DogBasic()
        self.basic = BasicNode()

    def execute(self, userdata):
        self.rm = userdata.rm
        # if self.rm == 1:
        #     while abs(self.drone_basic.drone_y- 1.0) > 0.1:
        #         self.dog_basic.qilin_cmd_vel( -(self.drone_basic.drone_y - 1.0), 0, 0, 0, 0)
        #         time.sleep(0.1)
        #     while abs(self.drone_basic.drone_yaw + 1.57) > 0.01:
        #         self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, -(self.drone_basic.drone_yaw + 1.57))
        #         time.sleep(0.1)
        #     self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
        #     while abs(self.drone_basic.drone_y) > 0.1:
        #         self.dog_basic.qilin_cmd_vel( (self.drone_basic.drone_y - 0.1), 0, 0, 0, 0)
        #         self.drone_basic.tag_position(self.drone_basic.tag_info, 12)
        #         time.sleep(0.1)
        #     while abs(self.drone_basic.tag_target_z - 0.7) > 0.05:
        #         self.drone_basic.tag_position(self.drone_basic.tag_info, 12)
        #         if self.drone_basic.tag_info == 0:
        #             rospy.logwarn("No tag detections.")
        #             self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
        #             return
        #         self.dog_basic.qilin_cmd_vel(2 * (self.drone_basic.tag_target_z - 0.7), 0, 0, 0, 0)
        #         time.sleep(0.1)
        #     self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
        #
        #     while abs(self.drone_basic.tag_target_pitch) > 0.1:
        #         self.drone_basic.tag_position(self.drone_basic.tag_info, 12)
        #         if self.drone_basic.tag_info == 0:
        #             rospy.logwarn("No tag detections.")
        #             self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
        #             return
        #         self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, - self.drone_basic.tag_target_pitch)
        #         time.sleep(0.1)
        #     self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
        #
        #     while abs(self.drone_basic.tag_target_x) > 0.01:
        #         self.drone_basic.tag_position(self.drone_basic.tag_info, 12)
        #         if self.drone_basic.tag_info == 0:
        #             rospy.logwarn("No tag detections.")
        #             self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
        #             return
        #         self.dog_basic.qilin_cmd_vel(0, 2 * -(self.drone_basic.tag_target_x - 0.05), 0, 0, 0)
        #         print(f'I got it!')
        #         time.sleep(0.1)
        #     self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
        #     self.drone_basic.tag_position(self.drone_basic.tag_info, 12)
        #     target_x = self.drone_basic.drone_x + self.drone_basic.tag_target_x
        #     target_y = self.drone_basic.drone_y - self.drone_basic.tag_target_z + self.desk_dimension[1] / 2
        #     self.put_position = [target_x, target_y, self.desk_dimension[2] + 0.15 ]
        #     userdata.put_position = self.put_position
        if self.rm == 1:
            rospy.sleep(0.1)
            self.drone_basic.add_module_trigger()

            self.drone_basic.tag_position(self.drone_basic.tag_info, 12)
            # while abs(self.drone_basic.tag_target_z - 1.20) > 0.1 or abs(self.drone_basic.tag_target_x ) > 0.1 or abs(self.drone_basic.tag_target_pitch) > 0.05:
            #     self.drone_basic.tag_position(self.drone_basic.tag_info, 12)
            #     # if self.drone_basic.tag_info == 0:
            #     #     rospy.logwarn("No tag detections.")
            #     #     self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
            #     #     return
            #     self.dog_basic.qilin_cmd_vel(0.2 * (self.drone_basic.tag_target_z - 1.2), - 0.5 * self.drone_basic.tag_target_x, 0, 0, - 0.5* self.drone_basic.tag_target_pitch)
            #     time.sleep(0.1)
            #     if (0.2 * (self.drone_basic.tag_target_z - 1.20) < 0.05) and ((-0.5 * self.drone_basic.tag_target_x) < 0.05) and ((- 0.5* self.drone_basic.tag_target_pitch) < 0.01):
            #         break
            self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
            rospy.loginfo("Tag z finish.")
            self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)

            rospy.sleep(3)
            # self.dog_basic.sit()
            rospy.sleep(2)
            self.drone_basic.tag_position(self.drone_basic.tag_info, 12)
            target_x = self.drone_basic.drone_x - self.drone_basic.tag_target_x + (self.basic.mocap_desk_x - self.basic.mocap_tag12_x)
            target_y = self.drone_basic.drone_y - (self.drone_basic.tag_target_z + 0.15) + (self.basic.mocap_desk_y - self.basic.mocap_tag12_y) - 0.05798
            target_z = self.drone_basic.drone_z - (self.drone_basic.tag_target_y - 0.15) + (self.basic.mocap_desk_z - self.basic.mocap_tag12_z) + 0.20 - 0.080 + 0.3
            target_yaw = self.drone_basic.drone_yaw - self.drone_basic.tag_target_pitch + 1.57 + (self.basic.mocap_desk_yaw - self.basic.mocap_tag12_yaw)
            target_mocap_x = self.basic.mocap_xuanwu_x - self.drone_basic.tag_target_x + (
                        self.basic.mocap_desk_x - self.basic.mocap_tag12_x)
            target_mocap_y = self.basic.mocap_xuanwu_y - (self.drone_basic.tag_target_z + 0.15) + (
                        self.basic.mocap_desk_y - self.basic.mocap_tag12_y) - 0.05798
            target_mocap_z = self.basic.mocap_xuanwu_z - (self.drone_basic.tag_target_y - 0.15) + (
                        self.basic.mocap_desk_z - self.basic.mocap_tag12_z) + 0.15 + 0.080
            put_position_mocap = [target_mocap_x, target_mocap_y, target_mocap_z]
            self.put_position = [target_x, target_y, target_z, target_yaw]
            userdata.put_position = self.put_position
            rospy.loginfo(f'put_position: {self.put_position}')
            rospy.loginfo(f'put_position_mocap: {put_position_mocap}')
            rospy.loginfo(f'desk_mocap: {self.basic.mocap_desk_x}, {self.basic.mocap_desk_y}, {self.basic.mocap_desk_z}')

        else:
            while abs(self.TargetSearch.drone_basic.drone_yaw - 1.57) > (3.14/12):
                self.TargetSearch.sim_dog_yaw = -10
                self.TargetSearch.sim_basic.sim_robot_twist('go1_gazebo', 0, 0, 0, 0, 0, self.TargetSearch.sim_dog_yaw)
                time.sleep(0.5)
                # if self.TargetSearch.sim_target_x != 0:
                #     print(f'I got 12!')
                #     break
            while self.TargetSearch.drone_basic.drone_y > -0.2:
                self.TargetSearch.sim_basic.sim_robot_twist('go1_gazebo', 0.5, 0, 0, 0, 0, 0)
                time.sleep(0.1)

            # while abs(self.TargetSearch.drone_basic.drone_yaw + 3.14) > (3.14/24):
            while True:
                self.TargetSearch.sim_dog_yaw = 10
                self.TargetSearch.sim_basic.sim_robot_twist('go1_gazebo', 0, 0, 0, 0, 0, self.TargetSearch.sim_dog_yaw)
                self.TargetSearch.sim_apriltag_position(self.TargetSearch.msg_apriltag, 12)
                if self.TargetSearch.sim_target_x != 0:
                    break
                time.sleep(0.5)

            while abs(self.TargetSearch.sim_target_z) > 0.8:
                self.TargetSearch.sim_apriltag_position(self.TargetSearch.msg_apriltag, 12)
                self.TargetSearch.sim_basic.sim_robot_twist('go1_gazebo', self.TargetSearch.sim_target_z, 0, 0, 0, 0, 0)
                time.sleep(0.5)
            while abs(self.TargetSearch.drone_basic_sim.sim_drone_yaw - self.TargetSearch.sim_target_roll) > (3.14/12):
                self.TargetSearch.sim_apriltag_position(self.TargetSearch.msg_apriltag, 12)
                self.TargetSearch.sim_basic.sim_robot_twist('go1_gazebo', 0, 0, 0, 0, 0, abs(self.TargetSearch.drone_basic_sim.sim_drone_yaw - self.TargetSearch.sim_target_roll))
                time.sleep(0.5)
            self.TargetSearch.sim_apriltag_position(self.TargetSearch.msg_apriltag, 12)
            self.put_position = [self.drone_basic.drone_x+self.TargetSearch.sim_target_z+ 0.45, self.drone_basic.drone_y - self.TargetSearch.sim_target_x, self.drone_basic.drone_z + 0.6]
            print(f'put_x:{self.put_position[0]} put_y:{self.put_position[1]} put_z:{self.put_position[2]}')
            userdata.put_position = self.put_position
        return 'succeeded'

class Takeoff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['rm', 'takeoff_position'], output_keys=['takeoff_position'])

        self.drone_basic = DroneBasic()
        # self.sim_links_attachment = AttachlinksNode()
        self.takeoff_position = [0.0, 0.0, 0.0, 0.0]

    def execute(self, userdata):
        self.rm = userdata.rm
        if self.rm ==0:
            self.drone_basic.record_takeoff_position(self.drone_basic.drone_x, self.drone_basic.drone_y, self.drone_basic.drone_z, self.drone_basic.drone_yaw)
            time.sleep(0.1)
            self.drone_basic.drone_start()
            time.sleep(0.1)
            self.drone_basic.drone_takeoff()
            self.sim_links_attachment.sim_detach_links("xuanwu", "root", "go1_gazebo", "base")
            userdata.takeoff_position = [self.drone_basic.takeoff_x, self.drone_basic.takeoff_y, self.drone_basic.takeoff_z, self.drone_basic.takeoff_yaw]
        if self.rm == 1:
            self.drone_basic.record_takeoff_position(self.drone_basic.drone_x, self.drone_basic.drone_y, self.drone_basic.drone_z, self.drone_basic.drone_yaw)
            time.sleep(0.1)
            # self.drone_basic.drone_start()
            # time.sleep(0.1)
            # self.drone_basic.drone_takeoff()
            rospy.loginfo(f'takeoff!!!!!!!!!!')
            userdata.takeoff_position = [self.drone_basic.takeoff_x, self.drone_basic.takeoff_y, self.drone_basic.takeoff_z, self.drone_basic.takeoff_yaw]
            print(f'takeoff_x:{self.drone_basic.takeoff_x}, takeoff_y:{self.drone_basic.takeoff_y}, takeoff_z:{self.drone_basic.takeoff_z}, takeoff_yaw:{self.drone_basic.takeoff_yaw}')
            # while self.drone_basic.drone_state != 5:
            #     time.sleep(0.1)
            rospy.sleep(5)
        return 'succeeded'

class FlyTarget(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['put_position', 'rm'])
        self.drone_basic = DroneBasic()
        self.dog_basic = DogBasic()
        self.gripper_move = GripperMoveNode()
        self.put_position = [0.0, 0.0, 0.0]
        self.rm = 0

    def execute(self, userdata):
        self.put_position = userdata.put_position
        self.rm = userdata.rm

        if self.rm == 1:
            qx, qy,qz, qw = tft.quaternion_from_euler(0,0, self.put_position[3])
            self.drone_basic.drone_target('world', self.put_position[0], self.put_position[1], 1.0, 0 , 0, 0, 1)
            rospy.loginfo(f'send fly target')
            rospy.sleep(3)
            self.drone_basic.drone_target('world', self.put_position[0], self.put_position[1], self.put_position[2], qx , qy, qz, qw)
            rospy.loginfo(f'{self.put_position[0]}, {self.put_position[1]}, {self.put_position[2]}, {qx}, {qy}, {qz}, {qw}')
            rospy.sleep(3)
            # self.gripper_move.return_qilin_trigger()
            # self.drone_basic.remove_module_trigger()

        else:
            while self.drone_basic.drone_state != 5:
                time.sleep(0.1)
            self.drone_basic.drone_target('world',self.put_position[0], self.put_position[1], self.put_position[2], 0, 0, 0.707107, -0.707107)

        return 'succeeded'
class Correction(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['put_position', 'rm'])
        rospy.Subscriber('/ground_robot/tag_detections', AprilTagDetectionArray, self._callback_simulation_apriltag)
        self.drone_basic = DroneBasic()
        self.dog_basic = DogBasic()
        self.gripper_move = GripperMoveNode()

        # self.sim_basic = SimbasicNode()

        self.rm = 0
        self.sim_desk_x, self.sim_desk_y, self.sim_desk_z = 0.0, 0.0, 0.0
        self.sim_desk_roll, self.sim_desk_pitch, self.sim_desk_yaw= 0.0, 0.0, 0.0
        self.sim_target_x, self.sim_target_y, self.sim_target_z = 0.0, 0.0, 0.0
        self.sim_target_roll, self.sim_target_pitch, self.sim_target_yaw= 0.0, 0.0, 0.0
        self.tag_desk_x, self.tag_desk_y, self.tag_desk_z = 0.0, 0.0, 0.0
        self.tag_desk_roll, self.tag_desk_pitch, self.tag_desk_yaw = 0.0, 0.0, 0.0
        self.tag_drone_x, self.tag_drone_y, self.tag_drone_z = 0.0, 0.0, 0.0
        self.tag_drone_roll, self.tag_drone_pitch, self.tag_drone_yaw = 0.0, 0.0, 0.0
        self.msg_apriltag = None
        self.user_input = None

    def _callback_simulation_apriltag(self, msg):
        self.msg_apriltag = msg

    def sim_apriltag_position(self,data,target_id):
        for det in data.detections:
            # print(f'{det}')
            # print(f'{det.id}')
            if target_id in det.id:
                pose = det.pose.pose.pose
                self.sim_target_x = pose.position.x
                self.sim_target_y = pose.position.y
                self.sim_target_z = pose.position.z
                qx = pose.orientation.x
                qy = pose.orientation.y
                qz = pose.orientation.z
                qw = pose.orientation.w
                self.sim_target_roll = \
                tft.euler_from_quaternion([qx, qy, qz, qw])[0]
                self.sim_target_pitch = \
                tft.euler_from_quaternion([qx, qy, qz, qw])[1]
                self.sim_target_yaw = \
                tft.euler_from_quaternion([qx, qy, qz, qw])[2]

        print(f'{self.sim_target_x}, {self.sim_target_y}, {self.sim_target_z}')

    def execute(self, userdata):

        self.rm= userdata.rm

        if self.rm != 1:
            self.put_position = userdata.put_position

            print(f'rosrun unitree_controller unitree_servo')
            print(f'input y')
            self.user_input = input()
            if self.user_input == 'y':
                pass
            while self.sim_desk_x == 0:
                self.sim_apriltag_position(self.msg_apriltag, 12)
                self.sim_desk_x, self.sim_desk_y, self.sim_desk_z = self.sim_target_x, self.sim_target_y, self.sim_target_z
                self.sim_desk_roll, self.sim_desk_pitch, self.sim_desk_yaw = self.sim_target_roll, self.sim_target_pitch, self.sim_target_yaw


            while (self.sim_desk_z+ 0.15 - self.tag_drone_z) > 0.03 or (self.sim_desk_x - self.tag_drone_x)> 0.03:
                self.sim_apriltag_position(self.msg_apriltag, 2)
                self.tag_drone_x, self.tag_drone_y, self.tag_drone_z = self.sim_target_x, self.sim_target_y, self.sim_target_z
                self.tag_drone_roll, self.tag_drone_pitch, self.tag_drone_yaw = self.sim_target_roll, self.sim_target_pitch, self.sim_target_yaw
                self.drone_basic.drone_target(self.sim_desk_z+0.15 - self.tag_drone_z, self.sim_desk_x - self.tag_drone_x, self.sim_desk_y + 0.3,
                                              self.put_position[2], 0, 0, 0.707107, -0.707107)
        if self.rm == 1:
            self.user_input = input()
            rospy.loginfo(f'input y')
            if self.user_input == 'y':
                pass
            self.dog_basic.stand()
            rospy.sleep(3)
            self.dog_basic.qilin_body_pose(0, 0.130526, 0, 0.991445)
            rospy.sleep(3)
            self.dog_basic.qilin_body_pose(0, 0.2588, 0, 0.9659)
            rospy.sleep(3)
            self.drone_basic.tag_position(self.dog_basic.tag_dog_info, 13)
            while self.drone_basic.tag_target_x == 0 and self.drone_basic.tag_target_y == 0:
                rospy.loginfo(f'No tag 13 info')
                rospy.sleep(1)
                continue
            self.drone_basic.tag_position(self.dog_basic.tag_dog_info, 3)
            while self.drone_basic.tag_target_x == 0 and self.drone_basic.tag_target_y == 0:
                rospy.loginfo(f'No tag 3 info')
                rospy.sleep(1)
                continue

            while abs(self.dog_basic.correction_err_x) > 0.03 or abs(self.dog_basic.correction_err_y) > 0.03 or abs(self.dog_basic.correction_err_z) > 0.03 or abs(self.dog_basic.correction_err_yaw) > 0.05:
                rospy.loginfo(f'{self.dog_basic.correction_err_x},{self.dog_basic.correction_err_y},{self.dog_basic.correction_err_z}')
                if abs(self.dog_basic.correction_err_x) > 0.3 or abs(self.dog_basic.correction_err_y) > 0.3:
                    continue
                qx, qy, qz, qw = tft.quaternion_from_euler(0, 0, self.dog_basic.correction_err_yaw + self.dog_basic.correction_err_yaw)
                self.drone_basic.drone_target('world', self.drone_basic.drone_x + self.dog_basic.correction_err_x, self.drone_basic.drone_y + self.dog_basic.correction_err_y, self.drone_basic.drone_z + self.dog_basic.correction_err_z, qx, qy, qz, qw)
                if abs(self.dog_basic.correction_err_x) < 0.03 and abs(self.dog_basic.correction_err_y) < 0.03 and abs(self.dog_basic.correction_err_z) < 0.03 and abs(self.dog_basic.correction_err_yaw) < 0.05:
                    self.gripper_move.return_qilin_trigger()
                    self.drone_basic.remove_module_trigger()
                    self.dog_basic.qilin_body_pose(0, 0, 0, 1)
                    break
                rospy.sleep(2)
        return 'succeeded'
class FlyBack(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],input_keys=['takeoff_position', 'rm'])
        # self.pub_drone_nav_info = rospy.Publisher('/quadrotor/uav/nav/info', FlightNav, queue_size=10)
        # self.pub_drone_nav_trigger = rospy.Publisher('/quadrotor/uav/nav/trigger', Empty, queue_size=10)
        # self.pub_drone_nav = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)

        self.pub_drone_target_info = rospy.Publisher('/quadrotor/target_pose/info', PoseStamped, queue_size=10)
        self.pub_drone_target_trigger = rospy.Publisher('/quadrotor/target_pose/trigger', Empty, queue_size=10)
        self.pub_drone_target_sim = rospy.Publisher('/quadrotor/target_pose', PoseStamped, queue_size=10)

        self.takeoff_x, self.takeoff_y, self.takeoff_z, self.takeoff_yaw = 0.0, 0.0, 0.0, 0.0
        self.rm= 0
        self.land_offset = 0.4
    # def drone_nav_trigger(self):
    #     time.sleep(0.5)
    #     rospy.loginfo("Publishing nav trigger command...")
    #     empty_msg = Empty()
    #     self.pub_drone_nav_trigger.publish(empty_msg)
    #
    # def drone_nav_info(self, x_y_mode, x, y, z_mode, z, yaw_mode, omega_z, yaw):
    #     flight_nav_msg = FlightNav()
    #     flight_nav_msg.header.stamp = rospy.Time.now()
    #     flight_nav_msg.header.frame_id = 'world'
    #
    #     flight_nav_msg.control_frame = 0
    #     flight_nav_msg.target = 0
    #     flight_nav_msg.pos_xy_nav_mode = x_y_mode
    #     flight_nav_msg.target_pos_x = x
    #     flight_nav_msg.target_vel_x = 0.0
    #     flight_nav_msg.target_acc_x = 0.0
    #     flight_nav_msg.target_pos_y = y
    #     flight_nav_msg.target_vel_y = 0.0
    #     flight_nav_msg.target_acc_y = 0.0
    #     flight_nav_msg.yaw_nav_mode = yaw_mode
    #     flight_nav_msg.target_omega_z = omega_z
    #     flight_nav_msg.target_yaw = yaw
    #     flight_nav_msg.pos_z_nav_mode = z_mode
    #     flight_nav_msg.target_pos_z = z
    #     flight_nav_msg.target_vel_z = 0.0
    #     flight_nav_msg.target_pos_diff_z = 0.0
    #
    #     self.pub_drone_nav.publish(flight_nav_msg)

        # self.pub_drone_nav_info.publish(flight_nav_msg)
        # rospy.sleep(0.5)
        # self.drone_nav_trigger()

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

        self.pub_drone_target_info.publish(drone_target_pose)
        rospy.sleep(1.5)
        self.drone_target_pose_trigger()

    def drone_target_pose_trigger(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing nav trigger command...")
        empty_msg = Empty()
        self.pub_drone_target_trigger.publish(empty_msg)

    def drone_target_pose_sim(self, x, y, z, ox, oy, oz, ow):
        drone_target_pose = PoseStamped()
        drone_target_pose.header.frame_id = 'world'
        drone_target_pose.pose.position.x = x
        drone_target_pose.pose.position.y = y
        drone_target_pose.pose.position.z = z
        drone_target_pose.pose.orientation.x = ox
        drone_target_pose.pose.orientation.y = oy
        drone_target_pose.pose.orientation.z = oz
        drone_target_pose.pose.orientation.w = ow

        self.pub_drone_target_sim.publish(drone_target_pose)

    def execute(self, userdata):
        self.takeoff_x = userdata.takeoff_position[0]
        self.takeoff_y = userdata.takeoff_position[1]
        self.takeoff_z = userdata.takeoff_position[2]
        self.rm = userdata.rm
        print(f'This is the fly back position: {self.takeoff_x}, {self.takeoff_y}, {self.takeoff_z}')
        time.sleep(1)
        if self.rm ==1:
            self.drone_target_pose( self.takeoff_x, self.takeoff_y, self.takeoff_z + self.land_offset, 0, 0, 1,0)
            time.sleep(10)
            # self.drone_target_pose(self.takeoff_x, self.takeoff_y, self.takeoff_z + self.land_offset, 0, 0, 1, 0)
            # time.sleep(5)
        else:
            self.drone_target_pose_sim( self.takeoff_x, self.takeoff_y,  self.takeoff_z + self.land_offset, 0, 0, 0,1)
        return 'succeeded'
class VisibilityAdjustment(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['takeoff_position','rm'])

        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self._callback_dog_position)

        # self.pub_drone_nav = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)
        # self.pub_drone_nav_info = rospy.Publisher('/quadrotor/uav/nav/info', FlightNav, queue_size=10)
        # self.pub_drone_nav_trigger = rospy.Publisher('/quadrotor/uav/nav/trigger', Empty, queue_size=10)

        self.pub_drone_target_info = rospy.Publisher('/quadrotor/target_pose/info', PoseStamped, queue_size=10)
        self.pub_drone_target_trigger = rospy.Publisher('/quadrotor/target_pose/trigger', Empty, queue_size=10)
        self.pub_drone_target_sim = rospy.Publisher('/quadrotor/target_pose', PoseStamped, queue_size=10)

        self.pub_sim_pose = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        self.pub_qilin_vel = rospy.Publisher('/go1/cmd_vel', Twist, queue_size=10)

        self.takeoff_x, self.takeoff_y, self.takeoff_z, self.takeoff_yaw = 0.0, 0.0, 0.0, 0.0
        self.april_drone_x, self.april_drone_y, self.april_drone_z, self.april_drone_yaw = 0.0, 0.0, 0.0, 0.0
        self.april_valve_x, self.april_valve_y, self.april_valve_z, self.april_valve_yaw = 0.0, 0.0, 0.0, 0.0
        self.dog_x, self.dog_y, self.dog_z, self.dog_yaw = 0.0, 0.0, 0.0, 0.0
        self.lx, self.ly = 0.0, 0.0
        self.land_offset = 0.4

        self.beginfollow = 0
        self.find_drone_tag = 0
        self.timer_detector_maker = 0
        self.rm = 0
        self.miss_first_time, self.miss_second_time = rospy.Time.now(), rospy.Time.now()


    def _callback_apriltag(self, data):
        current_time = rospy.Time.now()
        # print(f'apriltag:{current_time.to_sec()}')

        # get the apriltag`s position information compare with camera coordination
        if data.detections:
            self.find_valve_tag = self.find_target_tag(data.detections, 1)
            self.find_drone_tag = self.find_target_tag(data.detections, 0)
            if self.find_drone_tag == 1:
                if self.beginfollow == 1:
                    if self.rm == 1:
                        self.align_dog_with_drone()
                    else:
                        self.align_dog_with_drone_sim()


        else:
            self.find_valve_tag = 0
            self.find_drone_tag = 0
            if self.beginfollow == 1:
                self.miss_second_time = rospy.Time.now()
                duration = self.miss_second_time - self.miss_first_time
                # print(f'{duration.to_sec()}')

                if duration.to_sec() > 0.5:
                    self.qilin_cmd_vel(0, 0, 0, 0, 0)
                    self.miss_first_time = self.miss_second_time

    def _callback_dog_position(self, data):
        self.dog_x= data.pose[2].position.x
        self.dog_y= data.pose[2].position.y
        self.dog_z= data.pose[2].position.z

    def find_target_tag(self,data,target_id):
        b = len(data)
        a = 0

        while a < b:
            c = target_id in data[a].id
            # print(f'{a}')
            if c:
                if target_id == 0:
                    self.april_drone_x = -data[a].pose.pose.pose.position.y
                    self.april_drone_y = data[a].pose.pose.pose.position.x
                    self.april_drone_z = data[a].pose.pose.pose.position.z
                    self.april_drone_yaw = tft.euler_from_quaternion([data[a].pose.pose.pose.orientation.x,
                                                                          data[a].pose.pose.pose.orientation.y,
                                                                          data[a].pose.pose.pose.orientation.z,
                                                                          data[a].pose.pose.pose.orientation.w])[2]

                if target_id == 1:
                    self.april_valve_x = -data[a].pose.pose.pose.position.y
                    self.april_valve_y = data[a].pose.pose.pose.position.x
                    self.april_valve_z = data[a].pose.pose.pose.position.z
                    self.april_valve_yaw = tft.euler_from_quaternion([data[a].pose.pose.pose.orientation.x,
                                                         data[a].pose.pose.pose.orientation.y,
                                                         data[a].pose.pose.pose.orientation.z,
                                                         data[a].pose.pose.pose.orientation.w])[2]

                return 1
            else:
                return 0


    def sim_pose(self, px, py, ox, oy, oz, ow):

        sim_pose = ModelState()
        sim_pose.model_name = 'unitree'
        sim_pose.pose.position.x = px
        sim_pose.pose.position.y = py
        sim_pose.pose.orientation.x = ox
        sim_pose.pose.orientation.y = oy
        sim_pose.pose.orientation.z = oz
        sim_pose.pose.orientation.w = ow
        sim_pose.reference_frame = 'world'
        self.pub_sim_pose.publish(sim_pose)

    def qilin_cmd_vel(self, lx, ly, ax, ay, az):
        qilin_cmd_vel = Twist()
        qilin_cmd_vel.linear.x = lx
        qilin_cmd_vel.linear.y = ly
        qilin_cmd_vel.angular.x = ax
        qilin_cmd_vel.angular.y = ay
        qilin_cmd_vel.angular.z = az

        self.pub_qilin_vel.publish(qilin_cmd_vel)

    def align_dog_with_drone(self):
        self.lx = 1.2 * self.april_drone_x
        self.ly = 1.2 * self.april_drone_y
        self.april_z = 0.03 *self.april_drone_yaw
        if self.april_z > 0.3:
            self.april_z = 0.3
        if self.april_z < -0.3:
            self.april_z = -0.3
        # print("align speed lx, ly, :", self.lx, self.ly, self.april_z)
        # print(f'apriltag_time:{apriltag_time.to_sec()}')
        if abs(self.lx) < 2 and abs(self.ly) < 2:
            navigation_time = rospy.Time.now()
            # print(f'navigation_time:{navigation_time.to_sec()}')
            self.qilin_cmd_vel(self.lx, self.ly, 0, 0, self.april_z)

    def align_dog_with_drone_sim(self):
        if self.april_drone_x >0.2 and self.april_drone_y >0.2:
            print(f'This is the dog:{self.dog_x},{self.dog_y},april_drone:{self.april_drone_x},{self.april_drone_y}')
            self.lx = self.april_drone_x + self.dog_x
            self.ly = self.april_drone_y + self.dog_y
            self.sim_pose(self.lx, self.ly, 0, 0, 0, 1)
            print(f"This is the align:{self.lx},{self.ly}")


    def timer_detector(self,time,detec_num,result_num):
        begin = rospy.Time.now()
        begin = begin.to_sec()
        finish = rospy.Time.now()
        finish = finish.to_sec()
        print(f'{begin},{finish}')
        self.timer_detector_marker = result_num
        r = rospy.Rate(detec_num)
        while finish - begin < time:

            if self.find_drone_tag == 1:
                self.timer_detector_marker -= 1
                print(f'{self.timer_detector_marker}')
            r.sleep()
            finish = rospy.Time.now()
            finish = finish.to_sec()

        if self.timer_detector_marker <= 0:
            return 1
        else:
            return 0

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

        self.pub_drone_target_info.publish(drone_target_pose)
        rospy.sleep(1.5)
        self.drone_target_pose_trigger()

    def drone_target_pose_trigger(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing nav trigger command...")
        empty_msg = Empty()
        self.pub_drone_target_trigger.publish(empty_msg)

    def drone_target_pose_sim(self, x, y, z, ox, oy, oz, ow):
        drone_target_pose = PoseStamped()
        drone_target_pose.header.frame_id = 'world'
        drone_target_pose.pose.position.x = x
        drone_target_pose.pose.position.y = y
        drone_target_pose.pose.position.z = z
        drone_target_pose.pose.orientation.x = ox
        drone_target_pose.pose.orientation.y = oy
        drone_target_pose.pose.orientation.z = oz
        drone_target_pose.pose.orientation.w = ow

        self.pub_drone_target_sim.publish(drone_target_pose)


    # def drone_nav_trigger(self):
    #     time.sleep(0.5)
    #     rospy.loginfo("Publishing nav trigger command...")
    #     empty_msg = Empty()
    #     self.pub_drone_nav_trigger.publish(empty_msg)
    #
    # def drone_nav_info(self, x_y_mode, x, y, z_mode, z, yaw_mode, omega_z, yaw):
    #     flight_nav_msg = FlightNav()
    #     flight_nav_msg.header.stamp = rospy.Time.now()
    #     flight_nav_msg.header.frame_id = 'world'
    #
    #     flight_nav_msg.control_frame = 0
    #     flight_nav_msg.target = 0
    #     flight_nav_msg.pos_xy_nav_mode = x_y_mode
    #     flight_nav_msg.target_pos_x = x
    #     flight_nav_msg.target_vel_x = 0.0
    #     flight_nav_msg.target_acc_x = 0.0
    #     flight_nav_msg.target_pos_y = y
    #     flight_nav_msg.target_vel_y = 0.0
    #     flight_nav_msg.target_acc_y = 0.0
    #     flight_nav_msg.yaw_nav_mode = yaw_mode
    #     flight_nav_msg.target_omega_z = omega_z
    #     flight_nav_msg.target_yaw = yaw
    #     flight_nav_msg.pos_z_nav_mode = z_mode
    #     flight_nav_msg.target_pos_z = z
    #     flight_nav_msg.target_vel_z = 0.0
    #     flight_nav_msg.target_pos_diff_z = 0.0
    #
    #     # self.pub_drone_nav_info.publish(flight_nav_msg)
    #     # rospy.sleep(0.5)
    #     # self.drone_nav_trigger()
    #
    #     self.pub_drone_nav.publish(flight_nav_msg)


    def scanning(self,x,y,z,c):
        if self.rm == 1:
            self.drone_target_pose(x + c, y, z, 0, 0, 0, 0)
            print(f'1')
            time.sleep(3.5)
            if self.timer_detector(3, 10,5):
                print(f'find, return')
                return
            self.drone_target_pose(x, y + c, z, 0, 0, 0, 0)
            print(f'2')
            time.sleep(3.5)
            if self.timer_detector(3, 10,5):
                print(f'find, return')
                return
            self.drone_target_pose( x - c, y, z, 0, 0, 0,0)
            print(f'3')
            time.sleep(3.5)
            if self.timer_detector(3, 10,5):
                print(f'find, return')
                return
            self.drone_target_pose( x, y - c, z, 0, 0, 0, 0)
            print(f'4')
            time.sleep(3.5)
            if self.timer_detector(3, 10,5):
                print(f'find, return')
                return
        else:
            self.drone_target_pose_sim(x + c, y, z, 0, 0, 0, 1)
            print(f'1')
            time.sleep(1.5)
            if self.timer_detector(2, 3):
                print(f'find, return')
                return
            self.drone_target_pose_sim(x, y + c, z, 0, 0, 0, 1)
            print(f'2')
            time.sleep(1.5)
            if self.timer_detector(2, 3):
                print(f'find, return')
                return
            self.drone_target_pose_sim(x - c, y, z, 0, 0, 0, 1)
            print(f'3')
            time.sleep(1.5)
            if self.timer_detector(2, 3):
                print(f'find, return')
                return
            self.drone_target_pose_sim(x, y - c, z, 0, 0, 0, 1)
            print(f'4')
            time.sleep(1.5)
            if self.timer_detector(2, 3):
                print(f'find, return')
                return


    def execute(self, userdata):
        print('begin follow')

        self.takeoff_x = userdata.takeoff_position[0]
        self.takeoff_y = userdata.takeoff_position[1]
        self.takeoff_z = userdata.takeoff_position[2]
        self.rm = userdata.rm
        self.beginfollow = 1
        while not rospy.is_shutdown():
            # self.scanning(self.takeoff_x, self.takeoff_y, self.takeoff_z + self.land_offset, 0.2)
            self.scanning(self.takeoff_x, self.takeoff_y, self.takeoff_z + self.land_offset , 0.2)
            if self.timer_detector_marker <= 0:
                break
            # self.scanning(self.takeoff_x, self.takeoff_y, self.takeoff_z + self.land_offset, 0.3)
            self.scanning(self.takeoff_x, self.takeoff_y, self.takeoff_z + self.land_offset, 0.3)

            if self.timer_detector_marker <= 0:
                break
        time.sleep(5)
        return 'succeeded'

class AlignAndLand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'], input_keys=['rm'])

        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self._callback_dog_position)

        self.pub_land = rospy.Publisher('/quadrotor/teleop_command/land', Empty, queue_size=10)
        self.pub_qilin_vel = rospy.Publisher('/go1/cmd_vel', Twist, queue_size=10)
        self.pub_sim_pose = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

        self.miss_first_time, self.miss_second_time = rospy.Time.now(), rospy.Time.now()
        self.find_valve_tag = 0
        self.find_drone_tag = 0
        self.beginfollow = 0
        self.detection = 0.0
        self.flag = 0
        self.rm= 0
        self.timer_detector_marker = 5

        self.lx, self.ly, self.lz = 0, 0, 0
        self.april_x, self.april_y, self.april_z = 0.0, 0.0, 0.0
        self.april_drone_x, self.april_drone_y, self.april_drone_z, self.april_drone_yaw = 0.0, 0.0, 0.0, 0.0
        self.april_valve_x, self.april_valve_y, self.april_valve_z, self.april_valve_yaw = 0.0, 0.0, 0.0, 0.0
        self.dog_x, self.dog_y, self.dog_z, self.dog_yaw = 0.0, 0.0, 0.0, 0.0

    def _callback_apriltag(self, data):
        current_time = rospy.Time.now()
        # print(f'apriltag:{current_time.to_sec()}')

        # get the apriltag`s position information compare with camera coordination
        if data.detections:
            self.find_valve_tag = self.find_target_tag(data.detections,1)
            self.find_drone_tag = self.find_target_tag(data.detections, 0)
            if self.find_drone_tag == 1:
                if  self.beginfollow == 1:
                    if self.rm == 1:
                        self.align_dog_with_drone()
                    else:
                        self.align_dog_with_drone_sim()

        else:
            self.find_valve_tag = 0
            self.find_drone_tag = 0
            if self.beginfollow == 1:
                self.miss_second_time = rospy.Time.now()
                duration = self.miss_second_time - self.miss_first_time
                # print(f'{duration.to_sec()}')

                if duration.to_sec() > 0.5:
                    self.qilin_cmd_vel(0, 0, 0, 0, 0)
                    self.miss_first_time = self.miss_second_time

    def _callback_dog_position(self, data):
        self.dog_x= data.pose[2].position.x
        self.dog_y= data.pose[2].position.y
        self.dog_z= data.pose[2].position.z

    def sim_pose(self, px, py, ox, oy, oz, ow):

        sim_pose = ModelState()
        sim_pose.model_name = 'unitree'
        sim_pose.pose.position.x = px
        sim_pose.pose.position.y = py
        sim_pose.pose.orientation.x = ox
        sim_pose.pose.orientation.y = oy
        sim_pose.pose.orientation.z = oz
        sim_pose.pose.orientation.w = ow
        sim_pose.reference_frame = 'world'
        self.pub_sim_pose.publish(sim_pose)

    def find_target_tag(self,data,target_id):
        b = len(data)
        a = 0

        while a < b:
            c = target_id in data[a].id
            # print(f'{a}')
            if c:
                if target_id == 0:
                    self.april_drone_x = -data[a].pose.pose.pose.position.y
                    self.april_drone_y = data[a].pose.pose.pose.position.x
                    self.april_drone_z = data[a].pose.pose.pose.position.z
                    self.april_drone_yaw = tft.euler_from_quaternion([data[a].pose.pose.pose.orientation.x,
                                                                          data[a].pose.pose.pose.orientation.y,
                                                                          data[a].pose.pose.pose.orientation.z,
                                                                          data[a].pose.pose.pose.orientation.w])[2]

                if target_id == 1:
                    self.april_valve_x = -data[a].pose.pose.pose.position.y
                    self.april_valve_y = data[a].pose.pose.pose.position.x
                    self.april_valve_z = data[a].pose.pose.pose.position.z
                    self.april_valve_yaw = tft.euler_from_quaternion([data[a].pose.pose.pose.orientation.x,
                                                         data[a].pose.pose.pose.orientation.y,
                                                         data[a].pose.pose.pose.orientation.z,
                                                         data[a].pose.pose.pose.orientation.w])[2]

                return 1
            else:
                return 0

    def align_dog_with_drone(self):
        self.lx = 1.3 * self.april_drone_x
        self.ly = 1.3 * self.april_drone_y
        self.april_z = 0.03 *self.april_drone_yaw
        if self.april_z > 0.3:
            self.april_z = 0.3
        if self.april_z < -0.3:
            self.april_z = -0.3
        # print("align speed lx, ly, :", self.lx, self.ly, self.april_z)
        # print(f'apriltag_time:{apriltag_time.to_sec()}')
        if abs(self.lx) < 2 and abs(self.ly) < 2:
            navigation_time = rospy.Time.now()
            # print(f'navigation_time:{navigation_time.to_sec()}')
            self.qilin_cmd_vel(self.lx, self.ly, 0, 0, self.april_z)

    def align_dog_with_drone_sim(self):
        print(f"This is the align:{self.april_drone_x},{self.april_drone_y}")
        if abs(self.april_drone_x) >0.1 or abs(self.april_drone_y) >0.1:
            print(f'This is the dog:{self.dog_x},{self.dog_y},april_drone:{self.april_drone_x},{self.april_drone_y}')
            self.lx = self.april_drone_x + self.dog_x
            self.ly = self.april_drone_y + self.dog_y
            self.sim_pose(self.lx, self.ly, 0, 0, 0, 1)

    def land(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing land command...")
        empty_msg = Empty()
        self.pub_land.publish(empty_msg)

    def drone_landing_detection(self, i):
        r = rospy.Rate(i)
        number = i
        check = i
        while not rospy.is_shutdown():
            number = number - 1
            if math.sqrt(self.april_drone_x ** 2 + self.april_drone_y ** 2) < 0.03 and abs(self.april_drone_yaw) < 0.5:
            # if math.sqrt(self.april_drone_x ** 2 + self.april_drone_y ** 2) < 0.5:
                check = check - 1
            if number == 0:
                break
            r.sleep()
        self.flag = check

    def drone_landing_condition(self):
        while not rospy.is_shutdown():
            if self.april_drone_x != 0:
                break
            time.sleep(0.1)

        self.flag = 0
        self.drone_landing_detection(5)
        print(f'plus = {self.flag}')
        if self.flag == 0:
            self.beginfollow = 0
            # self.qilin_cmd_vel(0, 0, 0, 0, 0)
            self.land()
            print(f'landon')

    def qilin_cmd_vel(self, lx, ly, ax, ay, az):

        qilin_cmd_vel = Twist()
        qilin_cmd_vel.linear.x = lx
        qilin_cmd_vel.linear.y = ly
        qilin_cmd_vel.angular.x = ax
        qilin_cmd_vel.angular.y = ay
        qilin_cmd_vel.angular.z = az

        self.pub_qilin_vel.publish(qilin_cmd_vel)

    def execute(self, userdata):
        self.rm = userdata.rm
        self.beginfollow = 1
        current_time = rospy.Time.now()
        a = current_time.to_sec()
        b = current_time.to_sec()
        while b - a < 5:
            self.drone_landing_condition()
            if self.beginfollow== 0:
                return 'succeeded'
            current_time = rospy.Time.now()
            b = current_time.to_sec()
        return 'failed'
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        return 'succeeded'
class Finish(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted'])

    def execute(self, userdata):
        return 'preempted'

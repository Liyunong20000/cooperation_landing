#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import numpy as np
import time
import math
from aerial_robot_msgs.msg import FlightNav
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Empty, UInt8
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
import tf.transformations as tft

from drone_basic_function import DroneBasic
from dog_basic_function import DogBasic
# It is for  the Coopration for xuanwu and Qilin

class AprillandqilinNode:

    def __init__(self):
        print(f'Hi, I am Cloud Cube')
        rospy.init_node('Aprillandqilin', anonymous=True)

        self.lx, self.ly, self.lz = 0.0, 0.0, 0.0
        self.ryaw = 0.0
        self.target_x, self.target_y, self.target_z = 0.0, 0.0, 0.0
        self.target_qx, self.target_qy, self.target_qz, self.target_qw = 0.0, 0.0, 0.0, 0.0
        self.target_roll, self.target_pitch, self.target_yaw = 0.0, 0.0, 0.0

        self.takeoff_x, self.takeoff_y, self.takeoff_z = 0.0, 0.0, 0.0

        self.miss_first_time, self.miss_second_time = rospy.Time.now(), rospy.Time.now()

        self.state = 0
        self.beginland = 0
        self.beginfollow = 0
        self.flag = 0

        self.drone_basic = DroneBasic()
        self.dog_basic = DogBasic()

        # Subscribe and publish.
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)
        # rospy.Subscriber('/uavandgr/event', UInt8, self._callback_event)

        self.pub_event = rospy.Publisher('/uavandgr/event', UInt8, queue_size=10)

        self.pub_qilin_vel = rospy.Publisher('/go1/cmd_vel', Twist, queue_size=10)
        rospy.wait_for_service('/go1/sit')
        rospy.wait_for_service('/go1/stand')
        self.service_client_sit = rospy.ServiceProxy('/go1/sit', Trigger)
        self.service_client_stand = rospy.ServiceProxy('/go1/stand', Trigger)


        rospy.set_param('/landing_threshold', 0.04)
        self.landing_threshold = rospy.get_param("/landing_threshold")

        rospy.set_param('/movement_constant', 2)
        self.move_cons = rospy.get_param("/movement_constant")
        rospy.set_param('/rotation_constant', 0.05)
        self.rotate_cons = rospy.get_param("/rotation_constant")

    def _callback_apriltag(self, data):
        current_time = rospy.Time.now()
        # print(f'apriltag:{current_time.to_sec()}')

        # get the apriltag`s position information compare with camera coordination
        if data.detections:
            self.find_drone_tag = self.find_target_tag(data.detections, 0)
            if self.find_drone_tag == 1:
                if self.beginfollow == 1:
                    self.align_dog_with_drone()
            #rospy.loginfo("latest arigtarg timestamp: {}".format(data.header.stamp.to_sec()))

        else:
            self.find_drone_tag = 0
            if self.beginfollow == 1:
                self.miss_second_time = rospy.Time.now()
                duration = self.miss_second_time - self.miss_first_time
                # print(f'{duration.to_sec()}')
                if duration.to_sec() > 0.5:
                    self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
                    self.miss_first_time = rospy.Time.now()

    def event(self, x):
        event_msgs = UInt8()
        event_msgs.data = x
        self.pub_event.publish(event_msgs)


    def find_target_tag(self,data,target_id):
        b = len(data)
        a = 0

        while a < b:
            c = target_id in data[a].id
            # print(f'{a}')
            if c:
                if target_id == 0:
                    self.target_x = - data[a].pose.pose.pose.position.y
                    self.target_y = data[a].pose.pose.pose.position.x
                    self.target_qx = data[a].pose.pose.pose.orientation.x
                    self.target_qy = data[a].pose.pose.pose.orientation.y
                    self.target_qz = data[a].pose.pose.pose.orientation.z
                    self.target_qw = data[a].pose.pose.pose.orientation.w
                    self.target_roll = \
                        tft.euler_from_quaternion([self.target_qx, self.target_qy, self.target_qz, self.target_qw])[0]
                    self.target_pitch = \
                        tft.euler_from_quaternion([self.target_qx, self.target_qy, self.target_qz, self.target_qw])[1]
                    self.target_yaw = \
                        tft.euler_from_quaternion([self.target_qx, self.target_qy, self.target_qz, self.target_qw])[2]

                return 1

            else:
                return 0

    def align_dog_with_drone(self):
        self.lx = np.clip((self.move_cons * self.target_x), -2, 2)
        self.ly = np.clip((self.move_cons * self.target_y), -2, 2)
        self.ryaw = np.clip((self.rotate_cons * self.target_yaw), -0.3, 0.3)

        self.dog_basic.qilin_cmd_vel(self.lx, self.ly, 0, 0, self.ryaw)

    def landing_determination(self):
        a = 5
        while not rospy.is_shutdown():
            if self.find_drone_tag == 1:
                if math.sqrt(self.target_x ** 2 + self.target_y ** 2) < self.landing_interval and abs(
                    self.target_yaw) < 0.5:
                        a -= 1
            else:
                a = 5
            if a == 0:
                self.drone_basic.drone_land()
            time.sleep(0.1)

    def come_back(self):

        while not rospy.is_shutdown():
            if self.drone_basic.drone_state == 5:
                break
            time.sleep(0.1)
        # self.event(2)
        print(f'Move to above takeoff_Z')
        time.sleep(3)
        self.beginfollow = 1
        print(f'begin follow')

if __name__ == '__main__':
    node = AprillandqilinNode()
    node.dog_basic.stand()
    time.sleep(3)
    node.event(1)
    node.come_back()

    while not rospy.is_shutdown():
        rospy.spin()

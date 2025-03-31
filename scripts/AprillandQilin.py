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
import tf.transformations as tft

# It is for  the Coopration for Mini_Quadrotor and Qilin

# use the class to create a node


class AprillandqilinNode:

    def __init__(self):  # This part will work when this node is used.
        print(f'Hi, I am Cloud Cube')
        rospy.init_node('Aprillandqilin', anonymous=True)

        self.lx, self.ly, self.lz = 0, 0, 0
        self.target_x, self.target_y, self.target_z = 0.0, 0.0, 0.0
        self.target_qx, self.target_qy, self.target_qz, self.target_qw = 0.0, 0.0, 0.0, 0.0

        self.takeoff_x, self.takeoff_y, self.takeoff_z = 0.0, 0.0, 0.0

        self.D = 0
        self.time_rece = rospy.Time()
        self._seq = 0
        self.state = 0
        self.beginland = 0
        self.beginfollow = 0
        self.flag = 0

        # Subscribe and publish.
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)
        rospy.Subscriber('/quadrotor/flight_state', UInt8, self._callback_state)
        # rospy.Subscriber('/uavandgr/event', UInt8, self._callback_event)

        self.pub_event = rospy.Publisher('/uavandgr/event', UInt8, queue_size=10)

        self.pub_qilin_vel = rospy.Publisher('/go1/cmd_vel', Twist, queue_size=10)
        self.pub_qilin_pose = rospy.Publisher('/go1/body_pose', Pose, queue_size=10)
        rospy.wait_for_service('/go1/sit')
        rospy.wait_for_service('/go1/stand')
        self.service_client_sit = rospy.ServiceProxy('/go1/sit', Trigger)
        self.service_client_stand = rospy.ServiceProxy('/go1/stand', Trigger)

        rospy.set_param('/converge_interval', 0.05)
        self.converge_interval = rospy.get_param("/converge_interval")
        rospy.set_param('/above_z', 0.4)
        self.above_z = rospy.get_param("/above_z")

        rospy.set_param('/move_parameter', 2)
        self.move_param = rospy.get_param("/move_parameter")
        rospy.set_param('/rotate_parameter', 0.05)
        self.rotate_param = rospy.get_param("/rotate_parameter")

    def _callback_apriltag(self, data):
        current_time = rospy.Time.now()
        # print(f'apriltag:{current_time.to_sec()}')

        # get the apriltag`s position information compare with camera coordination
        if data.detections:
            #rospy.loginfo("latest arigtarg timestamp: {}".format(data.header.stamp.to_sec()))
            target = data.detections[0]
            self.target_x = target.pose.pose.pose.position.x
            self.target_y = target.pose.pose.pose.position.y
            self.target_qx = target.pose.pose.pose.orientation.x
            self.target_qy = target.pose.pose.pose.orientation.y
            self.target_qz = target.pose.pose.pose.orientation.z
            self.target_qw = target.pose.pose.pose.orientation.w
            self.target_roll = \
            tft.euler_from_quaternion([self.target_qx, self.target_qy, self.target_qz, self.target_qw])[0]
            self.target_pitch = \
            tft.euler_from_quaternion([self.target_qx, self.target_qy, self.target_qz, self.target_qw])[1]
            self.target_yaw = \
            tft.euler_from_quaternion([self.target_qx, self.target_qy, self.target_qz, self.target_qw])[2]
            if self.beginfollow == 1:
                self.lx = - self.move_param * self.target_y
                self.ly = self.move_param * self.target_x
                self.ryaw = self.rotate_param * self.target_yaw
                # self.qx = self.pose_parameter * self.april_qx
                # self.qy = self.pose_parameter * self.april_qy
                # self.qz = self.pose_parameter * self.april_qz
                # self.qw = self.pose_parameter * self.april_qw
                apriltag_time = rospy.Time.now()
                print("euler_z (degree):", self.lx, self.ly)
                # print(f'apriltag_time:{apriltag_time.to_sec()}')
                if abs(self.lx) < 5 and abs(self.ly) < 5:
                    navigation_time = rospy.Time.now()
                    print("enter")
                    # print(f'navigation_time:{navigation_time.to_sec()}')
                    self.qilin_cmd_vel(self.lx, self.ly, 0, 0, self.ryaw)
                    
        else:
            if self.beginfollow == 1:
                self.qilin_cmd_vel(0, 0, 0, 0, 0)

            # self.qilin_body_pose(0, 0, 0, 1)

    def _callback_position(self, odom_msg):
        self.drone_x = odom_msg.pose.pose.position.x
        self.drone_y = odom_msg.pose.pose.position.y
        self.drone_z = odom_msg.pose.pose.position.z

    def _callback_state(self, msg):
        self.state = msg.data

    def event(self, x):
        event_msgs = UInt8()
        event_msgs.data = x
        self.pub_event.publish(event_msgs)

    def drone_landing_detection(self, i):
        r = rospy.Rate(i)
        number = i
        while not rospy.is_shutdown():
            number = number - 1
            if math.sqrt(self.april_x ** 2 + self.april_y ** 2) < 0.03 and abs(self.april_z) < 10:
                i = i - 1
            if number == 0:
                break
            r.sleep()
        self.flag = i

    def drone_landing_condition(self):
        while not rospy.is_shutdown():
            i = 1
            plus = 0
            self.flag = 0

            while i > 0:
                i = i - 1
                self.drone_landing_detection(10)
                plus = plus + self.flag
                print(f'plus = {plus}')
            if plus == 0:
                self.beginfollow = 0
                self.qilin_cmd_vel(0, 0, 0, 0, 0)
                self.event(3)
                print(f'landon')

                break

    def sit(self):
        try:
            response = self.service_client_sit()
            if response.success:
                rospy.loginfo('Sit command executed successfully')
            else:
                rospy.logwarn('Stand command failed: %s', response.message)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)

    def stand(self):
        try:
            response = self.service_client_stand()
            if response.success:
                rospy.loginfo('Stand command executed successfully')
            else:
                rospy.logwarn('Stand command failed: %s', response.message)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)

    def qilin_cmd_vel(self, lx, ly, ax, ay, az):
        qilin_cmd_vel = Twist()
        qilin_cmd_vel.linear.x = lx
        qilin_cmd_vel.linear.y = ly
        qilin_cmd_vel.angular.x = ax
        qilin_cmd_vel.angular.y = ay
        qilin_cmd_vel.angular.z = az

        self.pub_qilin_vel.publish(qilin_cmd_vel)


    def come_back(self):

        while not rospy.is_shutdown():
            if self.state == 5:
                break
            time.sleep(0.1)
        # self.event(2)
        print(f'Move to above takeoff_Z')
        # self.converge(self.takeoff_x, self.takeoff_y, tz)
        time.sleep(3)
        self.beginfollow = 1
        print(f'begin follow')

if __name__ == '__main__':
    node = AprillandqilinNode()
    node.stand()
    time.sleep(3)
    node.event(1)
    node.come_back()
    node.drone_landing_condition()
    while not rospy.is_shutdown():
        rospy.spin()

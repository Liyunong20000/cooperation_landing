#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import numpy as np
import time
import math
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Empty, UInt8
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CameraInfo


# It is for  the Coopration for Mini_Quadrotor and Qilin

# use the class to create a node

class AprilmoveqilinNode:

    def __init__(self):  # This part will work when this node is used.
        print(f'Hi, I am Cloud Cube')
        rospy.init_node('Aprilmoveqilin', anonymous=True)

        self.lx, self.ly, self.lz = 0, 0, 0
        self.ax, self.ay, self.az = 0, 0, 0
        self.april_x, self.april_y, self.april_z = 0.0, 0.0, 0.0
        self.april_qx,self.april_qy, self.april_qz, self.april_qw = 0.0, 0.0, 0.0, 0.0
        self.time_rece = rospy.Time()
        self._seq = 0
        # Subscribe and publish.
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)

        self.pub_qilin_vel= rospy.Publisher('/go1/cmd_vel', Twist, queue_size=10)
        self.pub_qilin_pose = rospy.Publisher('/go1/body_pose', Pose, queue_size=10)
        rospy.wait_for_service('/go1/sit')
        rospy.wait_for_service('/go1/stand')
        self.service_client_sit = rospy.ServiceProxy('/go1/sit', Trigger)
        self.service_client_stand = rospy.ServiceProxy('/go1/stand', Trigger)
        rospy.set_param('/move_parameter', 0.8)
        self.move_parameter = rospy.get_param("/move_parameter")

    def _callback_apriltag(self, data):
        current_time = rospy.Time.now()
        # print(f'apriltag:{current_time.to_sec()}')

        # get the apriltag`s position information compare with camera coordination
        if data.detections:
            #rospy.loginfo("latest arigtarg timestamp: {}".format(data.header.stamp.to_sec()))
            a = data.detections[0]
            self.april_x = a.pose.pose.pose.position.x
            self.april_y = a.pose.pose.pose.position.y
            self.april_qx = a.pose.pose.pose.orientation.x
            self.april_qy = a.pose.pose.pose.orientation.y
            self.april_qz = a.pose.pose.pose.orientation.z
            self.april_qw = a.pose.pose.pose.orientation.w

            self.lx = 10 * self.april_x
            self.ly = 10 * self.april_y
            apriltag_time = rospy.Time.now()
            print("euler_z (degree):", self.lx, self.ly)
            #print(f'apriltag_time:{apriltag_time.to_sec()}')
            if abs(self.lx) < 5 and abs(self.ly) < 5:
                navigation_time = rospy.Time.now()
                print("enter")
                #print(f'navigation_time:{navigation_time.to_sec()}')
                self.qilin_cmd_vel(self.lx, self.ly, 0, 0, 0)
                # self.qilin_body_pose(self.april_qx, self.april_qy, self.april_qz, self.april_qw)
        else:
            self.qilin_cmd_vel(0, 0, 0, 0, 0)

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

    def qilin_body_pose(self, qx, qy, qz, qw):
        qilin_body_pose = Pose()
        qilin_body_pose.orientation.x = qx
        qilin_body_pose.orientation.y = qy
        qilin_body_pose.orientation.z = qz
        qilin_body_pose.orientation.w = qw

        self.pub_qilin_pose.publish(qilin_body_pose)

    # def quaternion_to_euler_angle(self, x, y, z, w):
    #
    #     R = np.array([[1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y],
    #                   [2 * x * y + 2 * w * z, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * w * x],
    #                   [2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x ** 2 - 2 * y ** 2]])
    #     # theta_x = math.degrees(np.arctan2(R[2, 1], R[2, 2]))
    #     # theta_y = math.degrees(np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2)))
    #     theta_z = math.degrees(np.arctan2(R[1, 0], R[0, 0]))
    #     return theta_z

if __name__ == '__main__':
    node = AprilmoveqilinNode()
    node.stand()
    time.sleep(1)
    # node.qilin_cmd_vel(0.05,0.05, 0, 0, 0)
    # time.sleep(3)
    # node.qilin_cmd_vel(-0.05, -0.05, 0, 0, 0)
    # time.sleep(3)
    # node.qilin_cmd_vel(0,0,0,0,0)
    # rospy.sleep(1)
    # node.sit()
    while not rospy.is_shutdown():
        rospy.spin()

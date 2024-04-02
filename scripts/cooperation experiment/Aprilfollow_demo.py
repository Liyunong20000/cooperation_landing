#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import numpy as np
from apriltag_ros.msg import AprilTagDetectionArray
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo
from dynamic_reconfigure.server import Server
# It is for Apirltag following demo for MyAGV

# use the class to create a node


class AprilfollowNode:

    def __init__(self):  # This part will work when this node is used.
        print(f'Hi, I am Cloud Cube')
        rospy.init_node('Aprilfollow', anonymous=True)

        self.agv_x, self.agv_y, self.agv_z = 0.0, 0.0, 0.0
        self.lvol_x, self.lvol_y, self.lvol_z = 0.0, 0.0, 0.0
        self.anglevol_x, self.anglevol_y, self.anglevol_z = 0.0, 0.0, 0.0
        self.april_x, self.april_y, self.april_z = 0.0, 0.0, 0.0
        self._seq = 0
        self.D = 1

        self.time_rece = rospy.Time()

        # Subscribe and publish.
        # rospy.Subscriber('/odom', Odometry, self._callback_position)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)
        rospy.Subscriber('/land_camera_upward/camera_info', CameraInfo, self._callback_camera_raw)
        # rospy.Subscriber('/usb_cam/camera_info', CameraInfo, self._callback_camera_raw)
        self.pub_nav = rospy.Publisher('/cmd_vel', Twist, queue_size=10, tcp_nodelay=True)
        # transport_hint='tcpNoDelay'
        # self.tf_buffer = tf2_ros.Buffer()
        # Initialize a TransformListener
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
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
            quaternion_x = a.pose.pose.pose.orientation.x
            quaternion_y = a.pose.pose.pose.orientation.y
            quaternion_z = a.pose.pose.pose.orientation.z
            quaternion_w = a.pose.pose.pose.orientation.w

            self.april_z = self.quaternion_to_euler_angle(quaternion_x,quaternion_y,quaternion_z,quaternion_w)
            print("euler_z (degree):", self.april_z)
            self.D = 1
            self.lvol_x = self.move_parameter * self.april_y
            self.lvol_y = - self.move_parameter * self.april_x
            self.anglevol_z = 0.05* self.move_parameter * self.april_z
            apriltag_time = rospy.Time.now()

            #print(f'apriltag_time:{apriltag_time.to_sec()}')
            if abs(self.lvol_x) < 0.5 and abs(self.lvol_y) < 0.5:
                navigation_time = rospy.Time.now()

                #print(f'navigation_time:{navigation_time.to_sec()}')
                self.agv_nav_info(self.lvol_x, self.lvol_y, self.anglevol_z)
        else:
            self.D = 0
            self.agv_nav_info(0, 0, 0)

        # orientation = detection.pose.pose.pose.orientation

    def _callback_camera_raw(self, data):
        current_time = rospy.Time.now()

        # print(f'camera_info:{current_time.to_sec()}')
        # rospy.loginfo("------------------------------------camera!")
    def agv_nav_info(self, lx, ly, az):
        agv_nav_msg = Twist()
        agv_nav_msg.linear.x = lx
        agv_nav_msg.linear.y = ly
        agv_nav_msg.angular.z = az

        self.pub_nav.publish(agv_nav_msg)

        #rospy.loginfo("pub vel cmd!")

    def dddprint(self):
        while not rospy.is_shutdown():
            rate = rospy.Rate(1)

            print(f'x = {self.april_x},y = {self.april_y}')
            rate.sleep()

    import numpy as np

    def quaternion_to_euler_angle(self, x, y, z, w):

        R = np.array([[1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y],
                      [2 * x * y + 2 * w * z, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * w * x],
                      [2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x ** 2 - 2 * y ** 2]])
        # theta_x = math.degrees(np.arctan2(R[2, 1], R[2, 2]))
        # theta_y = math.degrees(np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2)))
        theta_z = math.degrees(np.arctan2(R[1, 0], R[0, 0]))
        return theta_z

    # 示例四元数



if __name__ == '__main__':
    node = AprilfollowNode()
    # node.follow_camera(10)
    # node.dddprint()
    # while not rospy.is_shutdown():
    rospy.spin()

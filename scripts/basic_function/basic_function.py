#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, PoseStamped
import tf.transformations as tft

class BasicNode:

    def __init__(self):  # This part will work when this node is used.

        # Subscribe and publish.
        rospy.Subscriber('/qilin/mocap_node/brick/mocap/pose', PoseStamped, self._callback_brick_mocap)
        rospy.Subscriber('/qilin/mocap_node/desk/mocap/pose', PoseStamped, self._callback_desk_mocap)
        rospy.Subscriber('/qilin/mocap_node/tag12/mocap/pose', PoseStamped, self._callback_tag12_mocap)
        rospy.Subscriber('/qilin/mocap_node/tag13/mocap/pose', PoseStamped, self._callback_tag13_mocap)
        rospy.Subscriber('/xuanwu/mocap/pose', PoseStamped, self._callback_xuanwu_mocap)

        self.mocap_brick_x, self.mocap_brick_y, self.mocap_brick_z = 0.0, 0.0, 0.0
        self.mocap_brick_qx, self.mocap_brick_qy, self.mocap_brick_qz, self.mocap_brick_qw = 0.0, 0.0, 0.0, 0.0
        self.mocap_brick_roll, self.mocap_brick_pitch, self.mocap_brick_yaw = 0.0, 0.0, 0.0

        self.mocap_desk_x, self.mocap_desk_y, self.mocap_desk_z = 0.0, 0.0, 0.0
        self.mocap_desk_qx, self.mocap_desk_qy, self.mocap_desk_qz, self.mocap_desk_qw = 0.0, 0.0, 0.0, 0.0
        self.mocap_desk_roll, self.mocap_desk_pitch, self.mocap_desk_yaw = 0.0, 0.0, 0.0

        self.mocap_tag12_x, self.mocap_tag12_y, self.mocap_tag12_z = 0.0, 0.0, 0.0
        self.mocap_tag12_qx, self.mocap_tag12_qy, self.mocap_tag12_qz, self.mocap_tag12_qw = 0.0, 0.0, 0.0, 0.0
        self.mocap_tag12_roll, self.mocap_tag12_pitch, self.mocap_tag12_yaw = 0.0, 0.0, 0.0

        self.mocap_tag13_x, self.mocap_tag13_y, self.mocap_tag13_z = 0.0, 0.0, 0.0
        self.mocap_tag13_qx, self.mocap_tag13_qy, self.mocap_tag13_qz, self.mocap_tag13_qw = 0.0, 0.0, 0.0, 0.0
        self.mocap_tag13_roll, self.mocap_tag13_pitch, self.mocap_tag13_yaw = 0.0, 0.0, 0.0

        self.mocap_xuanwu_x, self.mocap_xuanwu_y, self.mocap_xuanwu_z = 0.0, 0.0, 0.0
        self.mocap_xuanwu_qx, self.mocap_xuanwu_qy, self.mocap_xuanwu_qz, self.mocap_xuanwu_qw = 0.0, 0.0, 0.0, 0.0
        self.mocap_xuanwu_roll, self.mocap_xuanwu_pitch, self.mocap_xuanwu_yaw = 0.0, 0.0, 0.0

    def _callback_brick_mocap(self, msg):
        pose = msg.pose
        self.mocap_brick_x = pose.position.x
        self.mocap_brick_y = pose.position.y
        self.mocap_brick_z = pose.position.z
        self.mocap_brick_qx = pose.orientation.x
        self.mocap_brick_qy = pose.orientation.y
        self.mocap_brick_qz = pose.orientation.z
        self.mocap_brick_qw = pose.orientation.w
        self.mocap_brick_roll, self.mocap_brick_pitch, self.mocap_brick_yaw = tft.euler_from_quaternion(
            [self.mocap_brick_qx, self.mocap_brick_qy, self.mocap_brick_qz, self.mocap_brick_qw]
        )
        # print(f'mocap_brick_x={self.mocap_brick_x}', f'mocap_brick_y={self.mocap_brick_y}', f'mocap_brick_z={self.mocap_brick_z}')
        # rospy.loginfo(f'mocap_brick_roll: {self.mocap_brick_roll}, mocap_brick_pitch: {self.mocap_brick_pitch}, mocap_brick_yaw: {self.mocap_brick_yaw}')
    def _callback_desk_mocap(self, msg):
        pose = msg.pose
        self.mocap_desk_x = pose.position.x
        self.mocap_desk_y = pose.position.y
        self.mocap_desk_z = pose.position.z
        self.mocap_desk_qx = pose.orientation.x
        self.mocap_desk_qy = pose.orientation.y
        self.mocap_desk_qz = pose.orientation.z
        self.mocap_desk_qw = pose.orientation.w
        self.mocap_desk_roll, self.mocap_desk_pitch, self.mocap_desk_yaw = tft.euler_from_quaternion(
            [self.mocap_desk_qx, self.mocap_desk_qy, self.mocap_desk_qz, self.mocap_desk_qw]
        )
        # print(f'mocap_desk_x={self.mocap_desk_x}', f'mocap_desk_y={self.mocap_desk_y}', f'mocap_brick_z={self.mocap_desk_z}')

    def _callback_tag12_mocap(self, msg):
        pose = msg.pose
        self.mocap_tag12_x = pose.position.x
        self.mocap_tag12_y = pose.position.y
        self.mocap_tag12_z = pose.position.z
        self.mocap_tag12_qx = pose.orientation.x
        self.mocap_tag12_qy = pose.orientation.y
        self.mocap_tag12_qz = pose.orientation.z
        self.mocap_tag12_qw = pose.orientation.w
        self.mocap_tag12_roll, self.mocap_tag12_pitch, self.mocap_tag12_yaw = tft.euler_from_quaternion(
            [self.mocap_tag12_qx, self.mocap_tag12_qy, self.mocap_tag12_qz, self.mocap_tag12_qw]
        )

    def _callback_tag13_mocap(self, msg):
        pose = msg.pose
        self.mocap_tag13_x = pose.position.x
        self.mocap_tag13_y = pose.position.y
        self.mocap_tag13_z = pose.position.z
        self.mocap_tag13_qx = pose.orientation.x
        self.mocap_tag13_qy = pose.orientation.y
        self.mocap_tag13_qz = pose.orientation.z
        self.mocap_tag13_qw = pose.orientation.w
        self.mocap_tag13_roll, self.mocap_tag13_pitch, self.mocap_tag13_yaw = tft.euler_from_quaternion(
            [self.mocap_tag13_qx, self.mocap_tag13_qy, self.mocap_tag13_qz, self.mocap_tag13_qw]
        )

    def _callback_xuanwu_mocap(self, msg):
        pose = msg.pose
        self.mocap_xuanwu_x = pose.position.x
        self.mocap_xuanwu_y = pose.position.y
        self.mocap_xuanwu_z = pose.position.z
        self.mocap_xuanwu_qx = pose.orientation.x
        self.mocap_xuanwu_qy = pose.orientation.y
        self.mocap_xuanwu_qz = pose.orientation.z
        self.mocap_xuanwu_qw = pose.orientation.w
        self.mocap_xuanwu_roll, self.mocap_xuanwu_pitch, self.mocap_xuanwu_yaw = tft.euler_from_quaternion(
            [self.mocap_xuanwu_qx, self.mocap_xuanwu_qy, self.mocap_xuanwu_qz, self.mocap_xuanwu_qw]
        )
if __name__ == '__main__':
    rospy.init_node('Basic', anonymous=True)
    rospy.sleep(0.1)
    node = BasicNode()
    rospy.spin()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import tf.transformations as tft
from apriltag_ros.msg import AprilTagDetectionArray


class TestNode:
    def __init__(self):
        rospy.logdebug('Initializing AprilTag debug node.')

        self.drone_x, self.drone_y, self.drone_z = 0.0, 0.0, 0.0
        self.drone_qx, self.drone_qy, self.drone_qz, self.drone_qw = 0.0, 0.0, 0.0, 1.0
        self.drone_roll, self.drone_pitch, self.drone_yaw = 0.0, 0.0, 0.0

        self.mocap_drone_x, self.mocap_drone_y, self.mocap_drone_z = 0.0, 0.0, 0.0
        self.mocap_camera_x, self.mocap_camera_y, self.mocap_camera_z = 0.0, 0.0, 0.0

        self.center_x, self.center_y, self.center_z = 0.0, 0.0, 0.0
        # rospy.Subscriber('/xuanwu/tag_detections', AprilTagDetectionArray, self._callback_tag_info)
        rospy.Subscriber(
            rospy.get_param('~tag_topic', '/xuanwu/tag_detections'),
            AprilTagDetectionArray,
            self._callback_tag_info,
            queue_size=1,
        )

    def _callback_tag_info(self, msg):
        if not msg.detections:
            rospy.logwarn('No tag detections.')
            return

        pose = msg.detections[0].pose.pose.pose  # 取第一个Tag
        self.drone_x = pose.position.x
        self.drone_y = pose.position.y
        self.drone_z = pose.position.z

        self.drone_qx = pose.orientation.x
        self.drone_qy = pose.orientation.y
        self.drone_qz = pose.orientation.z
        self.drone_qw = pose.orientation.w

        roll, pitch, yaw = tft.euler_from_quaternion(
            [self.drone_qx, self.drone_qy, self.drone_qz, self.drone_qw]
        )
        self.drone_roll, self.drone_pitch, self.drone_yaw = roll, pitch, yaw
        T = (
            np.array([[0, -1, 0, -0.13], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            @ np.array(
                [
                    [1, 0, 0, self.drone_x],
                    [0, 1, 0, self.drone_y],
                    [0, 0, 1, self.drone_z],
                    [0, 0, 0, 1],
                ]
            )
            @ np.array([[1, 0, 0, 0], [0, 1, 0, -0.05], [0, 0, 1, 0], [0, 0, 0, 1]])
            @ np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        )
        self.center_x, self.center_y, self.center_z = T[0, 3], T[1, 3], T[2, 3]
        # rospy.loginfo(f'Position: x=type({self.center_x}), y={self.center_y}, z={self.center_z}')
        # print(type(self.center_x))
        # rospy.loginfo(f'Position: x={self.drone_x:.3f}, y={self.drone_y:.3f}, z={self.drone_z:.3f}')
        rospy.loginfo('Orientation (rpy): roll=%.3f pitch=%.3f yaw=%.3f', roll, pitch, yaw)


if __name__ == '__main__':
    rospy.init_node('apriltag_debug')
    node = TestNode()
    rospy.spin()

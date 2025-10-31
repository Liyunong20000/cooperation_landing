#! /usr/bin/env python
# -*- coding: utf-8 -*-
import tf.transformations as tft
import rospy, sys
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np
class TestNode:

    def __init__(self):
        print('Hi, I am Cloud Cube')
        rospy.init_node('Test', anonymous=True)

        self.drone_x, self.drone_y, self.drone_z = 0.0, 0.0, 0.0
        self.drone_qx, self.drone_qy, self.drone_qz, self.drone_qw = 0.0, 0.0, 0.0, 0.0
        self.drone_roll, self.drone_pitch, self.drone_yaw = 0.0, 0.0, 0.0

        self.mocap_drone_x, self.mocap_drone_y, self.mocap_drone_z = 0.0, 0.0, 0.0
        self.mocap_camera_x, self.mocap_camera_y, self.mocap_camera_z = 0.0, 0.0, 0.0

        self.center_x, self.center_y, self.center_z = 0.0, 0.0, 0.0
        # rospy.Subscriber('/xuanwu/tag_detections', AprilTagDetectionArray, self._callback_tag_info)
        rospy.Subscriber('/xuanwu/tag_detections', AprilTagDetectionArray, self._callback_tag_info)

    def _callback_tag_info(self, msg):
        if not msg.detections:
            rospy.logwarn("No tag detections.")
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
        T = np.array([[0, -1, 0, -0.13],
                 [1, 0, 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]]) @ np.array([[1,0,0,self.drone_x],
                                            [0,1,0,self.drone_y],
                                            [0,0,1,self.drone_z],
                                            [0,0,0,1]]) @ np.array([[1, 0, 0, 0],
                                                              [0, 1, 0, -0.05],
                                                              [0, 0, 1, 0],
                                                              [0, 0, 0, 1]]) @ np.array([[1, 0, 0, 0],
                                                                                         [0, 1, 0, 0],
                                                                                         [0, 0, 1, 0],
                                                                                         [0, 0, 0, 1]])
        self.center_x, self.center_y, self.center_z = T[0,3], T[1,3], T[2,3]
        # rospy.loginfo(f'Position: x=type({self.center_x}), y={self.center_y}, z={self.center_z}')
        # print(type(self.center_x))
        # rospy.loginfo(f'Position: x={self.drone_x:.3f}, y={self.drone_y:.3f}, z={self.drone_z:.3f}')
        rospy.loginfo(f'Orientation (rpy): roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}')
        rospy.sleep(0.05)
if __name__ == '__main__':
    node = TestNode()
    rospy.spin()


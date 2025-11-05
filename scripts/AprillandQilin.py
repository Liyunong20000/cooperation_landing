#! /usr/bin/env python
# -*- coding: utf-8 -*-
from AprilmoveQilin import *
from drone_basic_function import DroneBasic
from dog_basic_function import DogBasic
from std_msgs.msg import Empty, UInt8
import rospy, time
import numpy as np
import tf.transformations as tft

# It is for  the Coopration for xuanwu and Qilin

class AprillandqilinNode:

    def __init__(self):
        print(f'Hi, I am Cloud Cube')
        rospy.init_node('Aprillandqilin', anonymous=True)

        self.takeoff_x, self.takeoff_y, self.takeoff_z = 0.0, 0.0, 0.0
        self.alignment_counter = 0
        self.required_frames = 10
        self.aligned = False
        self.pause_when_lost = rospy.Duration(1)
        self.drone_pose_matrix = []
        self.success_start_time = None

        self.drone_basic = DroneBasic()
        self.dog_basic = DogBasic()
        self.aqm = AprilmoveqilinNode()
        # Subscribe and publish.
        # rospy.Subscriber('/uavandgr/event', UInt8, self._callback_event)


    def is_alignment_success(self, T):
        if not isinstance(T, np.ndarray) or T.shape != (4, 4):
            return
        pos = T[:3, 3]
        x, y, _ = pos
        dist = np.linalg.norm([x, y])
        if not isinstance(self.aqm.find_target_tag(self.aqm.msg_apriltag, 0), np.ndarray) or self.aqm.find_target_tag(self.aqm.msg_apriltag, 0).shape != (4, 4):
            rospy.logwarn("Tag 0 not found or invalid transform.")
            return
        q = tft.quaternion_from_matrix(self.aqm.find_target_tag(self.aqm.msg_apriltag, 0))
        roll, pitch, yaw = tft.euler_from_quaternion(q)

        # q = tft.quaternion_from_matrix(T)
        # _, _, yaw = tft.euler_from_quaternion(q)
        return (dist < 0.03) and (abs(yaw) < 0.05)

    def check_and_land(self):
        self.aqm.align_dog_with_drone()
        # this already handles tag loss and stops dog if needed

        T = self.aqm.dog_align_drone_matrix

        # Make sure we have a valid transform
        if T is not None and self.is_alignment_success(T):
            self.alignment_counter += 1
            rospy.loginfo_throttle(1.0, f"Alignment frame count: {self.alignment_counter}")

            if self.alignment_counter >= self.required_frames and not self.aligned:
                rospy.loginfo("Alignment held for sufficient frames. Landing drone.")
                print('I will land')
                self.drone_basic.drone_land()
                self.aligned = True
        else:
            self.alignment_counter = 0
            self.aligned = False

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.check_and_land()
            rate.sleep()
if __name__ == '__main__':
    try:
        node = AprillandqilinNode()
        node.dog_basic.stand()
        rospy.sleep(0.5)
        node.drone_basic.record_takeoff_position(node.drone_basic.drone_x, node.drone_basic.drone_y, node.drone_basic.drone_z, node.drone_basic.drone_yaw)
        time.sleep(1)
        node.drone_basic.drone_start()
        node.drone_basic.drone_takeoff()
        time.sleep(3)
        node.drone_basic.drone_target('world', -2, 1, 1.0, 0, 0, 0, 1)
        time.sleep(3)
        node.drone_basic.drone_target('world', node.drone_basic.takeoff_x, node.drone_basic.takeoff_y, 1.0, 0, 0, 0, 1)
        time.sleep(3)
        node.drone_basic.drone_target('world', node.drone_basic.takeoff_x, node.drone_basic.takeoff_y, node.drone_basic.takeoff_z + 0.2, 0, 0, 0, 1)
        node.run()
    except rospy.ROSInterruptException:
        pass


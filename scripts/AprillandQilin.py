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
    """
    Node for cooperative landing between aerial drone and ground robot using AprilTags.
    """

    def __init__(self):
        print(f'Hi, I am Cloud Cube')

        self.takeoff_x, self.takeoff_y, self.takeoff_z = 0.0, 0.0, 0.0
        self.alignment_counter = 0
        self.aligned = False
        self.pause_when_lost = rospy.Duration(1)
        self.drone_pose_matrix = []
        self.success_start_time = None

        # Load parameters
        self.required_frames = rospy.get_param("~required_frames", 10)
        self.distance_threshold = rospy.get_param("~distance_threshold", 0.03)
        self.angle_threshold = rospy.get_param("~angle_threshold", 0.05)
        self.demo_target_x = rospy.get_param("~demo_target_x", -2)
        self.demo_target_y = rospy.get_param("~demo_target_y", 1)
        self.demo_target_z = rospy.get_param("~demo_target_z", 1.0)
        self.demo_hover_z = rospy.get_param("~demo_hover_z", 1.0)
        self.demo_final_z_offset = rospy.get_param("~demo_final_z_offset", 0.1)

        self.drone_basic = DroneBasic()
        self.dog_basic = DogBasic()
        self.aqm = AprilmoveqilinNode()
        # Subscribe and publish.
        # rospy.Subscriber('/uavandgr/event', UInt8, self._callback_event)


    def is_alignment_success(self, T):
        """
        Check if alignment is successful based on distance and angle thresholds.
        """
        if not isinstance(T, np.ndarray) or T.shape != (4, 4):
            return False
        pos = T[:3, 3]
        x, y, _ = pos
        dist = np.linalg.norm([x, y])
        # Check the tag data
        T_tag_0 = self.aqm.find_target_tag(self.aqm.msg_apriltag, 0)
        if not isinstance(T_tag_0, np.ndarray) or T_tag_0.shape != (4, 4):
            rospy.logwarn("Tag 0 not found or invalid transform.")
            return False
        # Get the quaternion data from tag and transfer them into roll, pitch, yaw euler angle
        q = tft.quaternion_from_matrix(T_tag_0)
        roll, pitch, yaw = tft.euler_from_quaternion(q)

        return (dist < self.distance_threshold) and (abs(yaw) < self.angle_threshold)

    def check_and_land(self):
        """
        Check alignment and land the drone if conditions are met.
        """
        self.aqm.align_dog_with_drone()
        # this already handles tag loss and stops dog if needed

        T = self.aqm.dog_align_drone_matrix

        # Make sure we have a valid transform
        if T is not None and self.is_alignment_success(T):
            self.alignment_counter += 1
            rospy.loginfo_throttle(1.0, f"Alignment frame count: {self.alignment_counter}")
            # If the drone haven`t land off and the alignment counter is enough, the drone will land off
            if self.alignment_counter >= self.required_frames and not self.aligned:
                rospy.loginfo("Alignment held for sufficient frames. Landing drone.")
                print('I will land')
                self.drone_basic.drone_land()
                self.aligned = True
        else:
            self.alignment_counter = 0
            self.aligned = False
    def demo(self):
        """
        Demo sequence for drone takeoff and movement.
        """
        self.drone_basic.record_takeoff_position(self.drone_basic.drone_x, self.drone_basic.drone_y, self.drone_basic.drone_z, self.drone_basic.drone_yaw)
        rate = rospy.Rate(10)
        for _ in range(10):  # 1 second
            rate.sleep()
        self.drone_basic.drone_start()
        for _ in range(10):  # 1 second
            rate.sleep()
        self.drone_basic.drone_takeoff()
        while self.drone_basic.drone_state != 5:
            rate.sleep()
        for _ in range(10):  # 1 second
            rate.sleep()
        self.drone_basic.drone_target('world', self.demo_target_x, self.demo_target_y, self.demo_target_z, 0, 0, 0, 1)
        for _ in range(50):  # 5 seconds
            rate.sleep()
        self.drone_basic.drone_target('world', self.drone_basic.takeoff_x, self.drone_basic.takeoff_y, self.demo_hover_z, 0, 0, 0, 1)
        for _ in range(50):  # 5 seconds
            rate.sleep()
        self.drone_basic.drone_target('world', self.drone_basic.takeoff_x, self.drone_basic.takeoff_y, self.drone_basic.takeoff_z + self.demo_final_z_offset, 0, 0, 0, 1)
    def run(self):
        """
        Main run loop for checking alignment and landing.
        """
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.check_and_land()
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('Aprillandqilin', anonymous=True)
        node = AprillandqilinNode()
        node.dog_basic.stand()
        rate = rospy.Rate(10)
        for _ in range(5):  # 0.5 seconds
            rate.sleep()
        node.demo()
        node.run()
    except rospy.ROSInterruptException:
        pass
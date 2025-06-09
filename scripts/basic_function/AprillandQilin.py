#! /usr/bin/env python
# -*- coding: utf-8 -*-
from AprilmoveQilin import *
from drone_basic_function import DroneBasic
from std_msgs.msg import Empty, UInt8
# It is for  the Coopration for xuanwu and Qilin

class AprillandqilinNode:

    def __init__(self):
        print(f'Hi, I am Cloud Cube')
        rospy.init_node('Aprillandqilin', anonymous=True)

        self.takeoff_x, self.takeoff_y, self.takeoff_z = 0.0, 0.0, 0.0

        self.last_tag_time = rospy.Time.now(), rospy.Time.now()
        self.aligned = False
        self.pause_when_lost = rospy.Duration(1)
        self.drone_pose_matrix = []


        self.drone_basic = DroneBasic()
        self.dog_basic = DogBasic()
        self.aqm = AprilmoveqilinNode()
        # Subscribe and publish.
        # rospy.Subscriber('/uavandgr/event', UInt8, self._callback_event)

        self.pub_event = rospy.Publisher('/uavandgr/event', UInt8, queue_size=10)

    def event(self, x):
        event_msgs = UInt8()
        event_msgs.data = x
        self.pub_event.publish(event_msgs)

    def is_alignment_success(self, T):
        pos = T[:3, 3]
        x, y, _ = pos
        dist = np.linalg.norm([x, y])
        q = tft.quaternion_from_matrix(T)
        _, _, yaw = tft.euler_from_quaternion(q)
        return (dist < self.aqm.coverge_interval) and (abs(yaw) < 0.15)

    def check_and_land(self):
        now = rospy.Time.now()

        if self.aqm.msg_apriltag is None:
            return
        if self.aqm.msg_apriltag.detections:
            self.last_tag_time = now
            self.aqm.dog_align_drone_matrix = self.aqm.origin_2_camera_matrix_param @ self.aqm.find_drone_center(self.aqm.msg_apriltag)
            if self.is_alignment_success(self.dog_align_drone_matrix):
                if not self.aligned:
                    print("Alignment complete. Triggering drone landing.")
                    self.send_drone_land_command()
                    self.aligned = True
            else:
                self.align_dog_with_drone()
                self.aligned = False

        else:

            time_since_last_tag = now - self.last_tag_time
            if time_since_last_tag > self.pause_when_lost:
                print("Tag lost. Pausing dog movement.")
                self.dog_basic_function.qilin_cmd_vel(0, 0, 0, 0, 0)

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

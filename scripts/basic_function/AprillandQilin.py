#! /usr/bin/env python
# -*- coding: utf-8 -*-
from AprilmoveQilin import *
from drone_basic_function import DroneBasic
from dog_basic_function import DogBasic
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
        self.success_start_time = None

        self.drone_basic = DroneBasic()
        self.dog_basic = DogBasic()
        self.aqm = AprilmoveqilinNode()
        # Subscribe and publish.
        # rospy.Subscriber('/uavandgr/event', UInt8, self._callback_event)

        self.pub_event = rospy.Publisher('/uavandgr/event', UInt8, queue_size=10)


    def is_alignment_success(self, T):
        pos = T[:3, 3]
        x, y, _ = pos
        dist = np.linalg.norm([x, y])
        q = tft.quaternion_from_matrix(T)
        _, _, yaw = tft.euler_from_quaternion(q)
        return (dist < 0.04) and (abs(yaw) < 0.15)

    def check_and_land(self):
        self.aqm.align_dog_with_drone()
        # this already handles tag loss and stops dog if needed

        T = self.aqm.dog_align_drone_matrix

        # Make sure we have a valid transform
        if T is not None:
            if self.is_alignment_success(T):
                if self.success_start_time is None:
                    self.success_start_time = time.time()
                elif time.time() - self.success_start_time > 1.0 and not self.aligned:
                    rospy.loginfo("Stable alignment achieved. Landing drone.")
                    self.drone_basic.drone_land()
                    self.aligned = True
            else:
                self.success_start_time = None
                self.aligned = False
        else:
            self.success_start_time = None
            self.aligned = False
if __name__ == '__main__':
    node = AprillandqilinNode()
    node.dog_basic.stand()
    time.sleep(3)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        node.check_and_land()
        rate.sleep()


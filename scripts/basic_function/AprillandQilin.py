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
        self.alignment_counter = 0

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
        if T is not None and self.is_alignment_success(T):
            self.alignment_counter += 1
            rospy.loginfo_throttle(1.0, f"Alignment frame count: {self.alignment_counter}")

            if self.alignment_counter >= self.required_frames and not self.aligned:
                rospy.loginfo("Alignment held for sufficient frames. Landing drone.")
                self.dog_basic.send_drone_land_command()
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
        time.sleep(3)
        node.run()
    except rospy.ROSInterruptException:
        pass


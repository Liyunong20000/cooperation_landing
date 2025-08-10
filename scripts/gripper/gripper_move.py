#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rosparam
import time
from spinal.msg import ServoStates, ServoControlCmd
from std_msgs.msg import Empty


# It is for the gripper of XUANWU

class GripperMoveNode:

    def __init__(self):  # This part will work when this node is used.
        print(f'Hi, I am Cloud Cube')
        self.servo_index = 0
        self.servo_angle = 0.0
        self.servo_temp = 0
        self.servo_load = 0.0
        self.servo_error = 0
        self.servo_target_index = 0
        self.servo_target_angles = 0
        self.robot_ns = rospy.get_param("~robot_ns", "xuanwu")
        self.servo_max_angles = rospy.get_param(self.robot_ns + '/servo_info/max_angles',1690)
        self.servo_min_angles = rospy.get_param(self.robot_ns + '/servo_info/min_angles',900)
        self.servo_max_load = rospy.get_param(self.robot_ns + '/servo_info/max_load',350)
        # Subscribe and publish.
        rospy.Subscriber(self.robot_ns + '/servo/states', ServoStates, self._callback_servo_states)

        self.pub_servo_target = rospy.Publisher(self.robot_ns + '/servo/target_states', ServoControlCmd, queue_size=10)
        self.pub_servo_target_qilin = rospy.Publisher(self.robot_ns + '/servo/target_states/info', ServoControlCmd, queue_size=10)
        self.pub_servo_target_qilin_trigger = rospy.Publisher(self.robot_ns + '/servo/target_states/trigger', Empty, queue_size=10)
        self.pub_servo_return_qilin_trigger = rospy.Publisher(self.robot_ns + '/servo/return/trigger', Empty, queue_size=10)

        time.sleep(1.0)

    def _callback_servo_states(self, msg):
        self.servo_index = msg.servos[0].index
        self.servo_angle = msg.servos[0].angle
        self.servo_temp = msg.servos[0].temp
        self.servo_load = msg.servos[0].load
        self.servo_error = msg.servos[0].error
            # print(f'{self.servo_index, self.servo_angle, self.servo_temp, self.servo_load, self.servo_error}')

    def servo_target_cmd(self, target_index, target_angle):
        servo_target_cmd = ServoControlCmd()
        servo_target_cmd.index = [target_index]
        servo_target_cmd.angles = [target_angle]
        time.sleep(0.1)
        self.pub_servo_target.publish(servo_target_cmd)
        print(f'servo_target_cmd:{servo_target_cmd}')

    def servo_target_cmd_qilin(self, target_index, target_angle):
        servo_target_cmd = ServoControlCmd()
        servo_target_cmd.index = [target_index]
        servo_target_cmd.angles = [target_angle]

        self.pub_servo_target_qilin.publish(servo_target_cmd)
        print(f'servo_target_cmd:{servo_target_cmd}')
        time.sleep(0.2)
        self.grasp_qilin_trigger()
    def return_zero(self):
        time.sleep(0.1)
        self.servo_target_cmd(0, self.servo_max_angles)
        time.sleep(0.5)

    def return_zero_qilin(self):
        time.sleep(0.1)
        self.servo_target_cmd_qilin(0, self.servo_max_angles)
        time.sleep(0.5)

    def grasp(self,servo_index, angle_feed):
        rospy.sleep(0.02)
        r = rospy.Rate(3)
        if self.servo_error == 1:
            rospy.loginfo(f'servo:{servo_index} error!')
            pass
        try:
            while self.servo_load > - (self.servo_max_load-50):
                self.servo_target_index = servo_index
                self.servo_target_angles = self.servo_angle - angle_feed
                self.servo_target_cmd(self.servo_target_index, self.servo_target_angles)
                print(f'---------')
                r.sleep()
        except KeyboardInterrupt:
            pass

        rospy.loginfo(f'servo:{servo_index} arrived target angle!')

    def grasp_qilin(self,servo_index, angle_feed):
        rospy.sleep(0.02)
        r = rospy.Rate(3)
        try:
            while self.servo_load > - (self.servo_max_load-50):
                self.servo_target_index = servo_index
                self.servo_target_angles = self.servo_angle - angle_feed
                self.servo_target_cmd_qilin(self.servo_target_index, self.servo_target_angles)
                print(f'---------')
                r.sleep()
        except KeyboardInterrupt:
            pass

        rospy.loginfo(f'servo:{servo_index} arrived target angle!')

    def grasp_qilin_trigger(self):
        time.sleep(0.1)
        empty_msg = Empty()
        self.pub_servo_target_qilin_trigger.publish(empty_msg)

    def return_qilin_trigger(self):
        time.sleep(0.1)
        empty_msg = Empty()
        self.pub_servo_return_qilin_trigger.publish(empty_msg)

if __name__ == '__main__':
    rospy.init_node('gripper_move', anonymous=True)

    node = GripperMoveNode()
    node.return_zero()
    node.grasp(0 ,50)
    # node.servo_target_cmd(0,2900)
    # node.servo_target_cmd(0, 2600)
    # node.servo_target_cmd(0, 2600)
    while not rospy.is_shutdown():
        rospy.spin()

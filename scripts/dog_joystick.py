#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)

# Copyright (c) 2025, DRAGON Laboratory, The University of Tokyo
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import math
import socket
import struct

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from dog_basic_function import *
from keyboard_control import *
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty, Int32MultiArray

class ControlInterface():
    def __init__(self):

        self.dog_basic = DogBasic()
        self.joy_rate = rospy.get_param('~joy_rate', 0.5)

        self.joy_deadzone = rospy.get_param('~joy_deadzone', 0.2)

        # mode
        self.base_mode = None
        self.ctrl_mode = None

        self.target_base_mode = None
        self.target_ctrl_mode = None

        self.ctrl_x1 = 0.0
        self.ctrl_y1 = 0.0
        self.ctrl_x2 = 0.0
        self.ctrl_y2 = 0.0
        self.ctrl_rl = 0.0
        self.ctrl_rr = 0.0

        self.move_mode = 0
        self.rotate_mode = 1

        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joyCb)
        self.debug_cmd_pub = rospy.Publisher('debug/command', Int32MultiArray, queue_size = 1)
        self.nav_pub = rospy.Publisher('/go1/cmd_vel', Twist, queue_size=1)

        # self.main_timer = rospy.Timer(rospy.Duration(0.1), self.mainCb)
        # self.hb_timer = rospy.Timer(rospy.Duration(1.0), self.hbCb)
        # self.recv_timer = rospy.Timer(rospy.Duration(0.01), self.recvCb)

    def joyCb(self, msg):

        axes = msg.axes
        buttons = msg.buttons

        # if len(axes) != 8 or len(buttons) != 13:
        #     return
        if buttons[8] == 1: # share bottuons
            self.dog_basic.sit()
        if buttons[9] == 1: # options bottuons
            self.dog_basic.stand()
        # if buttons[3] == -1:
        #     self.dog_basic.sit()
        # if buttons[4] == -1:
        #     self.gripper_move.servo_target_cmd_qilin(0, 200)
        self.ctrl_x1 = (self.joy_rate * axes[1])
        self.ctrl_y1 = (self.joy_rate * axes[0])

        self.ctrl_rl = (self.joy_rate * axes[2])
        # print(f"{self.ctrl_x1}, {self.ctrl_y1}, {self.ctrl_rl}")
        self.dog_basic.qilin_cmd_vel(self.ctrl_x1, self.ctrl_y1, 0, 0, self.ctrl_rl)

if __name__ == '__main__':

    rospy.init_node('unitree_go1_joystick')
    interface = ControlInterface()

    rospy.spin()


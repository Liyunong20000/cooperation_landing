"""Dog joystick implementation."""

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
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray

from cooperation_landing.dog_basic_function import DogBasic


class ControlInterface:
    def __init__(self):

        self.dog_basic = DogBasic()
        self.joy_rate = rospy.get_param('~joy_rate', 0.5)

        self.joy_deadzone = rospy.get_param('~joy_deadzone', 0.2)
        self.forward_axis = int(rospy.get_param('~forward_axis', 1))
        self.lateral_axis = int(rospy.get_param('~lateral_axis', 0))
        self.yaw_axis = int(rospy.get_param('~yaw_axis', 2))
        self.sit_button = int(rospy.get_param('~sit_button', 8))
        self.stand_button = int(rospy.get_param('~stand_button', 9))
        for name in ('forward_axis', 'lateral_axis', 'yaw_axis', 'sit_button', 'stand_button'):
            if getattr(self, name) < 0:
                raise ValueError(f'~{name} must be a non-negative joystick index')
        self._previous_buttons = []

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

        joy_topic = rospy.get_param('~joy_topic', '/joy')
        self.joy_sub = rospy.Subscriber(joy_topic, Joy, self.joyCb, queue_size=1)
        self.debug_cmd_pub = rospy.Publisher('debug/command', Int32MultiArray, queue_size=1)

        # self.main_timer = rospy.Timer(rospy.Duration(0.1), self.mainCb)
        # self.hb_timer = rospy.Timer(rospy.Duration(1.0), self.hbCb)
        # self.recv_timer = rospy.Timer(rospy.Duration(0.01), self.recvCb)

    def joyCb(self, msg):
        axes = msg.axes
        buttons = msg.buttons

        required_axis = max(self.forward_axis, self.lateral_axis, self.yaw_axis)
        required_button = max(self.sit_button, self.stand_button)
        if len(axes) <= required_axis or len(buttons) <= required_button:
            rospy.logwarn_throttle(
                5.0,
                'Joystick layout is too small: got %d axes/%d buttons, need indices %d/%d.',
                len(axes),
                len(buttons),
                required_axis,
                required_button,
            )
            return

        def pressed(index):
            was_pressed = bool(self._previous_buttons and self._previous_buttons[index])
            return bool(buttons[index]) and not was_pressed

        if pressed(self.sit_button):
            self.dog_basic.sit()
        if pressed(self.stand_button):
            self.dog_basic.stand()

        def apply_deadzone(value):
            return 0.0 if abs(value) < self.joy_deadzone else value

        self.ctrl_x1 = self.joy_rate * apply_deadzone(axes[self.forward_axis])
        self.ctrl_y1 = self.joy_rate * apply_deadzone(axes[self.lateral_axis])
        self.ctrl_rl = self.joy_rate * apply_deadzone(axes[self.yaw_axis])
        self._previous_buttons = list(buttons)
        self.dog_basic.qilin_cmd_vel(self.ctrl_x1, self.ctrl_y1, 0, 0, self.ctrl_rl)


def main():
    """Run the ROS node."""
    rospy.init_node('unitree_go1_joystick')
    _interface = ControlInterface()

    rospy.spin()

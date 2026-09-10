"""Interactive keyboard control for the Xuanwu gripper."""

import sys
import termios

import rospy

from cooperation_landing.gripper.gripper_move import GripperMoveNode
from cooperation_landing.terminal import get_key

HELP = """
Instruction:

---------------------------
        i             o
  (check state)  (return max)

        j             k         l
      (open)       (close)   (grasp)

Please don't have caps lock on.
CTRL+C to quit
---------------------------
"""


class GripperKeyboardNode:
    def __init__(self):
        self.gripper_move = GripperMoveNode()
        self.step = max(1, int(rospy.get_param('~angle_step', 50)))
        self.grasp_angle = int(rospy.get_param('~grasp_angle', -100))

    def command(self):
        if not sys.stdin.isatty():
            rospy.logfatal('gripper_keyboard requires an interactive terminal')
            return

        settings = termios.tcgetattr(sys.stdin)
        print(HELP)
        try:
            while not rospy.is_shutdown():
                key = get_key(settings)
                if key == '\x03':
                    break
                if key == 'i':
                    rospy.loginfo(
                        'servo index=%s angle=%s temp=%s load=%s error=%s',
                        self.gripper_move.servo_index,
                        self.gripper_move.servo_angle,
                        self.gripper_move.servo_temp,
                        self.gripper_move.servo_load,
                        self.gripper_move.servo_error,
                    )
                elif key == 'o':
                    self.gripper_move.return_zero()
                elif key == 'j':
                    angle = self.gripper_move.servo_angle + self.step
                    self.gripper_move.servo_target_cmd(self.gripper_move.servo_index, angle)
                elif key == 'k':
                    angle = self.gripper_move.servo_angle - self.step
                    self.gripper_move.servo_target_cmd(self.gripper_move.servo_index, angle)
                elif key == 'l':
                    self.gripper_move.servo_target_cmd(
                        self.gripper_move.servo_index, self.grasp_angle
                    )
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    rospy.init_node('gripper_keyboard')
    GripperKeyboardNode().command()

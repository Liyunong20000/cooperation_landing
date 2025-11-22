#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rosparam
import sys, select, termios, tty, time
from spinal.msg import ServoStates, ServoControlCmd
from gripper_move import GripperMoveNode


# It is for the gripper of XUANWU

class GripperKeyboardNode:

    def __init__(self):  # This part will work when this node is used.
        print(f'Hi, I am Cloud Cube')
        self.gripper_move = GripperMoveNode()
        time.sleep(1.0)
        self.msg = """
    Instruction:

    ---------------------------
            i             o      
      (check state)  (return max)

            j             k         l
          (open)       (close)   (grasp) 

    Please don't have caps lock on.
    CTRL+c to quit
    ---------------------------
    """


    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    def command(self):

        try:
                while(True):
                        step = 50
                        # nav_msg.control_frame = FlightNav.WORLD_FRAME
                        # nav_msg.target = FlightNav.COG

                        key = self.getKey()

                        msg = ""

                        if key == 'i':
                                print(
                                f'{self.gripper_move.servo_index, self.gripper_move.servo_angle, self.gripper_move.servo_temp, self.gripper_move.servo_load, self.gripper_move.servo_error}')
                                msg = "check servo state"

                        if key == 'o':
                                self.gripper_move.return_zero()
                                msg = "return servo to zero"

                        if key == 'j':
                                angle = self.gripper_move.servo_angle + step
                                self.gripper_move.servo_target_cmd(self.gripper_move.servo_index, angle)
                                msg = "open"

                        if key == 'k':
                                angle = self.gripper_move.servo_angle - step
                                self.gripper_move.servo_target_cmd(self.gripper_move.servo_index, angle)
                                msg = "close"
                        if key == 'l':
                                # self.gripper_move.grasp(0,50)
                                self.gripper_move.servo_target_cmd(self.gripper_move.servo_index, 200)
                                msg = "grasp"

                        if key == '\x03':
                                break

                        print(f'{msg}')
                        rospy.sleep(0.001)
        except Exception as e:
                print(repr(e))
        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    rospy.init_node('gripper_keyboard', anonymous=True)
    node = GripperKeyboardNode()
    print(node.msg)
    settings = termios.tcgetattr(sys.stdin)
    node.command()


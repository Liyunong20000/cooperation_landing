#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import copy
import math
import os
import select
import sys
import termios
import tty

import rospy
import tf.transformations as tft
from kortex_arm_motion import KortexArmMotion


HELP = """
Kortex MoveIt keyboard step control

Linear step:
  w/s: +x / -x      a/d: +y / -y      r/f: +z / -z

Angular step:
  u/o: +roll / -roll
  i/k: +pitch / -pitch
  j/l: +yaw / -yaw

Actions:
  space: stop current MoveIt execution
  h: MoveIt named target 'home'
  q: quit

Frame:
  --command-frame tool: steps are relative to the end-effector/tool frame.
  --command-frame base: steps are relative to the MoveIt planning frame.
"""


LINEAR_KEYS = {
    "w": ("x", 1.0),
    "s": ("x", -1.0),
    "a": ("y", 1.0),
    "d": ("y", -1.0),
    "r": ("z", 1.0),
    "f": ("z", -1.0),
}

ANGULAR_KEYS = {
    "u": ("roll", 1.0),
    "o": ("roll", -1.0),
    "i": ("pitch", 1.0),
    "k": ("pitch", -1.0),
    "j": ("yaw", 1.0),
    "l": ("yaw", -1.0),
}


def get_key(timeout):
    readable, _, _ = select.select([sys.stdin], [], [], timeout)
    if readable:
        return sys.stdin.read(1)
    return ""


def pose_quaternion(pose):
    return [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]


def set_pose_quaternion(pose, quat):
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]


class KortexMoveItKeyboardControl:
    def __init__(
        self,
        robot_name="my_gen3",
        linear_step=0.03,
        angular_step=0.10,
        key_rate=20.0,
        timeout=15.0,
        command_frame="tool",
        position_tolerance=0.01,
        orientation_tolerance=0.03,
    ):
        self.robot_name = robot_name
        self.linear_step = linear_step
        self.angular_step = angular_step
        self.key_rate = key_rate
        self.timeout = timeout
        self.command_frame = command_frame
        self.position_tolerance = position_tolerance
        self.orientation_tolerance = orientation_tolerance

        self.motion = KortexArmMotion(
            robot_name=robot_name,
            timeout=timeout,
            wait_for_init=True,
        )
        self.motion.init_moveit()
        self.group = self.motion.arm_group
        self.group.set_goal_position_tolerance(position_tolerance)
        self.group.set_goal_orientation_tolerance(orientation_tolerance)

    def print_state(self, text):
        sys.stdout.write("\r" + text.ljust(100))
        sys.stdout.flush()

    def current_pose(self):
        return self.group.get_current_pose().pose

    def apply_tool_translation(self, pose, axis, distance):
        quat = pose_quaternion(pose)
        rotation = tft.quaternion_matrix(quat)
        axis_vectors = {
            "x": rotation[:3, 0],
            "y": rotation[:3, 1],
            "z": rotation[:3, 2],
        }
        vector = axis_vectors[axis] * distance
        pose.position.x += vector[0]
        pose.position.y += vector[1]
        pose.position.z += vector[2]

    def apply_base_translation(self, pose, axis, distance):
        if axis == "x":
            pose.position.x += distance
        elif axis == "y":
            pose.position.y += distance
        elif axis == "z":
            pose.position.z += distance

    def apply_rotation(self, pose, axis, angle):
        current = pose_quaternion(pose)

        if axis == "roll":
            delta = tft.quaternion_from_euler(angle, 0.0, 0.0)
        elif axis == "pitch":
            delta = tft.quaternion_from_euler(0.0, angle, 0.0)
        else:
            delta = tft.quaternion_from_euler(0.0, 0.0, angle)

        if self.command_frame == "tool":
            updated = tft.quaternion_multiply(current, delta)
        else:
            updated = tft.quaternion_multiply(delta, current)

        updated = tft.unit_vector(updated)
        set_pose_quaternion(pose, updated)

    def execute_pose(self, pose, label):
        self.group.set_pose_target(pose)
        rospy.loginfo("MoveIt step: %s", label)
        success = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        self.print_state("{} {}".format(label, "ok" if success else "failed"))
        return success

    def step_linear(self, axis, direction):
        pose = copy.deepcopy(self.current_pose())
        distance = direction * self.linear_step

        if self.command_frame == "tool":
            self.apply_tool_translation(pose, axis, distance)
        else:
            self.apply_base_translation(pose, axis, distance)

        return self.execute_pose(
            pose,
            "{}{} {:.3f}m".format(axis, "+" if direction > 0 else "-", self.linear_step),
        )

    def step_angular(self, axis, direction):
        pose = copy.deepcopy(self.current_pose())
        angle = direction * self.angular_step
        self.apply_rotation(pose, axis, angle)
        return self.execute_pose(
            pose,
            "{}{} {:.1f}deg".format(
                axis,
                "+" if direction > 0 else "-",
                math.degrees(self.angular_step),
            ),
        )

    def stop(self):
        self.group.stop()
        self.group.clear_pose_targets()
        self.print_state("stop")

    def home(self):
        ok = self.motion.moveit_named_target("home")
        self.print_state("home {}".format("ok" if ok else "failed"))
        return ok

    def spin(self):
        old_settings = termios.tcgetattr(sys.stdin)
        rate = rospy.Rate(self.key_rate)

        print(HELP)
        self.print_state("ready")

        try:
            tty.setcbreak(sys.stdin.fileno())

            while not rospy.is_shutdown():
                key = get_key(1.0 / self.key_rate)

                if key in LINEAR_KEYS:
                    axis, direction = LINEAR_KEYS[key]
                    self.step_linear(axis, direction)
                elif key in ANGULAR_KEYS:
                    axis, direction = ANGULAR_KEYS[key]
                    self.step_angular(axis, direction)
                elif key == " ":
                    self.stop()
                elif key == "h":
                    self.home()
                elif key in ("q", "\x03"):
                    break

                rate.sleep()
        finally:
            self.stop()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            print("")


def main():
    parser = argparse.ArgumentParser(
        description="MoveIt keyboard step control for a Kortex arm."
    )
    parser.add_argument("--robot-name", default="my_gen3")
    parser.add_argument("--linear-step", type=float, default=0.03)
    parser.add_argument("--angular-step", type=float, default=0.10)
    parser.add_argument("--key-rate", type=float, default=20.0)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument(
        "--command-frame",
        choices=("tool", "base"),
        default="tool",
        help="Frame used for keyboard increments.",
    )
    parser.add_argument("--position-tolerance", type=float, default=0.01)
    parser.add_argument("--orientation-tolerance", type=float, default=0.03)

    # Backward-compatible aliases from the previous Twist implementation.
    parser.add_argument("--linear-speed", type=float, dest="linear_step")
    parser.add_argument("--angular-speed", type=float, dest="angular_step")
    parser.add_argument("--publish-rate", type=float, dest="key_rate")
    parser.add_argument("--command-mode", choices=("topic", "service", "both"))
    parser.add_argument("--reference-frame")

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    if args.command_mode:
        rospy.logwarn("--command-mode is ignored; this script now uses MoveIt steps.")
    if args.reference_frame:
        rospy.logwarn("--reference-frame is ignored; use --command-frame tool/base.")
    if args.linear_step <= 0.0:
        parser.error("--linear-step must be greater than 0")
    if args.angular_step <= 0.0:
        parser.error("--angular-step must be greater than 0")
    if args.key_rate <= 0.0:
        parser.error("--key-rate must be greater than 0")

    rospy.init_node("kortex_keyboard_control", anonymous=True)

    if not os.isatty(sys.stdin.fileno()):
        rospy.logwarn("stdin is not a TTY; keyboard input may not work in this terminal")

    controller = KortexMoveItKeyboardControl(
        robot_name=args.robot_name,
        linear_step=args.linear_step,
        angular_step=args.angular_step,
        key_rate=args.key_rate,
        timeout=args.timeout,
        command_frame=args.command_frame,
        position_tolerance=args.position_tolerance,
        orientation_tolerance=args.orientation_tolerance,
    )
    controller.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())

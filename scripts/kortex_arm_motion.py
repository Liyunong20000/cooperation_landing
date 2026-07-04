#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import sys
import time

import actionlib
import geometry_msgs.msg
import moveit_commander
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from kortex_driver.msg import ActionEvent, ActionNotification
from kortex_driver.srv import ExecuteAction, ExecuteActionRequest, ReadAllActions
from trajectory_msgs.msg import JointTrajectoryPoint


PRESET_TRAJECTORIES = {
    6: {
        "ready": [
            [-0.45, 0.15, -2.05, 2.15, -0.65, -1.15],
        ],
        "observe": [
            [0.0, 0.55, -2.2, 2.0, -0.2, -1.1],
        ],
        "wave": [
            [0.0, 0.35, 3.14, -2.0, 0.0, -1.0],
            [0.45, 0.35, 3.14, -2.0, 0.0, -1.0],
            [-0.45, 0.35, 3.14, -2.0, 0.0, -1.0],
            [0.0, 0.35, 3.14, -2.0, 0.0, -1.0],
        ],
    },
    7: {
        "ready": [
            [1.2, 0.4, 3.0, -1.8, 0.0, -1.0, 1.4],
        ],
        "observe": [
            [0.0, 0.75, 3.05, -1.45, 0.0, -0.85, 1.57],
        ],
        "wave": [
            [0.0, 0.35, 3.14, -2.0, 0.0, -1.0, 1.57],
            [0.5, 0.35, 3.14, -2.0, 0.0, -1.0, 1.2],
            [-0.5, 0.35, 3.14, -2.0, 0.0, -1.0, 1.9],
            [0.0, 0.35, 3.14, -2.0, 0.0, -1.0, 1.57],
        ],
    },
}


def parse_positions(text):
    return [float(item.strip()) for item in text.split(",") if item.strip()]


def normalized_ns(robot_name):
    return "/" + robot_name.strip("/")


class KortexArmMotion:
    def __init__(
        self,
        robot_name="my_gen3",
        controller="gen3_joint_trajectory_controller",
        timeout=15.0,
        wait_for_init=True,
    ):
        self.robot_name = robot_name
        self.robot_ns = normalized_ns(robot_name)
        self.controller = controller
        self.timeout = timeout
        self.last_action_notif_type = None
        self._moveit_initialized = False
        self.robot = None
        self.scene = None
        self.arm_group = None

        if wait_for_init:
            self.wait_for_initialization()

        self.action_topic_sub = rospy.Subscriber(
            self.robot_ns + "/action_topic",
            ActionNotification,
            self._action_notification_cb,
        )

    def _action_notification_cb(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_initialization(self):
        param_name = self.robot_ns + "/is_initialized"
        deadline = rospy.Time.now() + rospy.Duration(self.timeout)

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if rospy.get_param(param_name, False):
                return True
            rospy.sleep(0.2)

        raise RuntimeError(
            "Timed out waiting for {}. Is spawn_kortex_robot.launch ready?".format(
                param_name
            )
        )

    def wait_for_kortex_action(self):
        deadline = time.time() + self.timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.last_action_notif_type == ActionEvent.ACTION_END:
                rospy.loginfo("Received ACTION_END notification")
                return True
            if self.last_action_notif_type == ActionEvent.ACTION_ABORT:
                rospy.logerr("Received ACTION_ABORT notification")
                return False
            time.sleep(0.01)

        rospy.logwarn("Timed out waiting for Kortex action notification")
        return False

    def get_controller_joints(self):
        param_name = "{}/{}/joints".format(self.robot_ns, self.controller)
        joints = rospy.get_param(param_name, [])
        if not joints:
            raise RuntimeError("No joints found at {}".format(param_name))
        if len(joints) not in PRESET_TRAJECTORIES:
            raise RuntimeError(
                "Unsupported joint count {} from {}: {}".format(
                    len(joints), param_name, joints
                )
            )
        return joints

    def build_joint_trajectory_goal(self, joint_names, waypoints, segment_duration):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = joint_names
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)

        for index, positions in enumerate(waypoints, start=1):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = [0.0] * len(joint_names)
            point.time_from_start = rospy.Duration(segment_duration * index)
            goal.trajectory.points.append(point)

        return goal

    def send_joint_positions(self, positions, duration=6.0):
        joint_names = self.get_controller_joints()
        if len(positions) != len(joint_names):
            raise ValueError(
                "Got {} joint values, but controller expects {} joints: {}".format(
                    len(positions), len(joint_names), joint_names
                )
            )
        return self.send_joint_waypoints([positions], duration)

    def send_preset(self, preset, duration=6.0):
        joint_names = self.get_controller_joints()
        waypoints = PRESET_TRAJECTORIES[len(joint_names)][preset]
        return self.send_joint_waypoints(waypoints, duration)

    def send_joint_waypoints(self, waypoints, duration=6.0):
        if duration <= 0.0:
            raise ValueError("duration must be greater than 0")

        joint_names = self.get_controller_joints()
        action_name = "{}/{}/follow_joint_trajectory".format(
            self.robot_ns, self.controller
        )
        client = actionlib.SimpleActionClient(action_name, FollowJointTrajectoryAction)

        rospy.loginfo("Waiting for action server %s", action_name)
        if not client.wait_for_server(rospy.Duration(self.timeout)):
            raise RuntimeError("Action server {} is not available".format(action_name))

        rospy.loginfo("Using joints: %s", ", ".join(joint_names))
        goal = self.build_joint_trajectory_goal(joint_names, waypoints, duration)

        rospy.loginfo("Sending %d waypoint(s) to %s", len(waypoints), action_name)
        client.send_goal(goal)
        client.wait_for_result()

        result = client.get_result()
        if result and result.error_code == result.SUCCESSFUL:
            rospy.loginfo("Trajectory finished successfully")
            return True

        rospy.logerr("Trajectory failed, action state=%s, result=%s", client.get_state(), result)
        return False

    def get_builtin_action(self, action_name):
        action_name = action_name.lower()
        read_all_actions_name = self.robot_ns + "/base/read_all_actions"
        rospy.wait_for_service(read_all_actions_name, timeout=self.timeout)
        read_all_actions = rospy.ServiceProxy(read_all_actions_name, ReadAllActions)
        action_list = read_all_actions().output.action_list

        for action in action_list:
            if action.name.lower() == action_name:
                return action

        available = ", ".join(sorted(action.name for action in action_list))
        raise ValueError(
            "Unknown Kortex action '{}'. Available actions: {}".format(
                action_name, available or "<none>"
            )
        )

    def execute_builtin_action(self, action_name):
        execute_action_name = self.robot_ns + "/base/execute_action"
        rospy.wait_for_service(execute_action_name, timeout=self.timeout)
        execute_action = rospy.ServiceProxy(execute_action_name, ExecuteAction)
        action = self.get_builtin_action(action_name)

        self.last_action_notif_type = None
        execute_req = ExecuteActionRequest()
        execute_req.input = action
        rospy.loginfo(
            "Executing Kortex action '%s' (id=%d)",
            action.name,
            action.handle.identifier,
        )
        execute_action(execute_req)
        return self.wait_for_kortex_action()

    def home(self):
        return self.execute_builtin_action("home")

    def zero(self):
        return self.execute_builtin_action("zero")

    def retract(self):
        return self.execute_builtin_action("retract")

    def init_moveit(self):
        if self._moveit_initialized:
            return

        moveit_commander.roscpp_initialize(sys.argv)
        robot_description = self.robot_ns + "/robot_description"
        self.robot = moveit_commander.RobotCommander(
            robot_description=robot_description,
            ns=self.robot_ns,
        )
        self.scene = moveit_commander.PlanningSceneInterface(ns=self.robot_ns)
        self.arm_group = moveit_commander.MoveGroupCommander(
            "arm",
            robot_description=robot_description,
            ns=self.robot_ns,
        )
        self.arm_group.set_planning_time(10.0)
        self.arm_group.set_num_planning_attempts(10)
        self._moveit_initialized = True

    def moveit_named_target(self, target):
        self.init_moveit()
        available = self.arm_group.get_named_targets()
        if target not in available:
            raise ValueError(
                "Unknown MoveIt named target '{}'. Available targets: {}".format(
                    target, ", ".join(sorted(available)) or "<none>"
                )
            )

        rospy.loginfo("Planning to MoveIt named target '%s'", target)
        self.arm_group.set_named_target(target)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()

        if success:
            rospy.loginfo("MoveIt named target execution finished successfully")
        else:
            rospy.logerr("MoveIt named target execution failed")
        return success

    def moveit_pose(
        self,
        x,
        y,
        z,
        qx,
        qy,
        qz,
        qw,
        frame_id=None,
        position_tolerance=0.01,
        orientation_tolerance=0.02,
    ):
        self.init_moveit()

        planning_frame = frame_id or self.arm_group.get_planning_frame()
        self.arm_group.set_pose_reference_frame(planning_frame)
        self.arm_group.set_goal_position_tolerance(position_tolerance)
        self.arm_group.set_goal_orientation_tolerance(orientation_tolerance)

        pose = geometry_msgs.msg.Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        if frame_id:
            stamped_pose = geometry_msgs.msg.PoseStamped()
            stamped_pose.header.frame_id = frame_id
            stamped_pose.header.stamp = rospy.Time.now()
            stamped_pose.pose = pose
            self.arm_group.set_pose_target(stamped_pose)
        else:
            self.arm_group.set_pose_target(pose)

        rospy.loginfo(
            "Planning to pose in %s: position=(%.3f, %.3f, %.3f), orientation=(%.3f, %.3f, %.3f, %.3f)",
            planning_frame,
            x,
            y,
            z,
            qx,
            qy,
            qz,
            qw,
        )
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        if success:
            rospy.loginfo("MoveIt pose execution finished successfully")
        else:
            rospy.logerr("MoveIt pose execution failed")
        return success

    def current_moveit_pose(self):
        self.init_moveit()
        return self.arm_group.get_current_pose().pose


def add_common_args(parser):
    parser.add_argument("--robot-name", default="my_gen3")
    parser.add_argument("--controller", default="gen3_joint_trajectory_controller")
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument(
        "--skip-init-wait",
        action="store_true",
        help="Do not wait for /<robot_name>/is_initialized.",
    )


def main():
    parser = argparse.ArgumentParser(
        description="Control a simulated Kinova Kortex arm from one helper class."
    )
    add_common_args(parser)

    subparsers = parser.add_subparsers(dest="command")

    preset_parser = subparsers.add_parser(
        "preset",
        help="Send a predefined joint trajectory through FollowJointTrajectory.",
    )
    preset_parser.add_argument(
        "preset",
        choices=sorted(PRESET_TRAJECTORIES[6].keys()),
        nargs="?",
        default="ready",
    )
    preset_parser.add_argument("--duration", type=float, default=6.0)

    joints_parser = subparsers.add_parser(
        "joints",
        help="Send explicit joint positions through FollowJointTrajectory.",
    )
    joints_parser.add_argument("positions", type=parse_positions)
    joints_parser.add_argument("--duration", type=float, default=6.0)

    action_parser = subparsers.add_parser(
        "action",
        help="Execute a Kortex action by name through read_all_actions/execute_action services.",
    )
    action_parser.add_argument("name")

    named_target_parser = subparsers.add_parser(
        "named-target",
        help="Plan and execute a MoveIt named target such as home, retract, or vertical.",
    )
    named_target_parser.add_argument("name")

    pose_parser = subparsers.add_parser(
        "pose",
        help="Plan and execute a Cartesian pose with MoveIt.",
    )
    pose_parser.add_argument("--x", type=float, required=True)
    pose_parser.add_argument("--y", type=float, required=True)
    pose_parser.add_argument("--z", type=float, required=True)
    pose_parser.add_argument("--qx", type=float, required=True)
    pose_parser.add_argument("--qy", type=float, required=True)
    pose_parser.add_argument("--qz", type=float, required=True)
    pose_parser.add_argument("--qw", type=float, required=True)
    pose_parser.add_argument("--frame-id", default=None)
    pose_parser.add_argument("--position-tolerance", type=float, default=0.01)
    pose_parser.add_argument("--orientation-tolerance", type=float, default=0.02)

    current_pose_parser = subparsers.add_parser(
        "current-pose",
        help="Print the current MoveIt end-effector pose.",
    )
    current_pose_parser.set_defaults(command="current-pose")

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    if args.command is None:
        args.command = "preset"
        args.preset = "ready"
        args.duration = 6.0

    rospy.init_node("kortex_arm_motion", anonymous=True)

    try:
        motion = KortexArmMotion(
            robot_name=args.robot_name,
            controller=args.controller,
            timeout=args.timeout,
            wait_for_init=not args.skip_init_wait,
        )

        if args.command == "preset":
            success = motion.send_preset(args.preset, args.duration)
        elif args.command == "joints":
            success = motion.send_joint_positions(args.positions, args.duration)
        elif args.command == "action":
            success = motion.execute_builtin_action(args.name)
        elif args.command == "named-target":
            success = motion.moveit_named_target(args.name)
        elif args.command == "pose":
            success = motion.moveit_pose(
                args.x,
                args.y,
                args.z,
                args.qx,
                args.qy,
                args.qz,
                args.qw,
                frame_id=args.frame_id,
                position_tolerance=args.position_tolerance,
                orientation_tolerance=args.orientation_tolerance,
            )
        elif args.command == "current-pose":
            rospy.loginfo("Current end-effector pose:\n%s", motion.current_moveit_pose())
            success = True
        else:
            raise RuntimeError("Unsupported command {}".format(args.command))
    except Exception as exc:
        rospy.logerr(str(exc))
        return 1

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())

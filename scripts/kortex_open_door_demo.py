#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import sys

import moveit_commander
import rospy
import tf.transformations as tft
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from gazebo_msgs.srv import GetModelProperties
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import CollisionObject

from kortex_arm_motion import KortexArmMotion


MOVING_DOOR_OBJECTS = [
    "door_door_panel",
    "door_handle",
    "door_tag_plate",
]


def quaternion_from_rpy(roll, pitch, yaw):
    q = tft.quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(q[0], q[1], q[2], q[3])


def make_pose(x, y, z, quat):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation = quat
    return pose


class KortexOpenDoorDemo:
    def __init__(self, args):
        self.args = args
        self.motion = KortexArmMotion(
            robot_name=args.robot_name,
            timeout=args.timeout,
            wait_for_init=True,
        )
        self.motion.init_moveit()
        self.group = self.motion.arm_group
        self.group.set_planning_time(args.planning_time)
        self.group.set_num_planning_attempts(args.planning_attempts)
        self.group.set_goal_position_tolerance(args.position_tolerance)
        self.group.set_goal_orientation_tolerance(args.orientation_tolerance)

        self.collision_pub = rospy.Publisher(
            "/{}/collision_object".format(args.robot_name.strip("/")),
            CollisionObject,
            queue_size=10,
        )

        rospy.wait_for_service(args.attach_service, timeout=args.timeout)
        rospy.wait_for_service(args.detach_service, timeout=args.timeout)
        rospy.wait_for_service("/gazebo/get_model_properties", timeout=args.timeout)
        self.attach_srv = rospy.ServiceProxy(args.attach_service, Attach)
        self.detach_srv = rospy.ServiceProxy(args.detach_service, Attach)
        self.get_model_properties_srv = rospy.ServiceProxy(
            "/gazebo/get_model_properties",
            GetModelProperties,
        )

    def remove_moving_door_collision(self):
        rospy.set_param("/door_tag/publish_moving_collision", False)
        rospy.sleep(0.2)
        for object_id in MOVING_DOOR_OBJECTS:
            obj = CollisionObject()
            obj.header.frame_id = self.group.get_planning_frame()
            obj.header.stamp = rospy.Time.now()
            obj.id = object_id
            obj.operation = CollisionObject.REMOVE
            self.collision_pub.publish(obj)
        rospy.sleep(0.5)

    def restore_moving_door_collision(self):
        rospy.set_param("/door_tag/publish_moving_collision", True)

    def go_to_pose(self, pose, label):
        rospy.loginfo("Planning to %s", label)
        self.group.set_pose_target(pose)
        success = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        if not success:
            rospy.logerr("MoveIt failed while going to %s", label)
        return success

    def attach(self):
        if not self.model_has_link(self.args.robot_model, self.args.robot_link):
            return False
        if not self.model_has_link(self.args.door_model, self.args.door_link):
            return False

        req = AttachRequest()
        req.model_name_1 = self.args.robot_model
        req.link_name_1 = self.args.robot_link
        req.model_name_2 = self.args.door_model
        req.link_name_2 = self.args.door_link
        rospy.loginfo(
            "Attaching %s::%s to %s::%s",
            req.model_name_1,
            req.link_name_1,
            req.model_name_2,
            req.link_name_2,
        )
        res = self.attach_srv(req)
        if not res.ok:
            rospy.logwarn("Attach service returned ok=false")
        return res.ok

    def model_has_link(self, model_name, link_name):
        res = self.get_model_properties_srv(model_name)
        if not res.success:
            rospy.logerr("Model '%s' was not found: %s", model_name, res.status_message)
            return False
        if link_name not in res.body_names:
            rospy.logerr(
                "Link '%s' was not found in model '%s'. Available links: %s",
                link_name,
                model_name,
                ", ".join(res.body_names),
            )
            return False
        return True

    def detach(self):
        req = AttachRequest()
        req.model_name_1 = self.args.robot_model
        req.link_name_1 = self.args.robot_link
        req.model_name_2 = self.args.door_model
        req.link_name_2 = self.args.door_link
        rospy.loginfo("Detaching door")
        return self.detach_srv(req).ok

    def handle_pose_at_angle(self, angle, offset_x=0.0, offset_y=0.0, offset_z=0.0):
        hinge_x = self.args.hinge_x
        hinge_y = self.args.hinge_y
        hinge_z = self.args.hinge_z
        local_x = self.args.handle_local_x + offset_x
        local_y = self.args.handle_local_y + offset_y
        local_z = self.args.handle_local_z + offset_z

        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        x = hinge_x + cos_a * local_x - sin_a * local_y
        y = hinge_y + sin_a * local_x + cos_a * local_y
        z = hinge_z + local_z
        return x, y, z

    def target_pose_at_angle(self, angle, quat):
        x, y, z = self.handle_pose_at_angle(
            angle,
            self.args.contact_offset_x,
            self.args.contact_offset_y,
            self.args.contact_offset_z,
        )
        return make_pose(x, y, z, quat)

    def build_open_waypoints(self, quat, start_angle=0.0):
        waypoints = []
        steps = max(1, self.args.open_steps)
        for index in range(1, steps + 1):
            ratio = float(index) / float(steps)
            angle = start_angle + (self.args.open_angle - start_angle) * ratio
            waypoints.append(self.target_pose_at_angle(angle, quat))
        return waypoints

    def execute_cartesian_open_path(self, waypoints):
        trajectory, fraction = self.group.compute_cartesian_path(
            waypoints,
            self.args.eef_step,
            self.args.cartesian_avoid_collisions,
        )
        rospy.loginfo("Cartesian open path fraction: %.3f", fraction)
        if fraction < self.args.min_path_fraction:
            rospy.logwarn(
                "Cartesian open path fraction is too low; falling back to waypoint planning"
            )
            return self.execute_waypoint_open_path(waypoints)
        return self.group.execute(trajectory, wait=True)

    def execute_waypoint_open_path(self, waypoints):
        for index, waypoint in enumerate(waypoints, start=1):
            if not self.go_to_pose(waypoint, "door open waypoint {}/{}".format(index, len(waypoints))):
                return False
        return True

    def run(self):
        quat = quaternion_from_rpy(
            self.args.tool_roll,
            self.args.tool_pitch,
            self.args.tool_yaw,
        )

        contact_pose = self.target_pose_at_angle(0.0, quat)
        approach_pose = make_pose(
            contact_pose.position.x + self.args.approach_offset_x,
            contact_pose.position.y + self.args.approach_offset_y,
            contact_pose.position.z + self.args.approach_offset_z,
            quat,
        )

        attached = False
        completed = False
        try:
            if not self.go_to_pose(approach_pose, "door handle approach"):
                return False
            if not self.go_to_pose(contact_pose, "door handle contact"):
                return False

            self.remove_moving_door_collision()

            if self.attach():
                attached = True
            else:
                rospy.logwarn("Continuing even though attach returned false")

            start_angle = 0.0
            if self.args.pre_push_angle > 0.0:
                pre_push_pose = self.target_pose_at_angle(self.args.pre_push_angle, quat)
                if not self.go_to_pose(pre_push_pose, "door handle pre-push"):
                    return False
                start_angle = self.args.pre_push_angle

            waypoints = self.build_open_waypoints(quat, start_angle)
            if not self.execute_cartesian_open_path(waypoints):
                return False

            rospy.loginfo("Door opening demo completed")
            completed = True
            return True
        finally:
            if attached and (self.args.detach_at_end or not completed):
                self.detach()
            if self.args.restore_collision:
                self.restore_moving_door_collision()


def main():
    parser = argparse.ArgumentParser(description="Open the Gazebo door with Kortex and gazebo_ros_link_attacher.")
    parser.add_argument("--robot-name", default="my_gen3")
    parser.add_argument("--robot-model", default="my_gen3")
    parser.add_argument("--robot-link", default="bracelet_link")
    parser.add_argument("--door-model", default="hinged_door_with_tag")
    parser.add_argument("--door-link", default="door_handle")
    parser.add_argument("--attach-service", default="/link_attacher_node/attach")
    parser.add_argument("--detach-service", default="/link_attacher_node/detach")
    parser.add_argument("--timeout", type=float, default=15.0)

    parser.add_argument("--hinge-x", type=float, default=0.50)
    parser.add_argument("--hinge-y", type=float, default=0.38)
    parser.add_argument("--hinge-z", type=float, default=0.45)
    parser.add_argument("--handle-local-x", type=float, default=-0.07)
    parser.add_argument("--handle-local-y", type=float, default=-0.58)
    parser.add_argument("--handle-local-z", type=float, default=0.05)

    parser.add_argument("--approach-offset-x", type=float, default=-0.08)
    parser.add_argument("--approach-offset-y", type=float, default=0.0)
    parser.add_argument("--approach-offset-z", type=float, default=0.0)
    parser.add_argument("--contact-offset-x", type=float, default=-0.01)
    parser.add_argument("--contact-offset-y", type=float, default=0.0)
    parser.add_argument("--contact-offset-z", type=float, default=0.0)
    parser.add_argument("--open-angle", type=float, default=0.35)
    parser.add_argument("--open-steps", type=int, default=1)
    parser.add_argument("--pre-push-angle", type=float, default=0.0)
    parser.add_argument("--eef-step", type=float, default=0.01)
    parser.add_argument("--jump-threshold", type=float, default=0.0)
    parser.add_argument("--cartesian-avoid-collisions", action="store_true")
    parser.add_argument("--min-path-fraction", type=float, default=0.8)

    parser.add_argument("--tool-roll", type=float, default=math.pi / 2.0)
    parser.add_argument("--tool-pitch", type=float, default=0.0)
    parser.add_argument("--tool-yaw", type=float, default=math.pi / 2.0)
    parser.add_argument("--position-tolerance", type=float, default=0.02)
    parser.add_argument("--orientation-tolerance", type=float, default=0.2)
    parser.add_argument("--planning-time", type=float, default=10.0)
    parser.add_argument("--planning-attempts", type=int, default=10)
    parser.add_argument("--detach-at-end", action="store_true")
    parser.add_argument("--no-restore-collision", action="store_false", dest="restore_collision")
    parser.set_defaults(restore_collision=True)

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("kortex_open_door_demo", anonymous=True)
    success = KortexOpenDoorDemo(args).run()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())

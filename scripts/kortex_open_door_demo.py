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
from moveit_msgs.msg import CollisionObject, Constraints, OrientationConstraint

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
        self.pose_target_link = args.pose_target_link or args.robot_link
        rospy.loginfo(
            "MoveIt default end-effector link: '%s'; pose target link: '%s'",
            self.group.get_end_effector_link(),
            self.pose_target_link,
        )

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

    def go_to_pose(self, pose, label, target_link=None):
        target_link = target_link or self.pose_target_link
        rospy.loginfo("Planning to %s with target link %s", label, target_link)
        self.group.set_pose_target(pose, target_link)
        success = False
        try:
            plan = self.plan_to_current_target(label)
            if plan is not None:
                success = self.execute_timed_plan(plan, label)
        finally:
            self.group.stop()
            self.group.clear_pose_targets()
        if not success:
            if self.pose_reached(pose, target_link):
                rospy.logwarn(
                    "MoveIt reported failure for %s, but %s is within goal tolerance; continuing",
                    label,
                    target_link,
                )
                return True
            rospy.logerr("MoveIt failed while going to %s", label)
        return success

    def plan_to_current_target(self, label):
        result = self.group.plan()
        if isinstance(result, tuple):
            if len(result) >= 2:
                success = bool(result[0])
                plan = result[1]
            else:
                success = False
                plan = None
        else:
            plan = result
            points = getattr(plan.joint_trajectory, "points", [])
            success = len(points) > 0

        if not success or plan is None or len(plan.joint_trajectory.points) == 0:
            rospy.logerr("MoveIt failed to plan %s", label)
            return None
        return plan

    def execute_timed_plan(self, plan, label):
        timed_plan = self.retime_plan(plan)
        self.enforce_strictly_increasing_time(timed_plan)
        return self.group.execute(timed_plan, wait=True)

    def retime_plan(self, plan):
        return self.group.retime_trajectory(
            self.group.get_current_state(),
            plan,
            self.args.velocity_scaling,
            self.args.acceleration_scaling,
        )

    def enforce_strictly_increasing_time(self, plan):
        points = plan.joint_trajectory.points
        last_time = rospy.Duration(0.0)
        min_dt = rospy.Duration(self.args.min_waypoint_dt)
        fixed = 0
        for point in points:
            if point.time_from_start <= last_time:
                point.time_from_start = last_time + min_dt
                fixed += 1
            last_time = point.time_from_start
        if fixed:
            rospy.logwarn(
                "Adjusted %d trajectory waypoint timestamps to be strictly increasing",
                fixed,
            )

    def pose_reached(self, target_pose, target_link):
        current_pose = self.group.get_current_pose(target_link).pose
        dx = current_pose.position.x - target_pose.position.x
        dy = current_pose.position.y - target_pose.position.y
        dz = current_pose.position.z - target_pose.position.z
        position_error = math.sqrt(dx * dx + dy * dy + dz * dz)

        cq = current_pose.orientation
        tq = target_pose.orientation
        current_q = [cq.x, cq.y, cq.z, cq.w]
        target_q = [tq.x, tq.y, tq.z, tq.w]
        dot = abs(sum(c * t for c, t in zip(current_q, target_q)))
        dot = max(-1.0, min(1.0, dot))
        orientation_error = 2.0 * math.acos(dot)

        rospy.logwarn(
            "Post-failure pose error for %s: position %.4f m, orientation %.4f rad",
            target_link,
            position_error,
            orientation_error,
        )
        return (
            position_error <= self.args.position_tolerance * 2.0
            and orientation_error <= self.args.orientation_tolerance * 2.0
        )

    def set_orientation_constraint(self, quat, name="orientation", target_link=None):
        if not self.args.constrain_approach_orientation:
            return

        target_link = target_link or self.pose_target_link
        constraint = OrientationConstraint()
        constraint.header.frame_id = self.group.get_planning_frame()
        constraint.link_name = target_link
        constraint.orientation = quat
        constraint.absolute_x_axis_tolerance = self.args.orientation_constraint_tolerance
        constraint.absolute_y_axis_tolerance = self.args.orientation_constraint_tolerance
        constraint.absolute_z_axis_tolerance = self.args.orientation_constraint_tolerance
        constraint.weight = 1.0

        constraints = Constraints()
        constraints.name = name
        constraints.orientation_constraints.append(constraint)
        self.group.set_path_constraints(constraints)

    def clear_orientation_constraint(self):
        self.group.clear_path_constraints()

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
        self.group.stop()
        rospy.sleep(self.args.pre_detach_settle_time)
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

    def build_open_waypoints(self, quat, start_angle=0.0, base_pose=None):
        waypoints = []
        if base_pose is None:
            base_pose = self.target_pose_at_angle(start_angle, quat)
        base_handle_x, base_handle_y, base_handle_z = self.handle_pose_at_angle(
            start_angle,
            self.args.contact_offset_x,
            self.args.contact_offset_y,
            self.args.contact_offset_z,
        )
        steps = max(1, self.args.open_steps)
        for index in range(1, steps + 1):
            ratio = float(index) / float(steps)
            angle = start_angle + (self.args.open_angle - start_angle) * ratio
            handle_x, handle_y, handle_z = self.handle_pose_at_angle(
                angle,
                self.args.contact_offset_x,
                self.args.contact_offset_y,
                self.args.contact_offset_z,
            )
            waypoints.append(
                make_pose(
                    base_pose.position.x + handle_x - base_handle_x,
                    base_pose.position.y + handle_y - base_handle_y,
                    base_pose.position.z + handle_z - base_handle_z,
                    quat,
                )
            )
        return waypoints

    def displaced_pose_at_angle(self, base_pose, base_angle, target_angle, quat):
        base_handle_x, base_handle_y, base_handle_z = self.handle_pose_at_angle(
            base_angle,
            self.args.contact_offset_x,
            self.args.contact_offset_y,
            self.args.contact_offset_z,
        )
        handle_x, handle_y, handle_z = self.handle_pose_at_angle(
            target_angle,
            self.args.contact_offset_x,
            self.args.contact_offset_y,
            self.args.contact_offset_z,
        )
        return make_pose(
            base_pose.position.x + handle_x - base_handle_x,
            base_pose.position.y + handle_y - base_handle_y,
            base_pose.position.z + handle_z - base_handle_z,
            quat,
        )

    def execute_cartesian_open_path(self, waypoints, target_link=None):
        target_link = target_link or self.pose_target_link
        if target_link != self.group.get_end_effector_link():
            rospy.loginfo(
                "Skipping Cartesian path because compute_cartesian_path uses default end-effector %s, not %s",
                self.group.get_end_effector_link(),
                target_link,
            )
            return self.execute_waypoint_open_path(waypoints, target_link)

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
            return self.execute_waypoint_open_path(waypoints, target_link)
        return self.execute_timed_plan(trajectory, "Cartesian open path")

    def execute_waypoint_open_path(self, waypoints, target_link=None):
        for index, waypoint in enumerate(waypoints, start=1):
            if not self.go_to_pose(
                waypoint,
                "door open waypoint {}/{}".format(index, len(waypoints)),
                target_link,
            ):
                return False
        return True

    def yaw_rotated_current_pose(self, target_link, yaw_delta):
        current_pose = self.group.get_current_pose(target_link).pose
        q = current_pose.orientation
        current_q = [q.x, q.y, q.z, q.w]
        local_yaw_q = tft.quaternion_from_euler(0.0, 0.0, yaw_delta)
        rotated_q = tft.quaternion_multiply(current_q, local_yaw_q)
        rotated_q = tft.unit_vector(rotated_q)
        return make_pose(
            current_pose.position.x,
            current_pose.position.y,
            current_pose.position.z,
            Quaternion(rotated_q[0], rotated_q[1], rotated_q[2], rotated_q[3]),
        )

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
            self.set_orientation_constraint(quat)
            if not self.go_to_pose(approach_pose, "door handle approach"):
                return False
            if not self.go_to_pose(contact_pose, "door handle contact"):
                return False
            self.clear_orientation_constraint()

            self.remove_moving_door_collision()

            if self.attach():
                attached = True
            else:
                rospy.logwarn("Continuing even though attach returned false")

            open_target_link = self.args.open_target_link or self.args.robot_link
            rospy.loginfo("Opening with target link %s", open_target_link)
            open_base_pose = self.group.get_current_pose(open_target_link).pose
            open_quat = open_base_pose.orientation
            if abs(self.args.post_attach_yaw) > 1e-6:
                rotate_pose = self.yaw_rotated_current_pose(
                    open_target_link,
                    self.args.post_attach_yaw,
                )
                open_quat = rotate_pose.orientation
                if not self.go_to_pose(
                    rotate_pose,
                    "post-attach yaw rotation",
                    open_target_link,
                ):
                    return False
                rospy.sleep(self.args.post_attach_settle_time)
                open_base_pose = self.group.get_current_pose(open_target_link).pose

            self.set_orientation_constraint(open_quat, "open_orientation", open_target_link)
            start_angle = 0.0
            if self.args.pre_push_angle > 0.0:
                pre_push_pose = self.displaced_pose_at_angle(
                    open_base_pose,
                    start_angle,
                    self.args.pre_push_angle,
                    open_quat,
                )
                if not self.go_to_pose(pre_push_pose, "door handle pre-push", open_target_link):
                    return False
                start_angle = self.args.pre_push_angle
                open_base_pose = self.group.get_current_pose(open_target_link).pose

            waypoints = self.build_open_waypoints(open_quat, start_angle, open_base_pose)
            if not self.execute_cartesian_open_path(waypoints, open_target_link):
                return False

            rospy.loginfo("Door opening demo completed")
            completed = True
            return True
        finally:
            self.clear_orientation_constraint()
            if attached and self.args.detach_at_end:
                self.detach()
            if self.args.restore_collision:
                self.restore_moving_door_collision()


def main():
    parser = argparse.ArgumentParser(description="Open the Gazebo door with Kortex and gazebo_ros_link_attacher.")
    parser.add_argument("--robot-name", default="my_gen3")
    parser.add_argument("--robot-model", default="my_gen3")
    parser.add_argument("--robot-link", default="bracelet_link")
    parser.add_argument("--pose-target-link", default="tool_frame")
    parser.add_argument("--open-target-link", default="bracelet_link")
    parser.add_argument("--door-model", default="hinged_door_with_tag")
    parser.add_argument("--door-link", default="door_handle")
    parser.add_argument("--attach-service", default="/link_attacher_node/attach")
    parser.add_argument("--detach-service", default="/link_attacher_node/detach")
    parser.add_argument("--timeout", type=float, default=15.0)

    parser.add_argument("--hinge-x", type=float, default=0.55)
    parser.add_argument("--hinge-y", type=float, default=0.38)
    parser.add_argument("--hinge-z", type=float, default=0.45)
    parser.add_argument("--handle-local-x", type=float, default=-0.07)
    parser.add_argument("--handle-local-y", type=float, default=-0.58)
    parser.add_argument("--handle-local-z", type=float, default=0.05)

    parser.add_argument("--approach-offset-x", type=float, default=-0.05)
    parser.add_argument("--approach-offset-y", type=float, default=0.0)
    parser.add_argument("--approach-offset-z", type=float, default=0.0)
    parser.add_argument("--contact-offset-x", type=float, default=-0.01)
    parser.add_argument("--contact-offset-y", type=float, default=0.0)
    parser.add_argument("--contact-offset-z", type=float, default=0.0)
    parser.add_argument("--open-angle", type=float, default=0.20)
    parser.add_argument("--open-steps", type=int, default=1)
    parser.add_argument("--pre-push-angle", type=float, default=0.0)
    parser.add_argument("--eef-step", type=float, default=0.01)
    parser.add_argument("--jump-threshold", type=float, default=0.0)
    parser.add_argument("--cartesian-avoid-collisions", action="store_true")
    parser.add_argument("--min-path-fraction", type=float, default=0.8)

    parser.add_argument("--tool-roll", type=float, default=math.pi / 2.0)
    parser.add_argument("--tool-pitch", type=float, default=0.0)
    parser.add_argument("--tool-yaw", type=float, default=math.pi / 2.0)
    parser.add_argument("--post-attach-yaw", type=float, default=math.pi / 2.0)
    parser.add_argument("--post-attach-settle-time", type=float, default=0.5)
    parser.add_argument("--pre-detach-settle-time", type=float, default=1.0)
    parser.add_argument("--constrain-approach-orientation", action="store_true", default=True)
    parser.add_argument("--no-constrain-approach-orientation", action="store_false", dest="constrain_approach_orientation")
    parser.add_argument("--orientation-constraint-tolerance", type=float, default=0.15)
    parser.add_argument("--position-tolerance", type=float, default=0.02)
    parser.add_argument("--orientation-tolerance", type=float, default=0.2)
    parser.add_argument("--planning-time", type=float, default=10.0)
    parser.add_argument("--planning-attempts", type=int, default=10)
    parser.add_argument("--velocity-scaling", type=float, default=0.25)
    parser.add_argument("--acceleration-scaling", type=float, default=0.25)
    parser.add_argument("--min-waypoint-dt", type=float, default=0.02)
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

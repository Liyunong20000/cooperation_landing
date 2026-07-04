#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
import tf.transformations as tft
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def make_pose(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    pose = Pose()
    pose.position = Point(x, y, z)
    quat = tft.quaternion_from_euler(roll, pitch, yaw)
    pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
    return pose


def compose_pose(base, local):
    base_q = [
        base.orientation.x,
        base.orientation.y,
        base.orientation.z,
        base.orientation.w,
    ]
    local_q = [
        local.orientation.x,
        local.orientation.y,
        local.orientation.z,
        local.orientation.w,
    ]

    rotation = tft.quaternion_matrix(base_q)
    local_xyz = [local.position.x, local.position.y, local.position.z, 1.0]
    rotated = rotation.dot(local_xyz)

    pose = Pose()
    pose.position.x = base.position.x + rotated[0]
    pose.position.y = base.position.y + rotated[1]
    pose.position.z = base.position.z + rotated[2]

    composed_q = tft.quaternion_multiply(base_q, local_q)
    pose.orientation = Quaternion(composed_q[0], composed_q[1], composed_q[2], composed_q[3])
    return pose


def color(r, g, b, a=1.0):
    return ColorRGBA(r, g, b, a)


def scale(x, y, z):
    return Vector3(x, y, z)


OBJECT_SPECS = [
    {
        "name": "left_post",
        "marker_type": Marker.CUBE,
        "primitive_type": SolidPrimitive.BOX,
        "link": "frame",
        "local_pose": make_pose(0.0, -0.48, 0.0),
        "scale": scale(0.08, 0.08, 0.90),
        "color": color(0.35, 0.35, 0.35),
    },
    {
        "name": "right_post",
        "marker_type": Marker.CUBE,
        "primitive_type": SolidPrimitive.BOX,
        "link": "frame",
        "local_pose": make_pose(0.0, 0.48, 0.0),
        "scale": scale(0.08, 0.08, 0.90),
        "color": color(0.35, 0.35, 0.35),
    },
    {
        "name": "top_bar",
        "marker_type": Marker.CUBE,
        "primitive_type": SolidPrimitive.BOX,
        "link": "frame",
        "local_pose": make_pose(0.0, 0.0, 0.43),
        "scale": scale(0.08, 0.86, 0.08),
        "color": color(0.35, 0.35, 0.35),
    },
    {
        "name": "door_panel",
        "marker_type": Marker.CUBE,
        "primitive_type": SolidPrimitive.BOX,
        "link": "panel",
        "local_pose": make_pose(0.0, 0.35, 0.0),
        "scale": scale(0.04, 0.70, 0.80),
        "color": color(0.55, 0.36, 0.20),
    },
    {
        "name": "handle",
        "marker_type": Marker.CUBE,
        "primitive_type": SolidPrimitive.BOX,
        "link": "handle",
        "local_pose": make_pose(0.0, 0.0, 0.0),
        "scale": scale(0.06, 0.16, 0.04),
        "color": color(0.05, 0.05, 0.05),
    },
    {
        "name": "tag_plate",
        "marker_type": Marker.CUBE,
        "primitive_type": SolidPrimitive.BOX,
        "link": "panel",
        "local_pose": make_pose(-0.026, 0.35, 0.18, 0.0, math.pi / 2.0, 0.0),
        "scale": scale(0.002, 0.18, 0.18),
        "color": color(1.0, 1.0, 1.0),
    },
]


class DoorTagMarkerPublisher:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "hinged_door_with_tag")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.robot_name = rospy.get_param("~robot_name", "my_gen3").strip("/")
        self.publish_planning_scene = rospy.get_param("~publish_planning_scene", True)
        self.publish_rate = rospy.get_param("~publish_rate", 20.0)
        self.frame_link = self.model_name + "::door_frame"
        self.panel_link = self.model_name + "::door_panel"
        self.handle_link = self.model_name + "::door_handle"
        self.link_poses = {}

        self.default_frame_pose = make_pose(0.50, 0.03, 0.45)
        self.default_panel_pose = make_pose(0.50, 0.38, 0.45, 0.0, 0.0, math.pi)
        self.default_handle_pose = make_pose(0.43, -0.20, 0.50, 0.0, math.pi / 2.0, math.pi)

        self.pub = rospy.Publisher("/door_tag/markers", MarkerArray, queue_size=1)
        self.collision_pub = rospy.Publisher(
            "/{}/collision_object".format(self.robot_name),
            CollisionObject,
            queue_size=10,
        )
        self.sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.link_states_cb)

    def link_states_cb(self, msg):
        for name, pose in zip(msg.name, msg.pose):
            if name in (self.frame_link, self.panel_link, self.handle_link):
                self.link_poses[name] = pose

    def make_marker(self, marker_id, name, marker_type, pose, marker_scale, marker_color):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "door_tag"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale = marker_scale
        marker.color = marker_color
        marker.lifetime = rospy.Duration(0.2)
        marker.text = name
        return marker

    def build_markers(self):
        link_poses = self.current_link_poses()

        marker_array = MarkerArray()
        for marker_id, spec in enumerate(OBJECT_SPECS):
            base_pose = link_poses[spec["link"]]
            marker_array.markers.append(
                self.make_marker(
                    marker_id,
                    spec["name"],
                    spec["marker_type"],
                    compose_pose(base_pose, spec["local_pose"]),
                    spec["scale"],
                    spec["color"],
                )
            )

        return marker_array

    def current_link_poses(self):
        return {
            "frame": self.link_poses.get(self.frame_link, self.default_frame_pose),
            "panel": self.link_poses.get(self.panel_link, self.default_panel_pose),
            "handle": self.link_poses.get(self.handle_link, self.default_handle_pose),
        }

    def make_collision_object(self, spec, base_pose):
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.frame_id
        collision_object.header.stamp = rospy.Time.now()
        collision_object.id = "door_" + spec["name"]

        primitive = SolidPrimitive()
        primitive.type = spec["primitive_type"]
        if primitive.type == SolidPrimitive.BOX:
            primitive.dimensions = [spec["scale"].x, spec["scale"].y, spec["scale"].z]
        elif primitive.type == SolidPrimitive.CYLINDER:
            primitive.dimensions = [spec["scale"].z, spec["scale"].x / 2.0]

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(compose_pose(base_pose, spec["local_pose"]))
        collision_object.operation = CollisionObject.ADD
        return collision_object

    def publish_collision_objects(self):
        if not self.publish_planning_scene:
            return

        publish_moving_collision = rospy.get_param("/door_tag/publish_moving_collision", True)
        link_poses = self.current_link_poses()
        for spec in OBJECT_SPECS:
            if spec["link"] in ("panel", "handle") and not publish_moving_collision:
                continue
            self.collision_pub.publish(
                self.make_collision_object(spec, link_poses[spec["link"]])
            )

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.pub.publish(self.build_markers())
            self.publish_collision_objects()
            rate.sleep()


def main():
    rospy.init_node("door_tag_marker_publisher", anonymous=True)
    DoorTagMarkerPublisher().spin()


if __name__ == "__main__":
    main()

#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import numpy as np
import time
import tf.transformations as tft
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist

from dog_basic_function import DogBasic

# It is for ground robot used to align with aerial robot

class AprilmoveqilinNode:

    def __init__(self):  # This part will work when this node is used.
        print(f'Hi, I am Cloud Cube')
        rospy.init_node('Aprilmoveqilin', anonymous=True)

        self.dog_align_drone_matrix = []
        self.time_rece = rospy.Time()

        # Subscribe and publish.
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)

        self.pub_qilin_vel= rospy.Publisher('/go1/cmd_vel', Twist, queue_size=10)

        # Load the parameter
        self.drone_tags_matrix_param = rospy.get_param("/drone_tags_matrix")
        self.origin_2_camera_matrix_param = rospy.get_param("/camera_drone_matrix")
        self.drone_tags_matrix()
        self.converge_interval = rospy.get_param("/converge_interval")
        self.above_z = rospy.get_param("/above_z")
        self.move_param = rospy.get_param("/move_parameter")
        self.rotate_param = rospy.get_param("/rotate_parameter")
        # self.pose_parameter = rospy.get_param("/pose_parameter")

        self.msg_apriltag = None
        self.dog_basic_function = DogBasic()

    def _callback_apriltag(self, msg):
        self.msg_apriltag = msg
    def drone_tags_matrix(self):
        for entry in self.drone_tags_matrix_param:
            tag_id = entry.get("id")
            name = entry.get("name")
            matrix_raw = entry.get("matrix")

            # Convert matrix to list of 16 floats
            if isinstance(matrix_raw, list):
                matrix_flat = list(map(float, matrix_raw))  # in case any are string type

            else:
                print(f"Invalid matrix format for tag ID {tag_id}")
                continue

            # Convert to 4x4 NumPy matrix
            matrix_np = np.array(matrix_flat).reshape((4, 4))

            # Dynamically assign to self.tags_0_matrix, self.tags_1_matrix, ...
            attr_name = f"tags_{tag_id}_matrix"
            setattr(self, attr_name, matrix_np)

            # Use or print
            # print(f"Set self.{attr_name}:\n{matrix_np}")
            # print(f"ID: {tag_id}, Name: {name}")
            # print(matrix_np)

    def find_target_tag(self,data,target_id):
        T = 0
        # print(f'11111111')
        if len(data.detections) == 0:
            return None

        for det in data.detections:
            # print(f'{det}')
            # print(f'{det.id}')
            if target_id in det.id:
                pose = det.pose.pose.pose
                x = - pose.position.y
                y =  pose.position.x
                z = pose.position.z
                qx = pose.orientation.x
                qy = pose.orientation.y
                qz = pose.orientation.z
                qw = pose.orientation.w

                q = [qx, qy, qz, qw]
                t = [x, y, z]
                T = tft.quaternion_matrix(q)
                T[:3, 3] = t
                print(f'{T}')
                return T

    def average_with_svd(self, T_list):
        translations = []
        rotations = []

        for T in T_list:
            R = T[:3, :3]
            t = T[:3, 3]
            rotations.append(R)
            translations.append(t)

        t_avg = np.mean(translations, axis=0)

        R_avg_raw = np.mean(rotations, axis=0)
        U, _, Vt = np.linalg.svd(R_avg_raw)
        R_avg = U @ Vt

        if np.linalg.det(R_avg) < 0:
            U[:, -1] *= -1
            R_avg = U @ Vt

        T_avg = np.eye(4)
        T_avg[:3, :3] = R_avg
        T_avg[:3, 3] = t_avg

        return T_avg

    def find_drone_center(self, data):
        drone_world_matrices = []

        for tag_id in range(8):  # IDs 0 to 7
            T_camera_2_tag = self.find_target_tag(data, tag_id)
            # Check if tag detected
            if not isinstance(T_camera_2_tag, np.ndarray) or T_camera_2_tag.shape != (4, 4):
                continue

            # Get static transform from tag to drone center
            attr_name = f"tags_{tag_id}_matrix"
            T_tag_2_drone = getattr(self, attr_name, None)

            if not isinstance(T_tag_2_drone, np.ndarray) or T_tag_2_drone.shape != (4, 4):
                print(f"No static transform for tag {tag_id}")
                continue

            # Compute drone pose in world frame
            T_camera_2_drone = T_camera_2_tag @ T_tag_2_drone
            drone_world_matrices.append(T_camera_2_drone)

        if not drone_world_matrices:
            print("No valid drone tag detections")
            return None
        print(f'{self.average_with_svd(drone_world_matrices)}')
        return self.average_with_svd(drone_world_matrices)

    def align_dog_with_drone(self):
        self.dog_align_drone_matrix =  self.origin_2_camera_matrix_param @ self.find_drone_center(self.msg_apriltag)
        lx = np.clip((self.move_param * self.dog_align_drone_matrix[3, 0]), -2, 2)
        ly = np.clip((self.move_param * self.dog_align_drone_matrix[3, 1]), -2, 2)
        q = tft.quaternion_from_matrix(self.dog_align_drone_matrix)
        roll, pitch, yaw = tft.euler_from_quaternion(q)
        ryaw = np.clip((self.rotate_param * yaw), -0.3, 0.3)

        self.dog_basic_function.qilin_cmd_vel(lx, ly, 0, 0, ryaw)


if __name__ == '__main__':
    node = AprilmoveqilinNode()
    # node.stand()
    time.sleep(1)
    # print(type(node.drone_tags_matrix_param))
    # node.drone_tags_matrix()
    while not rospy.is_shutdown():
        node.find_drone_center(node.msg_apriltag)
    # while not rospy.is_shutdown():
    #
    #     rospy.spin()

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

        self.dog_align_drone_matrix = []
        self.time_rece = rospy.Time()
        self.last_tag_time = rospy.Time(0)

        self.smoothed_lx = 0
        self.smoothed_ly = 0
        self.smoothed_ryaw = 0
        self.smooth_alpha = 0.5
        
        # Subscribe and publish.
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)

        self.pub_qilin_vel= rospy.Publisher('/go1/cmd_vel', Twist, queue_size=10)

        # Load the parameter
        self.drone_tags_matrix_param = rospy.get_param("/drone_tags_matrix")
        self.origin_2_camera_matrix_param = np.array(rospy.get_param("/camera_drone_matrix")).reshape((4, 4))
        self.drone_tags_matrix()
        # self.landing_distance_threshold = rospy.get_param("/landing_info/landing_distance_threshold")
        # self.landing_angle_threshold = rospy.get_param("/landing_info/landing_angle_threshold") 
        # self.above_z = rospy.get_param("/above_z")
        self.move_param = rospy.get_param("/move_parameter",1)
        self.rotate_param = rospy.get_param("/rotate_parameter",0.5)
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
        # if len(data.detections) == 0:
        #     return
        
        for det in data.detections:
            # print(f'{det}')
            # print(f'{det.id}')
            if target_id in det.id:
                pose = det.pose.pose.pose
                x =  pose.position.x
                y =  pose.position.y
                z = pose.position.z
                qx = pose.orientation.x
                qy = pose.orientation.y
                qz = pose.orientation.z
                qw = pose.orientation.w

                q = [qx, qy, qz, qw]
                t = [x, y, z]
                T = tft.quaternion_matrix(q)
                T[:3, 3] = t
                # print(f'{T}')
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

    def filter_and_average_tags(self, T_list, sigma=1.0):
        if not T_list:
            return None

        translations = np.array([T[:3, 3] for T in T_list])
        rotations = [T[:3, :3] for T in T_list]

        t_mean = np.mean(translations, axis=0)
        dists = np.linalg.norm(translations - t_mean, axis=1)

        std = np.std(dists)
        keep_indices = np.where(dists < sigma * std)[0]

        if len(keep_indices) == 0:
            rospy.logwarn("All tag transforms are outliers, fallback to full average.")
            return self.average_with_svd(T_list)

        filtered_T = [T_list[i] for i in keep_indices]
        return self.average_with_svd(filtered_T)

    def find_drone_center(self, data):
        drone_world_matrices = []

        for tag_id in range(1):  # IDs 0 to 7
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
            # print(f'{drone_world_matrices}')
        if not drone_world_matrices:
            print("No valid drone tag detections")
            return None
        # print(f'{self.average_with_svd(drone_world_matrices)}')
        # return self.average_with_svd(drone_world_matrices)
        # T = self.filter_and_average_tags(drone_world_matrices, sigma = 1.0)
        # print(f'{T}')
        # return self.filter_and_average_tags(drone_world_matrices, sigma=1.0)
        return T_camera_2_drone
    
    def align_dog_with_drone(self):
        if self.msg_apriltag is None:
            rospy.logwarn("No AprilTag message yet.")
            return

        T_drone_center = self.find_drone_center(self.msg_apriltag)
        
        if T_drone_center is None:
        # Check if it's been too long since last detection
            if (rospy.Time.now() - self.last_tag_time) > rospy.Duration(0.5):
                rospy.logwarn("Tag lost for >0.5s. Stopping dog.")
                self.dog_basic_function.qilin_cmd_vel(0, 0, 0, 0, 0)
            return


        if not isinstance(T_drone_center, np.ndarray) or T_drone_center.shape != (4, 4):
            rospy.logwarn("Tag 0 not found or invalid transform.")
            return
      
        self.last_tag_time = rospy.Time.now()
        self.dog_align_drone_matrix = self.origin_2_camera_matrix_param @ T_drone_center
        # self.dog_align_drone_matrix = self.origin_2_camera_matrix_param @ self.find_target_tag(self.msg_apriltag, 0)
        # print(f'{self.dog_align_drone_matrix}')
        if (self.dog_align_drone_matrix[0, 3] < 0.5) or (self.dog_align_drone_matrix[1, 3] < 0.5):
            self.move_param = 1.15
        else:
            self.move_param = 1
        lx = np.clip((self.move_param * self.dog_align_drone_matrix[0, 3]), -1, 1)
        ly = np.clip((self.move_param * self.dog_align_drone_matrix[1, 3]), -1, 1)
        # q = tft.quaternion_from_matrix(self.dog_align_drone_matrix)
        if not isinstance(self.find_target_tag(self.msg_apriltag, 0), np.ndarray) or self.find_target_tag(self.msg_apriltag, 0).shape != (4, 4):
            rospy.logwarn("Tag 0 not found or invalid transform.")
            return
      
        q = tft.quaternion_from_matrix(self.find_target_tag(self.msg_apriltag, 0))
        roll, pitch, yaw = tft.euler_from_quaternion(q)
        ryaw = np.clip((self.rotate_param * yaw), -0.3, 0.3)
        # print(f'{lx}, {ly}, {ryaw}')

        self.smoothed_lx = (1- self.smooth_alpha) * self.smoothed_lx + self.smooth_alpha * lx
        self.smoothed_ly = (1- self.smooth_alpha) * self.smoothed_ly + self.smooth_alpha * ly
        self.smoothed_ryaw = (1- self.smooth_alpha) * self.smoothed_ryaw + self.smooth_alpha * ryaw
        # print(f'{self.smoothed_lx},{self.smoothed_ly},{self.smoothed_ryaw}')
        # self.dog_basic_function.qilin_cmd_vel(self.smoothed_lx, self.smoothed_ly, 0, 0, self.smoothed_ryaw)
        self.dog_basic_function.qilin_cmd_vel(self.smoothed_lx, self.smoothed_ly, 0, 0, self.smoothed_ryaw)

if __name__ == '__main__':
    rospy.init_node('Aprilmoveqilin', anonymous=True)
    node = AprilmoveqilinNode()
    # node.stand()
    time.sleep(1)
    # print(type(node.drone_tags_matrix_param))
    # node.drone_tags_matrix()
    while not rospy.is_shutdown():
        time.sleep(0.1)
        node.align_dog_with_drone()
    # while not rospy.is_shutdown():
    #
    #     rospy.spin()

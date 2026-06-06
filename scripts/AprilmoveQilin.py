#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import time
import tf.transformations as tft
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist

from dog_basic_function import DogBasic


def clamp(x, lo, hi):
    """Clamp x to [lo, hi]."""
    return max(lo, min(hi, x))


def p_with_deadzone(err, k, v_min, v_max, tol):
    """P control with tolerance dead-zone and velocity saturation."""
    # if abs(err) < tol:
    #     return 0.0
    v = k * err
    if v == 0:
        return 0.0
    if abs(v) < v_min:
        v = v_min if v > 0 else -v_min
    return clamp(v, -v_max, v_max)

# It is for ground robot used to align with aerial robot

class AprilmoveqilinNode:
    """
    Node for ground robot to align with aerial robot using AprilTag detections.
    """

    def __init__(self):  # This part will work when this node is used.
        print(f'Hi, I am Cloud Cube')

        self.dog_align_drone_matrix = []
        self.time_rece = rospy.Time()
        self.last_tag_time = rospy.Time(0)

        self.smoothed_lx = 0
        self.smoothed_ly = 0
        self.smoothed_ryaw = 0
        
        # Subscribe the tag_detection topic of dog camera.
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)
        # Pub the velocity command for dog
        self.pub_qilin_vel= rospy.Publisher('/go1/cmd_vel', Twist, queue_size=10)

        # Load the parameter for landing process
        self.drone_tags_matrix_param = rospy.get_param("/drone_tags_matrix")
        self.origin_2_camera_matrix_param = np.array(rospy.get_param("/camera_drone_matrix")).reshape((4, 4))
        self.drone_tags_matrix()
        # self.landing_distance_threshold = rospy.get_param("/landing_info/landing_distance_threshold")
        # self.landing_angle_threshold = rospy.get_param("/landing_info/landing_angle_threshold") 
        # self.above_z = rospy.get_param("/above_z")
        self.move_param = rospy.get_param("/move_parameter",1.5)
        self.rotate_param = rospy.get_param("/rotate_parameter",0.5)
        self.smooth_alpha = rospy.get_param("/smooth_alpha", 0.5)
        self.close_distance_threshold = rospy.get_param("/close_distance_threshold", 0.5)
        self.fast_move_param = rospy.get_param("/fast_move_param", 1.15)
        self.normal_move_param = rospy.get_param("/normal_move_param", 1.0)
        self.max_linear_vel = rospy.get_param("/max_linear_vel", 0.25)
        self.max_angular_vel = rospy.get_param("/max_angular_vel", 0.3)
        self.min_linear_vel = rospy.get_param("/min_linear_vel", 0.025)
        self.min_angular_vel = rospy.get_param("/min_angular_vel", 0.05)
        # self.pose_parameter = rospy.get_param("/pose_parameter")

        self.msg_apriltag = None
        self.dog_basic_function = DogBasic()

    def _callback_apriltag(self, msg):
        self.msg_apriltag = msg

    # Get the RT matrix of each tags from drone center
    def drone_tags_matrix(self):
        """
        Load and set tag matrices from ROS parameters.
        """
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

    # The function for getting target tag info
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
                x = pose.position.x
                y = pose.position.y
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

    # Avage the position and follow the orientation of tag 0 or tag 1
    def find_drone_center(self, data, yaw_source_id=0, max_pair_gap=0.25):

        # Calculate the rotation metrix from drone center --> tag --> camera of dog
        def cam_to_drone(tag_id):
            T_cam_tag = self.find_target_tag(data, tag_id)
            if not (isinstance(T_cam_tag, np.ndarray) and T_cam_tag.shape == (4, 4)):
                return None
            T_tag_drone = getattr(self, f"tags_{tag_id}_matrix", None)
            if not (isinstance(T_tag_drone, np.ndarray) and T_tag_drone.shape == (4, 4)):
                return None
            return T_cam_tag @ T_tag_drone

        # Get the metrix of tag 0 and tag 1
        T0 = cam_to_drone(0)
        T1 = cam_to_drone(1)

        if T0 is None and T1 is None:
            return None

        if T0 is None:
            return T1
        if T1 is None:
            return T0

        p0 = T0[:3, 3]
        p1 = T1[:3, 3]

        if np.linalg.norm(p0 - p1) > max_pair_gap:
            choose_T = T0 if np.linalg.norm(p0) < np.linalg.norm(p1) else T1
            return choose_T

        p_avg = 0.5 * (p0 + p1)

        if yaw_source_id == 0 and T0 is not None:
            R = T0[:3, :3]
        elif yaw_source_id == 1 and T1 is not None:
            R = T1[:3, :3]
        else:
            R = T0[:3, :3]

        T_camera_2_drone = np.eye(4)
        T_camera_2_drone[:3, :3] = R
        T_camera_2_drone[:3, 3] = p_avg
        return T_camera_2_drone
    def align_dog_with_drone(self):
        if self.msg_apriltag is None:
            rospy.logwarn("No AprilTag message yet.")
            return
        # Include the topic of apriltag detection and find the center
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
        if (self.dog_align_drone_matrix[0, 3] > self.close_distance_threshold) or (self.dog_align_drone_matrix[1, 3] > self.close_distance_threshold):
            self.move_param = self.fast_move_param
        else:
            self.move_param = self.normal_move_param
        lx = p_with_deadzone(self.dog_align_drone_matrix[0, 3], self.move_param, self.min_linear_vel, self.max_linear_vel, 0.0)
        ly = p_with_deadzone(self.dog_align_drone_matrix[1, 3], self.move_param, self.min_linear_vel, self.max_linear_vel, 0.0)
        # q = tft.quaternion_from_matrix(self.dog_align_drone_matrix)
        T_tag_0 = self.find_target_tag(self.msg_apriltag, 0)
        if not isinstance(T_tag_0, np.ndarray) or T_tag_0.shape != (4, 4):
            rospy.logwarn("Tag 0 not found or invalid transform.")
            return
      
        q = tft.quaternion_from_matrix(T_tag_0)
        roll, pitch, yaw = tft.euler_from_quaternion(q)
        ryaw = p_with_deadzone(yaw, self.rotate_param, self.min_angular_vel, self.max_angular_vel, 0.0)
        # print(f'{lx}, {ly}, {ryaw}')


        self.smoothed_lx = (1- self.smooth_alpha) * self.smoothed_lx + self.smooth_alpha * lx
        self.smoothed_ly = (1- self.smooth_alpha) * self.smoothed_ly + self.smooth_alpha * ly
        self.smoothed_ryaw = (1- self.smooth_alpha) * self.smoothed_ryaw + self.smooth_alpha * ryaw
        self.dog_basic_function.qilin_cmd_vel(self.smoothed_lx, self.smoothed_ly, 0, 0, self.smoothed_ryaw)

if __name__ == '__main__':
    rospy.init_node('Aprilmoveqilin', anonymous=True)
    node = AprilmoveqilinNode()
    # node.stand()
    time.sleep(1)
    # print(type(node.drone_tags_matrix_param))
    # node.drone_tags_matrix()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        node.align_dog_with_drone()
        rate.sleep()
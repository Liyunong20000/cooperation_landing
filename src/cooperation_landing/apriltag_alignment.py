"""Apriltag alignment implementation."""

import numpy as np
import rospy
import tf.transformations as tft
from apriltag_ros.msg import AprilTagDetectionArray

from cooperation_landing.control_utils import clamp, p_with_deadzone, smooth_with_min_velocity
from cooperation_landing.dog_basic_function import DogBasic


class AprilmoveqilinNode:
    """
    Node for ground robot to align with aerial robot using AprilTag detections.
    """

    def __init__(self):
        rospy.logdebug('Initializing AprilTag ground-alignment controller.')

        self.dog_align_drone_matrix = []
        self.time_rece = rospy.Time()
        self.last_tag_time = rospy.Time.now()

        self.smoothed_lx = 0
        self.smoothed_ly = 0
        self.smoothed_ryaw = 0

        ground_robot_ns = ('/' + str(rospy.get_param('~ground_robot_ns', 'go1')).strip('/')).rstrip(
            '/'
        )
        tag_topic = rospy.get_param('~ground_tag_topic', ground_robot_ns + '/tag_detections')

        # Load the parameter for landing process
        self.drone_tags_matrix_param = rospy.get_param(
            '~drone_tags_matrix', rospy.get_param('/drone_tags_matrix', [])
        )
        camera_matrix = rospy.get_param(
            '~camera_drone_matrix', rospy.get_param('/camera_drone_matrix', [])
        )
        if not isinstance(camera_matrix, list) or len(camera_matrix) != 16:
            raise ValueError('camera_drone_matrix must contain exactly 16 values')
        self.origin_2_camera_matrix_param = np.asarray(camera_matrix, dtype=float).reshape((4, 4))
        self.drone_tags_matrix()
        # self.landing_distance_threshold = rospy.get_param("/landing_info/landing_distance_threshold")
        # self.landing_angle_threshold = rospy.get_param("/landing_info/landing_angle_threshold")
        # self.above_z = rospy.get_param("/above_z")
        self.move_param = self._param('move_parameter', 1.5)
        self.rotate_param = self._param('rotate_parameter', 0.5)
        self.smooth_alpha = clamp(float(self._param('smooth_alpha', 0.5)), 0.0, 1.0)
        self.close_distance_threshold = max(
            0.0, float(self._param('close_distance_threshold', 0.5))
        )
        self.stop_distance_threshold = max(0.0, float(self._param('stop_distance_threshold', 0.02)))
        self.fast_move_param = self._param('fast_move_param', 1.15)
        self.normal_move_param = self._param('normal_move_param', 1.0)
        self.min_linear_vel = max(0.0, float(self._param('min_linear_vel', 0.025)))
        self.min_angular_vel = max(0.0, float(self._param('min_angular_vel', 0.05)))
        self.max_linear_vel = max(self.min_linear_vel, float(self._param('max_linear_vel', 0.25)))
        self.max_angular_vel = max(self.min_angular_vel, float(self._param('max_angular_vel', 0.3)))
        # self.pose_parameter = rospy.get_param("/pose_parameter")

        self.msg_apriltag = None
        self.dog_basic_function = DogBasic()

        # Callbacks may run as soon as a subscription is registered.
        self._apriltag_sub = rospy.Subscriber(
            tag_topic, AprilTagDetectionArray, self._callback_apriltag, queue_size=1
        )

    @staticmethod
    def _param(name, default):
        """Read a private parameter while accepting the historical root name."""
        return rospy.get_param(f'~{name}', rospy.get_param(f'/{name}', default))

    def _callback_apriltag(self, msg):
        self.msg_apriltag = msg

    # Get the RT matrix of each tags from drone center
    def drone_tags_matrix(self):
        """
        Load and set tag matrices from ROS parameters.
        """
        if not isinstance(self.drone_tags_matrix_param, list):
            raise ValueError('drone_tags_matrix must be a list')
        for entry in self.drone_tags_matrix_param:
            if not isinstance(entry, dict):
                rospy.logwarn('Ignoring non-dictionary drone tag entry: %r', entry)
                continue
            tag_id = entry.get('id')
            matrix_raw = entry.get('matrix')

            # Convert matrix to list of 16 floats
            if isinstance(matrix_raw, list) and len(matrix_raw) == 16:
                matrix_flat = list(map(float, matrix_raw))  # in case any are string type

            else:
                rospy.logwarn('Invalid 4x4 matrix for tag ID %s; entry ignored.', tag_id)
                continue

            # Convert to 4x4 NumPy matrix
            matrix_np = np.array(matrix_flat).reshape((4, 4))

            # Dynamically assign to self.tags_0_matrix, self.tags_1_matrix, ...
            attr_name = f'tags_{tag_id}_matrix'
            setattr(self, attr_name, matrix_np)

    # The function for getting target tag info
    def find_target_tag(self, data, target_id):
        if data is None:
            return None
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
        return None

    # Avage the position and follow the orientation of tag 0 or tag 1
    def find_drone_center(self, data, yaw_source_id=0, max_pair_gap=0.25):

        # Calculate the rotation metrix from drone center --> tag --> camera of dog
        def cam_to_drone(tag_id):
            T_cam_tag = self.find_target_tag(data, tag_id)
            if not (isinstance(T_cam_tag, np.ndarray) and T_cam_tag.shape == (4, 4)):
                return None
            T_tag_drone = getattr(self, f'tags_{tag_id}_matrix', None)
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
            rospy.logwarn('No AprilTag message yet.')
            return
        # Include the topic of apriltag detection and find the center
        T_drone_center = self.find_drone_center(self.msg_apriltag)

        if T_drone_center is None:
            # Check if it's been too long since last detection
            if (rospy.Time.now() - self.last_tag_time) > rospy.Duration(0.5):
                rospy.logwarn('Tag lost for >0.5s. Stopping dog.')
                self.dog_basic_function.qilin_cmd_vel(0, 0, 0, 0, 0)
            return

        if not isinstance(T_drone_center, np.ndarray) or T_drone_center.shape != (4, 4):
            rospy.logwarn('Tag 0 not found or invalid transform.')
            return

        self.last_tag_time = rospy.Time.now()
        self.dog_align_drone_matrix = self.origin_2_camera_matrix_param @ T_drone_center
        # self.dog_align_drone_matrix = self.origin_2_camera_matrix_param @ self.find_target_tag(self.msg_apriltag, 0)
        # print(f'{self.dog_align_drone_matrix}')
        x_error = self.dog_align_drone_matrix[0, 3]
        y_error = self.dog_align_drone_matrix[1, 3]
        dist = np.linalg.norm([x_error, y_error])

        if (abs(x_error) > self.close_distance_threshold) or (
            abs(y_error) > self.close_distance_threshold
        ):
            self.move_param = self.fast_move_param
        else:
            self.move_param = self.normal_move_param

        if dist < self.stop_distance_threshold:
            lx = 0.0
            ly = 0.0
        else:
            lx = p_with_deadzone(
                x_error, self.move_param, self.min_linear_vel, self.max_linear_vel, 0.0
            )
            ly = p_with_deadzone(
                y_error, self.move_param, self.min_linear_vel, self.max_linear_vel, 0.0
            )
        # q = tft.quaternion_from_matrix(self.dog_align_drone_matrix)
        T_tag_0 = self.find_target_tag(self.msg_apriltag, 0)
        if not isinstance(T_tag_0, np.ndarray) or T_tag_0.shape != (4, 4):
            rospy.logwarn('Tag 0 not found or invalid transform.')
            self.dog_basic_function.qilin_cmd_vel(0, 0, 0, 0, 0)
            return

        q = tft.quaternion_from_matrix(T_tag_0)
        _, _, yaw = tft.euler_from_quaternion(q)
        ryaw = p_with_deadzone(
            yaw, self.rotate_param, self.min_angular_vel, self.max_angular_vel, 0.0
        )
        # print(f'{lx}, {ly}, {ryaw}')

        self.smoothed_lx = smooth_with_min_velocity(
            self.smoothed_lx,
            lx,
            self.smooth_alpha,
            self.min_linear_vel,
            self.max_linear_vel,
        )
        self.smoothed_ly = smooth_with_min_velocity(
            self.smoothed_ly,
            ly,
            self.smooth_alpha,
            self.min_linear_vel,
            self.max_linear_vel,
        )
        self.smoothed_ryaw = (1 - self.smooth_alpha) * self.smoothed_ryaw + self.smooth_alpha * ryaw
        self.dog_basic_function.qilin_cmd_vel(
            self.smoothed_lx, self.smoothed_ly, 0, 0, self.smoothed_ryaw
        )


def main():
    """Run the ROS node."""
    rospy.init_node('Aprilmoveqilin', anonymous=True)
    node = AprilmoveqilinNode()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        node.align_dog_with_drone()
        rate.sleep()

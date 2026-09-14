"""Apriltag alignment implementation."""

import threading
import time

import numpy as np
import rospy
import tf.transformations as tft
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool, Float64, Int32

from cooperation_landing.dog_basic_function import DogBasic


class AprilmoveqilinNode:
    """
    Node for ground robot to align with aerial robot using AprilTag detections.
    """

    def __init__(self):
        rospy.logdebug('Initializing AprilTag ground-alignment controller.')

        self.dog_align_drone_matrix = []
        self.last_apriltag_receive_time = None
        self.last_tag_time = rospy.Time.now()
        self._apriltag_lock = threading.Lock()
        self.apriltag_message_counter = 0
        self.last_processed_message_counter = 0
        self.last_valid_detection_time = None

        self.xy_aligned = False
        self.yaw_aligned = False
        self.previous_drone_center_position = None
        self.last_valid_lx = 0.0
        self.last_valid_ly = 0.0
        self.last_valid_ryaw = 0.0
        self.command_hold_active = False

        # Camera namespace is independent of the /go1 motion interface.
        tag_topic = rospy.get_param('~ground_tag_topic', '/qilin/tag_detections')

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
        self.move_param = self._param('move_parameter', 1.10)
        self.rotate_param = self._param('rotate_parameter', 0.3)
        self.align_enter_distance = float(self._param('align_enter_distance', 0.025))
        self.align_exit_distance = float(self._param('align_exit_distance', 0.035))
        if not 0.0 <= self.align_enter_distance < self.align_exit_distance:
            raise ValueError('Require 0 <= align_enter_distance < align_exit_distance')
        self.yaw_enter_threshold = float(self._param('yaw_enter_threshold', 0.03))
        self.yaw_exit_threshold = float(self._param('yaw_exit_threshold', 0.05))
        if not 0.0 <= self.yaw_enter_threshold < self.yaw_exit_threshold:
            raise ValueError('Require 0 <= yaw_enter_threshold < yaw_exit_threshold')
        self.max_pair_gap = max(0.0, float(self._param('max_pair_gap', 0.06)))
        self.apriltag_message_timeout = max(0.0, float(self._param('apriltag_message_timeout', 0.5)))
        self.command_hold_timeout = float(self._param('command_hold_timeout', 0.35))
        self.tag_loss_timeout = float(self._param('tag_loss_timeout', 0.5))
        if not 0.0 <= self.command_hold_timeout < self.tag_loss_timeout:
            raise ValueError('Require 0 <= command_hold_timeout < tag_loss_timeout')
        self.min_linear_vel = max(0.0, float(self._param('min_linear_vel', 0.03)))
        self.min_angular_vel = max(0.0, float(self._param('min_angular_vel', 0.05)))
        self.max_linear_vel = max(self.min_linear_vel, float(self._param('max_linear_vel', 0.25)))
        self.max_angular_vel = max(self.min_angular_vel, float(self._param('max_angular_vel', 0.3)))
        # self.pose_parameter = rospy.get_param("/pose_parameter")

        self.msg_apriltag = None
        self.dog_basic_function = DogBasic()

        debug_ns = '/go1/alignment_debug/'
        self.debug_error_pub = rospy.Publisher(debug_ns + 'error', Vector3Stamped, queue_size=10)
        self.debug_distance_pub = rospy.Publisher(debug_ns + 'distance', Float64, queue_size=10)
        self.debug_tag0_pub = rospy.Publisher(debug_ns + 'tag0_visible', Bool, queue_size=10)
        self.debug_tag1_pub = rospy.Publisher(debug_ns + 'tag1_visible', Bool, queue_size=10)
        self.debug_xy_aligned_pub = rospy.Publisher(debug_ns + 'xy_aligned', Bool, queue_size=10)
        self.debug_yaw_aligned_pub = rospy.Publisher(debug_ns + 'yaw_aligned', Bool, queue_size=10)
        self.debug_lost_duration_pub = rospy.Publisher(debug_ns + 'lost_duration', Float64, queue_size=10)
        self.debug_command_hold_pub = rospy.Publisher(debug_ns + 'command_hold_active', Bool, queue_size=10)
        self.debug_yaw_source_pub = rospy.Publisher(debug_ns + 'yaw_source', Int32, queue_size=10)
        self.debug_cmd_pub = rospy.Publisher(debug_ns + 'cmd', Vector3Stamped, queue_size=10)

        # Callbacks may run as soon as a subscription is registered.
        self._apriltag_sub = rospy.Subscriber(
            tag_topic, AprilTagDetectionArray, self._callback_apriltag, queue_size=1
        )

    @staticmethod
    def _param(name, default):
        """Read a private parameter while accepting the historical root name."""
        return rospy.get_param(f'~{name}', rospy.get_param(f'/{name}', default))

    def _callback_apriltag(self, msg):
        # Keep message, arrival time and sequence consistent across callback/control threads.
        with self._apriltag_lock:
            self.msg_apriltag = msg
            self.last_apriltag_receive_time = time.monotonic()
            self.apriltag_message_counter += 1

    def has_fresh_apriltag(self):
        """Check message arrival age, independently of the ROS clock."""
        with self._apriltag_lock:
            return (
                self.last_apriltag_receive_time is not None
                and time.monotonic() - self.last_apriltag_receive_time < self.apriltag_message_timeout
            )

    def _stop_alignment(self):
        """Stop all commanded axes and discard the previous alignment result."""
        self.xy_aligned = False
        self.yaw_aligned = False
        self.previous_drone_center_position = None
        self.dog_align_drone_matrix = None
        self.last_valid_lx = 0.0
        self.last_valid_ly = 0.0
        self.last_valid_ryaw = 0.0
        self.last_valid_detection_time = None
        self.command_hold_active = False
        self._send_command(0.0, 0.0, 0.0)

    def _handle_detection_loss(self, now):
        """Hold, stop without resetting, or reset using the last valid observation age."""
        self.command_hold_active = False
        if self.last_valid_detection_time is None:
            self._stop_alignment()
            return
        lost_duration = now - self.last_valid_detection_time
        if lost_duration > self.tag_loss_timeout:
            rospy.logwarn_throttle(
                2.0, 'No new valid tag observation for >%.3fs. Resetting alignment.',
                self.tag_loss_timeout
            )
            self._stop_alignment()
        elif lost_duration <= self.command_hold_timeout:
            self.command_hold_active = True
            self._send_command(self.last_valid_lx, self.last_valid_ly, self.last_valid_ryaw)
        else:
            # Preserve alignment, continuity and last-valid command through brief loss.
            self._send_command(0.0, 0.0, 0.0)

    def _send_command(self, lx, ly, ryaw):
        """Record exactly the command passed to DogBasic, including stops."""
        cmd = Vector3Stamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.vector.x, cmd.vector.y, cmd.vector.z = lx, ly, ryaw
        self.debug_cmd_pub.publish(cmd)
        self.dog_basic_function.qilin_cmd_vel(lx, ly, 0, 0, ryaw)

    def _publish_debug(self, tag0_visible=False, tag1_visible=False, yaw_source=-1,
                       x_error=float('nan'), y_error=float('nan'), yaw_error=float('nan')):
        """Publish each cycle; NaN marks unavailable errors rather than false zeros."""
        error = Vector3Stamped()
        error.header.stamp = rospy.Time.now()
        # Mixed units/frames: body-frame XY (m), raw detection-frame yaw (rad).
        # Leave frame_id empty: this diagnostic tuple is not a spatial vector.
        error.vector.x, error.vector.y, error.vector.z = x_error, y_error, yaw_error
        self.debug_error_pub.publish(error)
        self.debug_distance_pub.publish(Float64(data=np.hypot(x_error, y_error)))
        self.debug_tag0_pub.publish(Bool(data=tag0_visible))
        self.debug_tag1_pub.publish(Bool(data=tag1_visible))
        self.debug_xy_aligned_pub.publish(Bool(data=self.xy_aligned))
        self.debug_yaw_aligned_pub.publish(Bool(data=self.yaw_aligned))
        # NaN denotes no valid observation yet, or history cleared by a full reset.
        lost_duration = (float('nan') if self.last_valid_detection_time is None
                         else time.monotonic() - self.last_valid_detection_time)
        self.debug_lost_duration_pub.publish(Float64(data=lost_duration))
        self.debug_command_hold_pub.publish(Bool(data=self.command_hold_active))
        self.debug_yaw_source_pub.publish(Int32(data=yaw_source))

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
                if not np.all(np.isfinite(q + t)) or np.linalg.norm(q) == 0.0:
                    return None
                T = tft.quaternion_matrix(q)
                T[:3, 3] = t
                # print(f'{T}')
                return T
        return None

    # Average consistent positions and follow the orientation of tag 0 or tag 1.
    def find_drone_center(self, data, yaw_source_id=0, max_pair_gap=None):
        if max_pair_gap is None:
            max_pair_gap = self.max_pair_gap

        # Calculate the rotation metrix from drone center --> tag --> camera of dog
        def cam_to_drone(tag_id):
            T_cam_tag = self.find_target_tag(data, tag_id)
            if not (isinstance(T_cam_tag, np.ndarray) and T_cam_tag.shape == (4, 4)):
                return None
            T_tag_drone = getattr(self, f'tags_{tag_id}_matrix', None)
            if not (isinstance(T_tag_drone, np.ndarray) and T_tag_drone.shape == (4, 4)):
                return None
            estimate = T_cam_tag @ T_tag_drone
            return estimate if np.all(np.isfinite(estimate)) else None

        # Get the metrix of tag 0 and tag 1
        T0 = cam_to_drone(0)
        T1 = cam_to_drone(1)

        if T0 is None and T1 is None:
            return None

        if T0 is None:
            T_camera_2_drone = T1
        elif T1 is None:
            T_camera_2_drone = T0
        else:
            p0 = T0[:3, 3]
            p1 = T1[:3, 3]
            if np.linalg.norm(p0 - p1) > max_pair_gap:
                previous = self.previous_drone_center_position
                if previous is None:
                    rospy.logwarn_throttle(
                        2.0, 'Drone-center tag estimates disagree; no history, using Tag 0.'
                    )
                    T_camera_2_drone = T0
                else:
                    d0 = np.linalg.norm(p0 - previous)
                    d1 = np.linalg.norm(p1 - previous)
                    T_camera_2_drone = T0 if d0 <= d1 else T1
            else:
                T_camera_2_drone = np.eye(4)
                T_camera_2_drone[:3, :3] = (T1 if yaw_source_id == 1 else T0)[:3, :3]
                T_camera_2_drone[:3, 3] = 0.5 * (p0 + p1)

        # Keep an independent snapshot in the camera frame; no position filtering.
        self.previous_drone_center_position = T_camera_2_drone[:3, 3].copy()
        return T_camera_2_drone

    def align_dog_with_drone(self):
        with self._apriltag_lock:
            data = self.msg_apriltag
            receive_time = self.last_apriltag_receive_time
            new_message = self.apriltag_message_counter != self.last_processed_message_counter
            self.last_processed_message_counter = self.apriltag_message_counter
            now = time.monotonic()
        fresh = receive_time is not None and now - receive_time < self.apriltag_message_timeout
        tag0_visible = fresh and data is not None and any(0 in det.id for det in data.detections)
        tag1_visible = fresh and data is not None and any(1 in det.id for det in data.detections)
        if data is None:
            rospy.logwarn_throttle(2.0, 'No AprilTag message yet. Stopping dog.')
        elif not fresh:
            rospy.logwarn_throttle(
                2.0, 'AprilTag message stream timed out after %.3fs; applying detection-loss policy.',
                self.apriltag_message_timeout
            )
        if data is None or not fresh or not new_message:
            # A cached message may describe visibility, but cannot refresh control history.
            self._handle_detection_loss(now)
            self._publish_debug(tag0_visible, tag1_visible)
            return

        T_drone_center = self.find_drone_center(data)
        if (not isinstance(T_drone_center, np.ndarray) or T_drone_center.shape != (4, 4)
                or not np.all(np.isfinite(T_drone_center))):
            self._handle_detection_loss(now)
            self._publish_debug(tag0_visible, tag1_visible)
            return

        # Use one fresh message for visibility, center estimation and raw yaw.
        T_yaw_tag = self.find_target_tag(data, 0)
        yaw_source = 0
        if T_yaw_tag is None:
            T_yaw_tag = self.find_target_tag(data, 1)
            yaw_source = 1
        yaw_error = float('nan')
        ryaw = 0.0
        if T_yaw_tag is not None:
            # Preserve the raw AprilTag yaw frame and sign, independently of XY.
            q = tft.quaternion_from_matrix(T_yaw_tag)
            _, _, yaw_error = tft.euler_from_quaternion(q)
            abs_yaw_error = abs(yaw_error)
            if self.yaw_aligned:
                if abs_yaw_error >= self.yaw_exit_threshold:
                    self.yaw_aligned = False
            else:
                if abs_yaw_error <= self.yaw_enter_threshold:
                    self.yaw_aligned = True
            if not self.yaw_aligned:
                ryaw_raw = self.rotate_param * yaw_error
                ryaw = ryaw_raw
                if 0.0 < abs(ryaw) < self.min_angular_vel:
                    ryaw = np.sign(ryaw) * self.min_angular_vel
                if abs(ryaw) > self.max_angular_vel:
                    ryaw = np.sign(ryaw) * self.max_angular_vel
        else:
            yaw_source = -1

        self.last_tag_time = rospy.Time.now()
        self.dog_align_drone_matrix = self.origin_2_camera_matrix_param @ T_drone_center
        x_error = self.dog_align_drone_matrix[0, 3]
        y_error = self.dog_align_drone_matrix[1, 3]
        dist = np.hypot(x_error, y_error)

        if self.xy_aligned:
            if dist >= self.align_exit_distance:
                self.xy_aligned = False
        else:
            if dist <= self.align_enter_distance:
                self.xy_aligned = True

        if self.xy_aligned:
            lx = 0.0
            ly = 0.0
        else:
            vx_raw = self.move_param * x_error
            vy_raw = self.move_param * y_error
            lx, ly = vx_raw, vy_raw
            speed = np.hypot(lx, ly)
            if 0.0 < speed < self.min_linear_vel:
                scale = self.min_linear_vel / speed
                lx *= scale
                ly *= scale
            speed = np.hypot(lx, ly)
            if speed > self.max_linear_vel:
                scale = self.max_linear_vel / speed
                lx *= scale
                ly *= scale

        # Only a newly received, usable observation can renew the timer and stored command.
        self.last_valid_detection_time = now
        self.last_valid_lx = lx
        self.last_valid_ly = ly
        self.last_valid_ryaw = ryaw
        self.command_hold_active = False
        self._publish_debug(tag0_visible, tag1_visible, yaw_source, x_error, y_error, yaw_error)
        self._send_command(lx, ly, ryaw)


def main():
    """Run the ROS node."""
    rospy.init_node('Aprilmoveqilin', anonymous=True)
    node = AprilmoveqilinNode()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        node.align_dog_with_drone()
        rate.sleep()

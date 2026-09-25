"""Measurement-only AprilTag estimator: UAV center expressed in the dog frame.

The geometry intentionally follows AprilmoveqilinNode.find_drone_center without
importing/initializing its ground-motion interface or changing the fallback.
"""

import threading

import numpy as np
import rospy
import tf.transformations as tft
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped


def rigid_matrix(values, name):
    matrix = np.asarray(values, dtype=float)
    if matrix.size != 16:
        raise ValueError(name + " must contain 16 values")
    matrix = matrix.reshape(4, 4)
    rotation = matrix[:3, :3]
    if (
        not np.all(np.isfinite(matrix))
        or not np.allclose(matrix[3], [0, 0, 0, 1])
        or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
        or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5)
    ):
        raise ValueError(name + " must be a finite rigid transform")
    return matrix


def valid_stamp(stamp):
    try:
        return np.isfinite(stamp.to_sec()) and stamp.to_sec() > 0
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False


class AprilTagRelativePose:
    def __init__(self):
        self.robot_ns = (
            "/" + str(rospy.get_param("~robot_ns", "xuanwu")).strip("/")
        ).rstrip("/")
        self.frame_id = rospy.get_param("~docking_frame", "qilin_docking_frame")
        self.camera_matrix = rigid_matrix(
            rospy.get_param(
                "~camera_drone_matrix", rospy.get_param("/camera_drone_matrix", [])
            ),
            "camera_drone_matrix",
        )
        entries = rospy.get_param(
            "~drone_tags_matrix", rospy.get_param("/drone_tags_matrix", [])
        )
        self.tag_matrices = {}
        for entry in entries:
            if isinstance(entry, dict) and entry.get("id") in (0, 1):
                self.tag_matrices[entry["id"]] = rigid_matrix(
                    entry.get("matrix", []), "drone_tags_matrix"
                )
        if not self.tag_matrices:
            raise ValueError("drone_tags_matrix must configure tag 0 or tag 1")
        raw_pair_gap: object = rospy.get_param("~max_pair_gap", 0.06)
        if isinstance(raw_pair_gap, bool) or not isinstance(
            raw_pair_gap, (int, float, str)
        ):
            raise ValueError("max_pair_gap must be numeric")
        self.max_pair_gap = float(raw_pair_gap)
        if not np.isfinite(self.max_pair_gap) or self.max_pair_gap < 0:
            raise ValueError("max_pair_gap must be finite and nonnegative")
        self.previous_center = None  # Camera frame, matching the fallback.
        self._lock = threading.Lock()
        self.publisher = rospy.Publisher(
            self.robot_ns + "/visual_landing/info", PoseStamped, queue_size=1
        )
        self.subscriber = rospy.Subscriber(
            rospy.get_param("~ground_tag_topic", "/qilin/tag_detections"),
            AprilTagDetectionArray,
            self._callback,
            queue_size=1,
        )

    def estimate(self, message):
        estimates = {}
        stamps = {}
        for detection in message.detections:
            for tag_id in (0, 1):
                if (
                    tag_id not in detection.id
                    or tag_id not in self.tag_matrices
                    or tag_id in estimates
                ):
                    continue
                pose = detection.pose.pose.pose
                p, q = pose.position, pose.orientation
                position = [p.x, p.y, p.z]
                quaternion = [q.x, q.y, q.z, q.w]
                if (
                    not np.all(np.isfinite(position + quaternion))
                    or np.linalg.norm(quaternion) < 1e-12
                ):
                    continue
                camera_tag = tft.quaternion_matrix(quaternion)
                camera_tag[:3, 3] = position
                estimates[tag_id] = camera_tag @ self.tag_matrices[tag_id]
                stamps[tag_id] = detection.pose.header.stamp
        if not estimates:
            return None
        used = sorted(estimates)
        center = estimates[used[0]].copy()
        if len(used) == 2:
            other = estimates[1]
            if np.linalg.norm(center[:3, 3] - other[:3, 3]) > self.max_pair_gap:
                selected = 0
                if self.previous_center is not None:
                    selected = min(
                        used,
                        key=lambda i: np.linalg.norm(
                            estimates[i][:3, 3] - self.previous_center
                        ),
                    )
                rospy.logwarn_throttle(
                    2.0, "Tag center estimates disagree; using tag %d.", selected
                )
                center = estimates[selected].copy()
                used = [selected]
            else:
                center[:3, 3] = 0.5 * (center[:3, 3] + other[:3, 3])
                # Average full UAV orientations on the shortest quaternion arc.
                q = tft.quaternion_slerp(
                    tft.quaternion_from_matrix(center),
                    tft.quaternion_from_matrix(other),
                    0.5,
                )
                center[:3, :3] = tft.quaternion_matrix(q)[:3, :3]
        self.previous_center = center[:3, 3].copy()
        # Same calibrated transform and signs as the existing XY implementation.
        docking_uav = self.camera_matrix @ center
        stamp = message.header.stamp
        if not valid_stamp(stamp):
            sources = [stamps[i] for i in used if valid_stamp(stamps[i])]
            stamp = min(sources, key=lambda s: s.to_nsec()) if sources else None
        return docking_uav, stamp

    def _callback(self, message):
        received = rospy.Time.now()
        with self._lock:
            result = self.estimate(message)
            if result is None:
                return
            transform, stamp = result
            output = PoseStamped()
            output.header.stamp = stamp if stamp is not None else received
            output.header.frame_id = self.frame_id
            output.pose.position.x, output.pose.position.y, output.pose.position.z = (
                transform[:3, 3]
            )
            q = tft.quaternion_from_matrix(transform)
            (
                output.pose.orientation.x,
                output.pose.orientation.y,
                output.pose.orientation.z,
                output.pose.orientation.w,
            ) = q
            self.publisher.publish(output)


def main():
    rospy.init_node("apriltag_relative_pose")
    AprilTagRelativePose()
    rospy.spin()

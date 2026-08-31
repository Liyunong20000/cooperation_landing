#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Detect a commanded-but-stationary Go1 and request stand recovery."""

import math
import threading

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger


def wrap_angle(angle):
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(quaternion):
    """Return yaw from a geometry_msgs/Quaternion without extra dependencies."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class DogStallMonitor:
    """Monitor final velocity commands against UAV COG odometry progress.

    The UAV COG pose is used as the motion proxy requested by the manipulation
    system.  Linear commands are checked against planar XY displacement, while
    angular commands are checked against yaw change so that in-place rotations
    are not incorrectly classified as stalls.
    """

    def __init__(self):
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/go1/cmd_vel')
        self.pose_topic = rospy.get_param('~pose_topic', '/xuanwu/uav/cog/odom')
        self.stand_service = rospy.get_param('~stand_service', '/go1/stand')

        self.window_s = max(0.1, float(rospy.get_param('~window_s', 1.0)))
        self.min_translation_m = max(
            0.0, float(rospy.get_param('~min_translation_m', 0.01))
        )
        self.min_yaw_rad = max(
            0.0, float(rospy.get_param('~min_yaw_rad', 0.02))
        )
        self.linear_cmd_threshold = max(
            0.0, float(rospy.get_param('~linear_cmd_threshold', 0.02))
        )
        self.angular_cmd_threshold = max(
            0.0, float(rospy.get_param('~angular_cmd_threshold', 0.05))
        )
        self.cmd_timeout_s = max(
            self.window_s, float(rospy.get_param('~cmd_timeout_s', 1.5))
        )
        self.pose_timeout_s = max(
            0.1, float(rospy.get_param('~pose_timeout_s', 1.0))
        )
        self.required_windows = max(
            1, int(rospy.get_param('~required_windows', 2))
        )
        self.recovery_cooldown_s = max(
            0.0, float(rospy.get_param('~recovery_cooldown_s', 2.0))
        )
        self.stand_service_timeout_s = max(
            0.0, float(rospy.get_param('~stand_service_timeout_s', 1.0))
        )
        check_rate_hz = max(1.0, float(rospy.get_param('~check_rate_hz', 10.0)))

        self._lock = threading.RLock()
        self._pose = None
        self._pose_time = None
        self._linear_cmd = 0.0
        self._angular_cmd = 0.0
        self._cmd_time = None
        self._reference_pose = None
        self._reference_time = None
        self._bad_windows = 0
        self._last_recovery_time = None

        self._zero_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self._stand_client = rospy.ServiceProxy(self.stand_service, Trigger)
        self._cmd_sub = rospy.Subscriber(
            self.cmd_vel_topic, Twist, self._cmd_callback, queue_size=20
        )
        self._pose_sub = rospy.Subscriber(
            self.pose_topic, Odometry, self._pose_callback, queue_size=20
        )
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / check_rate_hz), self._check_stall
        )

        rospy.loginfo(
            'DogStallMonitor: cmd=%s pose=%s window=%.2fs '
            'min_xy=%.3fm min_yaw=%.3frad required_windows=%d',
            self.cmd_vel_topic,
            self.pose_topic,
            self.window_s,
            self.min_translation_m,
            self.min_yaw_rad,
            self.required_windows,
        )

    def _motion_is_commanded(self):
        return (
            self._linear_cmd >= self.linear_cmd_threshold
            or self._angular_cmd >= self.angular_cmd_threshold
        )

    def _clear_window(self):
        self._reference_pose = None
        self._reference_time = None
        self._bad_windows = 0

    def _cmd_callback(self, msg):
        now = rospy.Time.now()
        linear_cmd = math.hypot(msg.linear.x, msg.linear.y)
        angular_cmd = abs(msg.angular.z)

        with self._lock:
            was_commanded = self._motion_is_commanded()
            self._linear_cmd = linear_cmd
            self._angular_cmd = angular_cmd
            self._cmd_time = now
            is_commanded = self._motion_is_commanded()

            if not is_commanded:
                self._clear_window()
            elif not was_commanded:
                self._reference_pose = self._pose
                self._reference_time = now if self._pose is not None else None
                self._bad_windows = 0

    def _pose_callback(self, msg):
        position = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        with self._lock:
            self._pose = (position.x, position.y, yaw)
            self._pose_time = rospy.Time.now()

    def _set_reference(self, pose, now):
        self._reference_pose = pose
        self._reference_time = now

    def _check_stall(self, _event):
        now = rospy.Time.now()
        recovery = None

        with self._lock:
            if self._pose is None or self._pose_time is None:
                return

            if (now - self._pose_time).to_sec() > self.pose_timeout_s:
                self._clear_window()
                rospy.logwarn_throttle(
                    5.0,
                    'DogStallMonitor: pose topic %s is stale; stall check suspended.',
                    self.pose_topic,
                )
                return

            if self._cmd_time is None or (
                    now - self._cmd_time).to_sec() > self.cmd_timeout_s:
                self._clear_window()
                return

            if not self._motion_is_commanded():
                self._clear_window()
                return

            if self._last_recovery_time is not None and (
                    now - self._last_recovery_time).to_sec() < self.recovery_cooldown_s:
                self._set_reference(self._pose, now)
                self._bad_windows = 0
                return

            if self._reference_pose is None or self._reference_time is None:
                self._set_reference(self._pose, now)
                return

            if (now - self._reference_time).to_sec() < self.window_s:
                return

            ref_x, ref_y, ref_yaw = self._reference_pose
            x, y, yaw = self._pose
            translation = math.hypot(x - ref_x, y - ref_y)
            yaw_change = abs(wrap_angle(yaw - ref_yaw))

            translation_expected = self._linear_cmd >= self.linear_cmd_threshold
            rotation_expected = self._angular_cmd >= self.angular_cmd_threshold
            translation_stalled = (
                translation_expected and translation < self.min_translation_m
            )
            rotation_stalled = rotation_expected and yaw_change < self.min_yaw_rad

            if translation_stalled or rotation_stalled:
                self._bad_windows += 1
            else:
                self._bad_windows = 0

            self._set_reference(self._pose, now)

            if self._bad_windows < self.required_windows:
                return

            reasons = []
            if translation_stalled:
                reasons.append('translation')
            if rotation_stalled:
                reasons.append('rotation')
            recovery = (
                '+'.join(reasons),
                translation,
                yaw_change,
                self._linear_cmd,
                self._angular_cmd,
            )
            self._linear_cmd = 0.0
            self._angular_cmd = 0.0
            self._cmd_time = now
            self._last_recovery_time = now
            self._clear_window()

        if recovery is not None:
            reason, translation, yaw_change, linear_cmd, angular_cmd = recovery
            rospy.logwarn(
                'DogStallMonitor: %s stall detected: cmd_linear=%.3f m/s, '
                'cmd_yaw=%.3f rad/s, moved_xy=%.4f m, moved_yaw=%.4f rad. '
                'Publishing zero velocity and calling stand().',
                reason,
                linear_cmd,
                angular_cmd,
                translation,
                yaw_change,
            )
            self._zero_pub.publish(Twist())
            self._call_stand()

    def _call_stand(self):
        try:
            rospy.wait_for_service(
                self.stand_service, timeout=self.stand_service_timeout_s
            )
            response = self._stand_client()
            if response.success:
                rospy.loginfo('DogStallMonitor: stand recovery succeeded.')
            else:
                rospy.logwarn(
                    'DogStallMonitor: stand recovery was rejected: %s',
                    response.message,
                )
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr('DogStallMonitor: stand recovery failed: %s', exc)


def main():
    rospy.init_node('dog_stall_monitor')
    DogStallMonitor()
    rospy.spin()


if __name__ == '__main__':
    main()

"""Apriltag landing implementation."""

import math

import numpy as np
import rospy
import tf.transformations as tft

from cooperation_landing.apriltag_alignment import AprilmoveqilinNode
from cooperation_landing.dog_basic_function import DogBasic
from cooperation_landing.drone_basic_function import DroneBasic


def landing_param(name, default, config_name=None):
    """Read a private landing parameter with a shared-config fallback."""
    config_name = config_name or name
    return rospy.get_param(f'~{name}', rospy.get_param(f'/landing_info/{config_name}', default))


class AprillandqilinNode:
    """
    Node for cooperative landing between aerial drone and ground robot using AprilTags.
    """

    def __init__(self):
        rospy.logdebug('Initializing AprilTag landing controller.')

        self.takeoff_x, self.takeoff_y, self.takeoff_z = 0.0, 0.0, 0.0
        self.alignment_counter = 0
        self.aligned = False
        self.pause_when_lost = rospy.Duration(1)
        self.drone_pose_matrix = []
        self.success_start_time = None

        # Load parameters
        self.demo_target_index = rospy.get_param('~demo_target_index', 2)
        self.landing_points = rospy.get_param('/landing_points', [])
        self.required_frames = max(1, int(landing_param('required_frames', 10)))
        self.distance_threshold = max(0.0, float(landing_param('distance_threshold', 0.03)))
        self.drone_distance_threshold = max(
            0.0, float(landing_param('drone_distance_threshold', 0.10))
        )
        self.angle_threshold = max(
            0.0, float(landing_param('angle_threshold', 0.05, 'angle_threshold_rad'))
        )
        self.demo_hover_z = rospy.get_param('~demo_hover_z', 1.0)
        self.demo_final_z_offset = rospy.get_param('~demo_final_z_offset', 0.3)

        self.demo_target_x = rospy.get_param('~demo_target_x', -2)
        self.demo_target_y = rospy.get_param('~demo_target_y', 1)
        self.demo_target_z = rospy.get_param('~demo_target_z', 1.0)
        self.demo_target_quaternion = [0, 0, 0, 1]
        self.demo_target_timeout = rospy.get_param('~demo_target_timeout', 10.0)
        self.demo_target_retries = max(0, int(rospy.get_param('~demo_target_retries', 3)))
        self.takeoff_state_timeout = max(
            0.1, float(rospy.get_param('~takeoff_state_timeout_s', 30.0))
        )
        self.landing_alignment_timeout = max(
            0.1,
            float(landing_param('landing_alignment_timeout_s', 60.0, 'alignment_timeout_s')),
        )
        self._apply_landing_point_selection()

        self.drone_basic = DroneBasic()
        self.dog_basic = DogBasic()
        self.aqm = AprilmoveqilinNode()
        # Subscribe and publish.
        # rospy.Subscriber('/uavandgr/event', UInt8, self._callback_event)

    def _apply_landing_point_selection(self):
        """Select one landing/demo point from the YAML-loaded list."""
        if not isinstance(self.landing_points, list) or not self.landing_points:
            rospy.logwarn(
                'Aprillandqilin: no landing_points param found; using fallback demo target.'
            )
            return

        # Only support selection by index. Ensure index is valid and pick that point.
        try:
            idx = int(self.demo_target_index)
        except (TypeError, ValueError):
            idx = 0

        idx = max(0, min(idx, len(self.landing_points) - 1))
        point = self.landing_points[idx]

        self.demo_target_x = float(point.get('x', self.demo_target_x))
        self.demo_target_y = float(point.get('y', self.demo_target_y))
        self.demo_target_z = float(point.get('z', self.demo_target_z))

        quat = point.get('quaternion', None)
        if isinstance(quat, list) and len(quat) == 4:
            self.demo_target_quaternion = [float(v) for v in quat]
        else:
            yaw = float(point.get('yaw', 0.0))
            self.demo_target_quaternion = [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

        rospy.loginfo(
            'Aprillandqilin: selected landing point #%d -> x=%.3f y=%.3f z=%.3f quat=%s',
            idx,
            self.demo_target_x,
            self.demo_target_y,
            self.demo_target_z,
            str(self.demo_target_quaternion),
        )

    def is_alignment_success(self, T):
        """
        Check if alignment is successful based on distance and angle thresholds.
        """
        if not isinstance(T, np.ndarray) or T.shape != (4, 4):
            return False
        pos = T[:3, 3]
        x, y, _ = pos
        dist = np.linalg.norm([x, y])
        # Check the tag data
        T_tag_0 = self.aqm.find_target_tag(self.aqm.msg_apriltag, 0)
        if not isinstance(T_tag_0, np.ndarray) or T_tag_0.shape != (4, 4):
            rospy.logwarn('Tag 0 not found or invalid transform.')
            return False
        # Get the quaternion data from tag and transfer them into roll, pitch, yaw euler angle
        q = tft.quaternion_from_matrix(T_tag_0)
        roll, pitch, yaw = tft.euler_from_quaternion(q)

        return (dist < self.distance_threshold) and (abs(yaw) < self.angle_threshold)

    def check_and_land(self):
        """
        Check alignment and land the drone if conditions are met.
        """
        self.aqm.align_dog_with_drone()
        # this already handles tag loss and stops dog if needed

        T = self.aqm.dog_align_drone_matrix

        # Make sure we have a valid transform
        if T is not None and self.is_alignment_success(T):
            self.alignment_counter += 1
            rospy.loginfo_throttle(1.0, f'Alignment frame count: {self.alignment_counter}')
            # If the drone haven`t land off and the alignment counter is enough, the drone will land off
            if self.alignment_counter >= self.required_frames and not self.aligned:
                rospy.loginfo('Alignment held for sufficient frames. Landing drone.')
                self.drone_basic.drone_land()
                self.aligned = True
                return True
        else:
            self.alignment_counter = 0
            self.aligned = False
        return False

    def _wait_for_target(self, target_x, target_y, target_z, timeout_s=None):
        """
        Wait for drone to reach the target position.

        Args:
            target_x, target_y, target_z: Target position in world frame
            timeout_s: Timeout in seconds. If None, uses self.demo_target_timeout

        Returns:
            True if drone reached target, False if timeout occurred
        """
        if timeout_s is None:
            timeout_s = self.demo_target_timeout

        rate = rospy.Rate(10)
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            # Calculate distance from current position to target
            dx = self.drone_basic.drone_x - target_x
            dy = self.drone_basic.drone_y - target_y
            dz = self.drone_basic.drone_z - target_z
            distance = np.sqrt(dx**2 + dy**2 + dz**2)

            rospy.loginfo_throttle(
                1.0,
                f'Distance to target: {distance:.3f} m (threshold: {self.distance_threshold:.3f} m)',
            )

            # Check if drone has reached the target
            if distance < self.drone_distance_threshold:
                rospy.loginfo('Drone reached target position.')
                return True

            # Check timeout
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > timeout_s:
                rospy.logwarn(
                    f'Timeout waiting for target (elapsed: {elapsed:.1f}s > {timeout_s:.1f}s)'
                )
                return False

            rate.sleep()

    def _send_target_and_wait(self, x, y, z, quaternion):
        """Send a target with a bounded number of retries."""
        for attempt in range(self.demo_target_retries + 1):
            self.drone_basic.drone_target('world', x, y, z, *quaternion)
            if self._wait_for_target(x, y, z):
                return True
            rospy.logwarn(
                'Target attempt %d/%d failed; resending.',
                attempt + 1,
                self.demo_target_retries + 1,
            )
        return False

    def demo(self):
        """
        Demo sequence for drone takeoff and movement.
        Wait for drone to reach each target before moving to the next.
        """
        if not self.drone_basic.wait_for_odom():
            return False
        self.drone_basic.record_takeoff_position(
            self.drone_basic.drone_x,
            self.drone_basic.drone_y,
            self.drone_basic.drone_z,
            self.drone_basic.drone_yaw,
        )
        rate = rospy.Rate(10)
        for _ in range(10):  # 1 second
            rate.sleep()
        self.drone_basic.drone_start()
        for _ in range(10):  # 1 second
            rate.sleep()
        self.drone_basic.drone_takeoff()
        takeoff_deadline = rospy.Time.now() + rospy.Duration(self.takeoff_state_timeout)
        while not rospy.is_shutdown() and self.drone_basic.drone_state != 5:
            if rospy.Time.now() >= takeoff_deadline:
                rospy.logerr(
                    'Timed out after %.1f s waiting for flight state 5.', self.takeoff_state_timeout
                )
                return False
            rate.sleep()
        if rospy.is_shutdown():
            return False
        for _ in range(60):  # 6 seconds
            rate.sleep()
        # Send first target and wait for drone to reach it
        rospy.loginfo(
            'Sending first target: (%.3f, %.3f, %.3f)',
            self.demo_target_x,
            self.demo_target_y,
            self.demo_target_z,
        )
        if not self._send_target_and_wait(
            self.demo_target_x,
            self.demo_target_y,
            self.demo_target_z,
            self.demo_target_quaternion,
        ):
            return False

        rospy.sleep(5.0)
        # Send hover target and wait for drone to reach it
        hover_x = self.drone_basic.takeoff_x
        hover_y = self.drone_basic.takeoff_y
        rospy.loginfo(
            'Sending hover target: (%.3f, %.3f, %.3f)', hover_x, hover_y, self.demo_hover_z
        )
        if not self._send_target_and_wait(hover_x, hover_y, self.demo_hover_z, [0, 0, 0, 1]):
            return False

        rospy.sleep(3.0)
        # Send final target and wait for drone to reach it
        final_z = self.drone_basic.takeoff_z + self.demo_final_z_offset
        rospy.loginfo('Sending final target: (%.3f, %.3f, %.3f)', hover_x, hover_y, final_z)
        if not self._send_target_and_wait(hover_x, hover_y, final_z, [0, 0, 0, 1]):
            return False
        return True

    def run(self):
        """
        Main run loop for checking alignment and landing.
        """
        rate = rospy.Rate(10)  # 10 Hz
        deadline = rospy.Time.now() + rospy.Duration(self.landing_alignment_timeout)
        while not rospy.is_shutdown():
            if self.check_and_land():
                return True
            if rospy.Time.now() >= deadline:
                self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
                rospy.logerr(
                    'Landing alignment timed out after %.1f s.', self.landing_alignment_timeout
                )
                return False
            rate.sleep()
        return False


def main():
    """Run the ROS node."""
    try:
        rospy.init_node('Aprillandqilin', anonymous=True)
        if not rospy.get_param('~run_demo', False):
            rospy.logerr('Demo is disabled; set ~run_demo:=true to enable it explicitly.')
            raise SystemExit(2)
        if not rospy.get_param('~allow_takeoff', False):
            rospy.logerr('Takeoff is disabled; set ~allow_takeoff:=true after safety checks.')
            raise SystemExit(2)
        node = AprillandqilinNode()
        node.dog_basic.stand()
        rate = rospy.Rate(10)
        for _ in range(5):  # 0.5 seconds
            rate.sleep()
        if node.demo():
            node.run()
        else:
            rospy.logerr('Landing demo aborted because a bounded wait failed.')
    except rospy.ROSInterruptException:
        pass

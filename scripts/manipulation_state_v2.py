#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import math
import time
import numpy as np
import smach
import smach_ros
import tf.transformations as tft
from std_msgs.msg import Empty, UInt8
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist, PoseStamped
from tf.tfwtf import rostime_delta
from aerial_robot_msgs.msg import FlightNav
from apriltag_ros.msg import AprilTagDetectionArray

from basic_function import *
from dog_basic_function import *
from drone_basic_function import *
from gripper.gripper_move import *
from AprillandQilin import *


# ── Shared P-controller utilities ──────────────────────────────────────────

def clamp(x, lo, hi):
    """Clamp x to [lo, hi]."""
    return max(lo, min(hi, x))


def p_with_deadzone(err, k, v_min, v_max, tol):
    """P control with tolerance dead-zone and velocity saturation."""
    if abs(err) < tol:
        return 0.0
    v = k * err
    if abs(v) < v_min:
        v = v_min if v > 0 else -v_min
    return clamp(v, -v_max, v_max)


def wrap_angle(rad):
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(rad), math.cos(rad))


def scan_target_marker(userdata):
    object_state = getattr(userdata, 'object_state', 0)
    return int(userdata.picking_marker_far) if object_state == 0 else int(userdata.placing_marker_far)


def scan_tag_detected(drone_basic, marker_id):
    tag_info = drone_basic.tag_info
    for det in tag_info.detections:
        if marker_id in det.id:
            drone_basic.tag_position(tag_info, marker_id)
            return True
    return False


def scan_write_target_pose(userdata, drone_basic):
    pose = [
        drone_basic.tag_target_x,
        drone_basic.tag_target_y,
        drone_basic.tag_target_z,
        drone_basic.tag_target_qx,
        drone_basic.tag_target_qy,
        drone_basic.tag_target_qz,
        drone_basic.tag_target_qw,
    ]
    object_state = int(getattr(userdata, 'object_state', 0))
    if object_state == 0:
        userdata.picking_position = pose
    else:
        userdata.placing_position = pose


def resolve_marker_pair(userdata):
    """Select far/near marker IDs by object_state (0: pick, 1: place)."""
    object_state = int(getattr(userdata, 'object_state', 0))
    if object_state == 0:
        marker_far = int(userdata.picking_marker_far)
        marker_near = int(userdata.picking_marker_near)
    else:
        marker_far = int(userdata.placing_marker_far)
        marker_near = int(userdata.placing_marker_near)
    return marker_far, marker_near


def detect_with_marker_pair(drone_basic, marker_far, marker_near, marker_switch_z):
    """Detect far marker first, switch to near marker at close range if configured."""
    if scan_tag_detected(drone_basic, marker_far):
        if marker_near != marker_far and drone_basic.tag_target_z < marker_switch_z:
            if scan_tag_detected(drone_basic, marker_near):
                return True, marker_near
        return True, marker_far
    if marker_near != marker_far and scan_tag_detected(drone_basic, marker_near):
        return True, marker_near
    return False, marker_far


def run_visual_approach_loop(dog_basic, drone_basic, marker_far, marker_near,
                             marker_switch_z, target_z, x_bias,
                             tol_z, tol_x, tol_pitch,
                             k_vx, k_vy, k_yaw,
                             vx_min, vy_min, wz_min,
                             vx_max, vy_max, wz_max,
                             control_dt, timeout_s,
                             lost_tag_hold_s, relaxed_factor,
                             state_name='Approach'):
    """Shared closed-loop approach: keep tag centered and depth at target_z."""
    start_t = rospy.Time.now()
    rate = rospy.Rate(1.0 / control_dt)
    last_err = None
    last_seen_t = rospy.Time.now()
    best_err = [float('inf'), float('inf'), float('inf')]

    while not rospy.is_shutdown():
        if (rospy.Time.now() - start_t).to_sec() > timeout_s:
            close_enough = (
                best_err[0] < relaxed_factor * tol_z and
                best_err[1] < relaxed_factor * tol_x and
                best_err[2] < relaxed_factor * tol_pitch
            )
            dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
            return close_enough

        detected, active_marker = detect_with_marker_pair(drone_basic, marker_far, marker_near, marker_switch_z)
        if detected:
            z_err = drone_basic.tag_target_z - target_z
            x_err = drone_basic.tag_target_x - x_bias
            pitch_err = drone_basic.tag_target_pitch
            last_err = (z_err, x_err, pitch_err)
            last_seen_t = rospy.Time.now()
        else:
            if last_err is not None and (rospy.Time.now() - last_seen_t).to_sec() < lost_tag_hold_s:
                z_err, x_err, pitch_err = last_err
            else:
                dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
                rate.sleep()
                continue

        if abs(z_err) < tol_z and abs(x_err) < tol_x and abs(pitch_err) < tol_pitch:
            dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
            return True

        best_err[0] = min(best_err[0], abs(z_err))
        best_err[1] = min(best_err[1], abs(x_err))
        best_err[2] = min(best_err[2], abs(pitch_err))

        vx = p_with_deadzone(z_err, k_vx, vx_min, vx_max, tol_z)
        vy = p_with_deadzone(x_err, k_vy, vy_min, vy_max, tol_x)
        wz = p_with_deadzone(pitch_err, k_yaw, wz_min, wz_max, tol_pitch)
        dog_basic.qilin_cmd_vel(vx, vy, 0, 0, wz)
        rospy.logdebug('%s marker=%s err[z,x,p]=[%.3f, %.3f, %.3f]',
                       state_name, active_marker if detected else 'none', z_err, x_err, pitch_err)
        rate.sleep()

    dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
    return False

# The Cooperative manipulation system by Xuanwu
class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['object_state'])
        self.dog_basic = DogBasic()
        self.gripper_move = GripperMoveNode()

    def execute(self, userdata):
        self.dog_basic.stand()
        rospy.sleep(0.1)
        object_state = int(getattr(userdata, 'object_state', 0))
        if object_state == 0:
            self.gripper_move.servo_target_cmd_qilin(0, 1400)
        rospy.loginfo('pass begin.')
        return 'succeeded'


class HorizontalScan(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'need_vertical', 'failed'],
            input_keys=['object_state', 'picking_marker_far', 'placing_marker_far'],
            output_keys=['picking_position', 'placing_position']
        )
        self.dog_basic = DogBasic()
        self.drone_basic = DroneBasic()

        self.alpha_h = math.radians(rospy.get_param('~scan_alpha_h_deg', 70.0))
        self.rho_h = rospy.get_param('~scan_rho_h', 0.25)
        self.detect_wait_s = rospy.get_param('~scan_detect_wait_s', 2)
        self.yaw_tol = rospy.get_param('~scan_yaw_tol_rad', 0.3)
        self.yaw_kp = rospy.get_param('~scan_yaw_kp', 0.9)
        self.yaw_wz_max = rospy.get_param('~scan_yaw_wz_max', 0.6)
        self.rotate_timeout_s = rospy.get_param('~scan_rotate_timeout_s', 4.0)

        self.delta_yaw = max(0.12, (1.0 - self.rho_h) * self.alpha_h)
        self.n_h = int(math.ceil((2.0 * math.pi) / self.delta_yaw))
        self.yaw_offsets = [k * self.delta_yaw for k in range(self.n_h)]

        self.scan_index = 0
        self.base_yaw = None

    def _rotate_to(self, target_yaw):
        start_t = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            yaw_err = wrap_angle(target_yaw - self.drone_basic.drone_yaw)
            if abs(yaw_err) < self.yaw_tol:
                self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
                return True
            if (rospy.Time.now() - start_t).to_sec() > self.rotate_timeout_s:
                self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
                return False
            wz = clamp(self.yaw_kp * yaw_err, -self.yaw_wz_max, self.yaw_wz_max)
            self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, wz)
            rate.sleep()

    def execute(self, userdata):
        if self.scan_index == 0 or self.base_yaw is None:
            self.base_yaw = self.drone_basic.drone_yaw

        if self.scan_index >= len(self.yaw_offsets):
            rospy.logwarn('HorizontalScan: exhausted all yaw angles, target not found.')
            self.scan_index = 0
            self.base_yaw = None
            return 'failed'

        marker_id = scan_target_marker(userdata)
        target_yaw = wrap_angle(self.base_yaw + self.yaw_offsets[self.scan_index])
        rospy.loginfo('HorizontalScan: index=%d/%d, target_yaw=%.3f', self.scan_index + 1, len(self.yaw_offsets), target_yaw)

        self._rotate_to(target_yaw)

        detect_deadline = rospy.Time.now() + rospy.Duration(self.detect_wait_s)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < detect_deadline:
            if scan_tag_detected(self.drone_basic, marker_id):
                scan_write_target_pose(userdata, self.drone_basic)
                rospy.loginfo('HorizontalScan: marker %d detected.', marker_id)
                self.scan_index = 0
                self.base_yaw = None
                return 'succeeded'
            rate.sleep()

        self.scan_index += 1
        return 'need_vertical'


class VerticalScan(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['object_state', 'picking_marker_far', 'placing_marker_far'],
            output_keys=['picking_position', 'placing_position']
        )
        self.dog_basic = DogBasic()
        self.drone_basic = DroneBasic()

        # Vertical scan is limited to 0 deg and +30 deg.
        self.pitch_angles_deg = [15.0, 30.0]
        self.detect_wait_s = rospy.get_param('~scan_vertical_detect_wait_s', 2)


    def _set_pitch_deg(self, pitch_deg):
        # To reduce body oscillation, approach +45 deg through +30 deg first.
        if abs(pitch_deg - 30.0) < 1e-6:
            qx, qy, qz, qw = tft.quaternion_from_euler(0.0, math.radians(15.0), 0.0)
            self.dog_basic.qilin_body_pose(qx, qy, qz, qw)
            rospy.sleep(2.0)
        qx, qy, qz, qw = tft.quaternion_from_euler(0.0, math.radians(pitch_deg), 0.0)
        self.dog_basic.qilin_body_pose(qx, qy, qz, qw)
        rospy.sleep(0.2)

    def execute(self, userdata):
        marker_id = scan_target_marker(userdata)
        rospy.loginfo('VerticalScan: marker=%d, pitch_set=%s', marker_id, self.pitch_angles_deg)

        for pitch_deg in self.pitch_angles_deg:
            self._set_pitch_deg(pitch_deg)
            detect_deadline = rospy.Time.now() + rospy.Duration(self.detect_wait_s)
            rate = rospy.Rate(20)
            while not rospy.is_shutdown() and rospy.Time.now() < detect_deadline:
                if scan_tag_detected(self.drone_basic, marker_id):
                    scan_write_target_pose(userdata, self.drone_basic)
                    self._set_pitch_deg(0.0)
                    rospy.loginfo('VerticalScan: marker %d detected at pitch %.1f deg.', marker_id, pitch_deg)
                    return 'succeeded'
                rate.sleep()

        self._set_pitch_deg(0.0)
        return 'failed'


class StateJudgment(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeed_docking', 'succeed_detaching'],
            input_keys=['object_state', 'switching_threshold', 'picking_position', 'placing_position']
        )

    def execute(self, userdata):
        try:
            object_state = int(userdata.object_state)
            switching_threshold = float(userdata.switching_threshold)
            picking_position = userdata.picking_position
            placing_position = userdata.placing_position
        except AttributeError as e:
            rospy.logerr('StateJudgment: userdata key missing: %s', e)
            return 'succeed_docking'

        target_z = None
        if object_state == 0:
            if isinstance(picking_position, (list, tuple)) and len(picking_position) > 2:
                target_z = float(picking_position[2])
        elif object_state == 1:
            if isinstance(placing_position, (list, tuple)) and len(placing_position) > 2:
                target_z = float(placing_position[2])
        else:
            rospy.logwarn('StateJudgment: unsupported object_state=%s, fallback to docking.', object_state)
            return 'succeed_docking'

        if target_z is None:
            rospy.logwarn('StateJudgment: no valid target z found for object_state=%s, fallback to docking.', object_state)
            return 'succeed_docking'

        if target_z > switching_threshold:
            return 'succeed_detaching'
        return 'succeed_docking'


class DockApproach(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['object_state', 'picking_marker_far', 'placing_marker_far', 'picking_marker_near', 'placing_marker_near']
        )
        self.dog_basic = DogBasic()
        self.drone_basic = DroneBasic()

        self.manipulation_target_z_pick = rospy.get_param('~manipulation_target_z_pick', 0.79)
        self.manipulation_target_z_place = rospy.get_param('~manipulation_target_z_place', 0.79)
        # === Convergence tolerances ===
        # When errors are within these bounds, alignment is considered complete
        self.tol_z = 0.03  # longitudinal tolerance (m)
        self.tol_x = 0.05  # lateral tolerance (m)
        self.tol_pitch = 0.05  # yaw tolerance (rad)

        # === Proportional gains ===
        self.k_vx = 0.2  # gain for forward/backward motion
        self.k_vy = -0.5 # gain for lateral motion (sign depends on frame)
        self.k_yaw = -0.5  # gain for yaw rotation

        # === Minimum executable velocities (deadzone compensation) ===
        # These values ensure the Go1 actually moves when commands are small
        self.vx_min = 0.02       # m/s
        self.vy_min = 0.02       # m/s
        self.wz_min = 0.10       # rad/s

        # === Maximum velocities (safety limits) ===
        self.vx_max = 0.25 # m/s
        self.vy_max = 0.25 # m/s
        self.wz_max = 0.60  # rad/s

        # Control loop parameters
        self.control_dt = 0.1  # control period (s)
        self.timeout_s = 18.0  # maximum duration of this state (s)
        self.marker_switch_z = rospy.get_param('~manipulation_marker_switch_z', 0.5)
        self.lost_tag_hold_s = rospy.get_param('~dock_lost_tag_hold_s', 0.5)
        self.relaxed_factor = rospy.get_param('~dock_relaxed_factor', 2.0)
        self.x_bias = - 0.04

    def _active_marker_config(self, userdata):
        object_state = int(getattr(userdata, 'object_state', 0))
        if object_state == 0:
            marker_far = int(userdata.picking_marker_far)
            marker_near = int(userdata.picking_marker_near)
            manipulation_target_z = self.manipulation_target_z_pick

        else:
            marker_far = int(userdata.placing_marker_far)
            marker_near = int(userdata.placing_marker_near)
            manipulation_target_z = self.manipulation_target_z_place
        return marker_far, marker_near, manipulation_target_z


    def execute(self, userdata):
        marker_far, marker_near, manipulation_target_z = self._active_marker_config(userdata)
        ok = run_visual_approach_loop(
            self.dog_basic, self.drone_basic,
            marker_far, marker_near, self.marker_switch_z,
            manipulation_target_z, self.x_bias,
            self.tol_z, self.tol_x, self.tol_pitch,
            self.k_vx, self.k_vy, self.k_yaw,
            self.vx_min, self.vy_min, self.wz_min,
            self.vx_max, self.vy_max, self.wz_max,
            self.control_dt, self.timeout_s,
            self.lost_tag_hold_s, self.relaxed_factor,
            state_name='DockApproach'
        )
        if ok:
            self.dog_basic.sit()
            rospy.sleep(1.0)
            return 'succeeded'
        rospy.logwarn('DockApproach: timeout and not close enough, failed.')
        return 'failed'

class DockManipulation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['object_state'], output_keys=['object_state'])
        self.user_input = 0
        self.gripper_move = GripperMoveNode()
        self.dog_basic = DogBasic()
        self.drone_basic = DroneBasic()

    def execute(self, userdata):
        object_state = int(getattr(userdata, 'object_state', 0))
        time.sleep(1)

        if object_state == 0:
            # Picking: close gripper and attach module.
            self.gripper_move.servo_target_cmd_qilin(0, 200)
            time.sleep(1)
            self.gripper_move.servo_target_cmd_qilin(0, 200)
            rospy.sleep(2)
            self.drone_basic.call_add_extra_module(1, "brick", "main_body")
            userdata.object_state = 1
            rospy.loginfo('DockManipulation: pick mode, gripper close.')
        else:
            # Placing: open gripper and detach module.
            self.gripper_move.servo_target_cmd_qilin(0, 1400)
            time.sleep(1)
            self.gripper_move.servo_target_cmd_qilin(0, 1400)
            rospy.sleep(1)
            self.drone_basic.remove_module_trigger()
            userdata.object_state = 0
            rospy.loginfo('DockManipulation: place mode, gripper open.')

        self.dog_basic.stand()
        rospy.sleep(2)
        return 'succeeded'
class DockHome(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeed', 'finish'], input_keys=['object_state'])
        self.dog_basic = DogBasic()
        self.drone_basic = DroneBasic()
        self.home_x = rospy.get_param('~dock_home_x', 0.0)
        self.home_y = rospy.get_param('~dock_home_y', 0.0)
        self.home_yaw = rospy.get_param('~dock_home_yaw', 3.14)
        self.back_speed = rospy.get_param('~dock_home_back_speed', -0.15)
        self.back_duration_s = rospy.get_param('~dock_home_back_duration_s', 1.2)
        self.home_timeout_s = rospy.get_param('~dock_home_timeout_s', 8.0)
        self.k_home_xy = rospy.get_param('~dock_home_k_xy', 0.6)
        self.k_home_yaw = rospy.get_param('~dock_home_k_yaw', 0.2)
        self.max_home_v = rospy.get_param('~dock_home_max_v', 0.25)
        self.max_home_w = rospy.get_param('~dock_home_max_w', 0.6)
        self.pos_tol = rospy.get_param('~dock_home_pos_tol', 0.08)
        self.yaw_tol = rospy.get_param('~dock_home_yaw_tol', 0.10)

    def execute(self, userdata):
        object_state = int(getattr(userdata, 'object_state', 0))

        # Step 1: retreat backward to leave the marker/workspace safely.
        back_end_t = rospy.Time.now() + rospy.Duration(self.back_duration_s)
        back_rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < back_end_t:
            self.dog_basic.qilin_cmd_vel(self.back_speed, 0, 0, 0, 0)
            back_rate.sleep()
        self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)

        # Step 2: return to home (origin by default) using current odom.
        start_t = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            dx = self.home_x - self.drone_basic.drone_x
            dy = self.home_y - self.drone_basic.drone_y
            yaw_err = wrap_angle(self.home_yaw - self.drone_basic.drone_yaw)

            if math.hypot(dx, dy) < self.pos_tol and abs(yaw_err) < self.yaw_tol:
                break

            if (rospy.Time.now() - start_t).to_sec() > self.home_timeout_s:
                rospy.logwarn('DockHome: return-home timeout, stop at current pose.')
                break

            vx = clamp(self.k_home_xy * dx, -self.max_home_v, self.max_home_v)
            vy = clamp(self.k_home_xy * dy, -self.max_home_v, self.max_home_v)
            wz = clamp(self.k_home_yaw * yaw_err, -self.max_home_w, self.max_home_w)
            self.dog_basic.qilin_cmd_vel(vx, vy, 0, 0, wz)
            rate.sleep()

        self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)

        # If carrying payload now (state=1), go back to search for placing marker.
        if object_state == 1:
            return 'succeed'
        # If payload already released (state=0), this cycle is finished.
        return 'finish'

class DetachApproach(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['object_state', 'picking_marker_far', 'placing_marker_far', 'picking_marker_near', 'placing_marker_near', 'detaching_takeoff_threshold'],
            output_keys=['picking_position', 'placing_position']
        )
        self.drone_basic = DroneBasic()
        self.dog_basic = DogBasic()
        self.basic = BasicNode()
        self.marker_switch_z = rospy.get_param('~manipulation_marker_switch_z', 0.75)
        self.tol_z = 0.03
        self.tol_x = 0.05
        self.tol_pitch = 0.05

        self.k_vx = 0.2
        self.k_vy = -0.5
        self.k_yaw = -0.5

        # === Minimum executable velocities (deadzone compensation) ===
        # These values ensure the Go1 actually moves when commands are small
        # These values ensure the Go1 actually moves when commands are small
        self.vx_min = 0.02       # m/s
        self.vy_min = 0.02       # m/s
        self.wz_min = 0.10       # rad/s

        # === Maximum velocities (safety limits) ===
        self.vx_max = 0.25  # m/s
        self.vy_max = 0.25  # m/s
        self.wz_max = 0.60  # rad/s

        # Control loop parameters
        self.control_dt = 0.1  # control period (s)
        self.timeout_s = 12.0  # maximum duration of this state (s)
        self.x_bias = rospy.get_param('~detach_x_bias', 0.0)
        self.lost_tag_hold_s = rospy.get_param('~detach_lost_tag_hold_s', 0.5)
        self.relaxed_factor = rospy.get_param('~detach_relaxed_factor', 2.0)

        # Reference transform chain (same idea as MoveDestination):
        # world->base, base<-camera, camera->tag, tag->target(desk).
        self.tag2desk_y = rospy.get_param('~tag2desk_y', 0.31)
        self.tag2desk_z = rospy.get_param('~tag2desk_z', 0.105)
        self.d_camera2spinal = rospy.get_param('~d_camera2spinal', 0.058)
        self.h_camera2spinal = rospy.get_param('~h_camera2spinal', 0.080)
        self.h_gripper2spinal = rospy.get_param('~h_gripper2spinal', 0.201)
        self.h_safe = rospy.get_param('~h_safe', 0.2)
        self.t_cam2base = np.array([
            [0, 0, 1, 0.058],
            [-1, 0, 0, 0.0],
            [0, -1, 0, -0.0798],
            [0, 0, 0, 1]
        ], dtype=float)
        self.t_desk2tag = np.array([
            [-1, 0, 0, 0],
            [0, 0, 1, 0.105],
            [0, 1, 0, -0.31],
            [0, 0, 0, 1]
        ], dtype=float)

    def _t_se3(self, x, y, z, qx, qy, qz, qw):
        T = tft.quaternion_matrix([qx, qy, qz, qw])
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = z
        return T

    def _compute_target_pose_world(self):
        wtb = self._t_se3(
            self.drone_basic.drone_x, self.drone_basic.drone_y, self.drone_basic.drone_z,
            self.drone_basic.drone_qx, self.drone_basic.drone_qy, self.drone_basic.drone_qz, self.drone_basic.drone_qw
        )
        ctt = self._t_se3(
            self.drone_basic.tag_target_x, self.drone_basic.tag_target_y, self.drone_basic.tag_target_z,
            self.drone_basic.tag_target_qx, self.drone_basic.tag_target_qy, self.drone_basic.tag_target_qz, self.drone_basic.tag_target_qw
        )
        wtd = wtb @ self.t_cam2base @ ctt @ self.t_desk2tag

        target_x = float(wtd[0, 3])
        target_y = float(wtd[1, 3])
        target_z = float(wtd[2, 3] + self.h_safe + self.h_gripper2spinal)
        _, _, target_yaw = tft.euler_from_matrix(wtd, axes='sxyz')
        qx, qy, qz, qw = tft.quaternion_from_euler(0.0, 0.0, target_yaw)

        # Mocap values are only for reference comparison and do not affect output.
        target_mocap_x = self.basic.mocap_xuanwu_x - self.drone_basic.tag_target_x
        target_mocap_y = self.basic.mocap_xuanwu_y - self.d_camera2spinal - self.drone_basic.tag_target_z - self.tag2desk_y
        target_mocap_z = self.basic.mocap_xuanwu_z - self.h_camera2spinal - self.drone_basic.tag_target_y + self.tag2desk_z + self.h_safe + self.h_gripper2spinal
        rospy.loginfo('DetachApproach target(world): [%.3f, %.3f, %.3f], yaw=%.3f', target_x, target_y, target_z, target_yaw)
        rospy.loginfo('DetachApproach target(mocap-ref): [%.3f, %.3f, %.3f]', target_mocap_x, target_mocap_y, target_mocap_z)

        return [target_x, target_y, target_z, qx, qy, qz, qw]

    def execute(self, userdata):
        object_state = int(getattr(userdata, 'object_state', 0))
        marker_far, marker_near = resolve_marker_pair(userdata)
        target_z = float(userdata.detaching_takeoff_threshold)

        ok = run_visual_approach_loop(
            self.dog_basic, self.drone_basic,
            marker_far, marker_near, self.marker_switch_z,
            target_z, self.x_bias,
            self.tol_z, self.tol_x, self.tol_pitch,
            self.k_vx, self.k_vy, self.k_yaw,
            self.vx_min, self.vy_min, self.wz_min,
            self.vx_max, self.vy_max, self.wz_max,
            self.control_dt, self.timeout_s,
            self.lost_tag_hold_s, self.relaxed_factor,
            state_name='DetachApproach'
        )
        if ok:
            target_pose = self._compute_target_pose_world()
            if object_state == 0:
                userdata.picking_position = target_pose
            else:
                userdata.placing_position = target_pose
            rospy.loginfo('DetachApproach: reached z threshold %.3f', target_z)
            return 'succeeded'
        rospy.logwarn('DetachApproach: timeout and not close enough, failed.')
        return 'failed'

class Takeoff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['takeoff_position'], output_keys=['takeoff_position'])
        self.drone_basic = DroneBasic()
        self.takeoff_position = [0.0, 0.0, 0.0, 0.0]

    def execute(self, userdata):
        self.drone_basic.record_takeoff_position(self.drone_basic.drone_x, self.drone_basic.drone_y,
                                                 self.drone_basic.drone_z, self.drone_basic.drone_yaw)
        time.sleep(0.1)
        # self.drone_basic.drone_start()
        time.sleep(0.1)
        # self.drone_basic.drone_takeoff()
        rospy.loginfo(f'takeoff!!!!!!!!!!')
        userdata.takeoff_position = [self.drone_basic.takeoff_x, self.drone_basic.takeoff_y,
                                     self.drone_basic.takeoff_z, self.drone_basic.takeoff_yaw]
        rospy.loginfo(f'takeoff_x:{self.drone_basic.takeoff_x}, takeoff_y:{self.drone_basic.takeoff_y}, '
                      f'takeoff_z:{self.drone_basic.takeoff_z}, takeoff_yaw:{self.drone_basic.takeoff_yaw}')
        while self.drone_basic.drone_state != 5:
            time.sleep(0.1)
        rospy.sleep(0.1)
        return 'succeeded'

class FlyTarget(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['object_state', 'picking_position', 'placing_position'],
            output_keys=['object_state']
        )
        self.drone_basic = DroneBasic()
        self.dog_basic = DogBasic()
        self.gripper_move = GripperMoveNode()
        self.target_position = [0.0] * 7

    def execute(self, userdata):
        object_state = int(getattr(userdata, 'object_state', 0))
        self.target_position = userdata.picking_position if object_state == 0 else userdata.placing_position
        if not isinstance(self.target_position, (list, tuple)) or len(self.target_position) < 7:
            rospy.logerr('FlyTarget: invalid target position from DetachApproach: %s', self.target_position)
            return 'failed'

        qx, qy, qz, qw = self.target_position[3], self.target_position[4], self.target_position[5], self.target_position[6]
        self.drone_basic.drone_target('world', self.target_position[0], self.target_position[1], 1.0, 0, 0, 0, 1)
        rospy.loginfo(f'send fly target')
        rospy.sleep(8)
        rospy.loginfo(f'send fly target again')
        self.drone_basic.drone_target('world', self.target_position[0], self.target_position[1],
                                      self.target_position[2], qx, qy, qz, qw)
        rospy.loginfo(f'{self.target_position[0]}, {self.target_position[1]}, {self.target_position[2]}, {qx}, {qy}, {qz}, {qw}')
        rospy.sleep(6)
        if object_state == 0:
            rospy.loginfo('Close gripper')
            self.gripper_move.servo_target_cmd_qilin(0, 200)
            rospy.sleep(0.5)
            self.gripper_move.servo_target_cmd_qilin(0, 200)
            userdata.object_state = 1
        elif object_state == 1:
            rospy.loginfo('Open gripper')
            self.gripper_move.servo_target_cmd_qilin(0, 1400)
            self.drone_basic.remove_module_trigger()
            userdata.object_state = 0
        while not rospy.is_shutdown():
            s = input("Type 'y' to continue: ").strip().lower()
            if s == 'y':
                break
        return 'succeeded'

class FlyBack(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['takeoff_position'])
        self.takeoff_x, self.takeoff_y, self.takeoff_z, self.takeoff_yaw = 0.0, 0.0, 0.0, 0.0
        self.land_offset = 0.3
        self.drone_basic = DroneBasic()
        self.dog_basic = DogBasic()
        self.aqm = AprilmoveqilinNode()
        self.timeout_s = 12.0
        self.takeoff_position = [0.0, 0.0, 0.0, 0.0]

    def execute(self, userdata):
        rospy.loginfo(f'Flyback!!')
        self.takeoff_x = userdata.takeoff_position[0]
        self.takeoff_y = userdata.takeoff_position[1]
        self.takeoff_z = userdata.takeoff_position[2]
        qx, qy, qz, qw = tft.quaternion_from_euler(0, 0, userdata.takeoff_position[3])
        print(f'This is the fly back position: {self.takeoff_x}, {self.takeoff_y}, {self.takeoff_z}')
        time.sleep(1)
        self.drone_basic.drone_target('world', self.takeoff_x, self.takeoff_y, 1.0, qx, qy, qz, qw)
        rospy.loginfo(f'fly back:x= {self.takeoff_x}, {self.takeoff_y}, {self.takeoff_z}')
        time.sleep(8)
        self.drone_basic.drone_target('world', self.takeoff_x, self.takeoff_y,
                                      self.takeoff_z + self.land_offset, qx, qy, qz, qw)
        rospy.loginfo(f'above 0.3m!!')
        return 'succeeded'

class AlignAndLand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.alm = AprillandqilinNode()

    def execute(self, userdata):
        self.alm.run()
        return 'succeeded'

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        return 'succeeded'

class Finish(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted'])

    def execute(self, userdata):
        return 'preempted'

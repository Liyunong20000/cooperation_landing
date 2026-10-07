"""Repeat the AprilTag demo route, then yield navigation to UAV visual landing."""

import math
import time

import rospy
from std_msgs.msg import Empty, UInt8

from cooperation_landing.bridge_preflight import BridgePreflight
from cooperation_landing.dog_basic_function import DogBasic
from cooperation_landing.drone_basic_function import DroneBasic


IDLE, ALIGNING, ALIGNED, VISION_LOST, ABORTED, DESCENDING, BRAKING = range(7)
ARM_OFF, START, ARM_ON, LAND, HOVER = 0, 1, 2, 4, 5
VISUAL_NAMES = ('IDLE', 'ALIGNING', 'ALIGNED', 'VISION_LOST', 'ABORTED', 'DESCENDING', 'BRAKING')
FLIGHT_NAMES = {0: 'ARM_OFF', 1: 'START', 2: 'ARM_ON', 3: 'TAKEOFF', 4: 'LAND', 5: 'HOVER', 6: 'STOP'}


def select_point(points, index):
    """Validate selection before issuing commands; never silently change the point."""
    if not isinstance(points, list) or not points:
        raise ValueError('landing_points must be a nonempty list')
    if isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < len(points):
        raise ValueError('demo_target_index must be an integer in [0, %d]' % (len(points) - 1))
    point = points[index]
    xyz = tuple(float(point[key]) for key in ('x', 'y', 'z'))
    if not all(math.isfinite(value) for value in xyz):
        raise ValueError('landing point coordinates must be finite')
    if 'quaternion' in point:
        quat = tuple(float(value) for value in point['quaternion'])
        if len(quat) != 4 or not all(math.isfinite(value) for value in quat):
            raise ValueError('landing point quaternion must contain four finite values')
        norm = math.sqrt(sum(value * value for value in quat))
        if norm < 1e-6:
            raise ValueError('landing point quaternion must be nonzero')
        quat = tuple(value / norm for value in quat)
    else:
        yaw = float(point.get('yaw', 0.0))
        if not math.isfinite(yaw):
            raise ValueError('landing point yaw must be finite')
        quat = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    return xyz, quat


class DemoDrone(DroneBasic):
    """Track callback arrival times so cached telemetry cannot finish a waypoint."""

    def __init__(self):
        self.odom_arrival = None
        self.odom_stamp = None
        self.flight_arrival = None
        super().__init__()

    def _callback_drone_position(self, msg):
        if msg.header.frame_id.lstrip('/') != 'world':
            self.odom_arrival = None
            return
        stamp = msg.header.stamp.to_nsec()
        if stamp <= 0 or (self.odom_stamp is not None and stamp <= self.odom_stamp):
            return  # Repeated bridge cache must not refresh source odometry.
        super()._callback_drone_position(msg)
        if all(math.isfinite(value) for value in
               (self.drone_x, self.drone_y, self.drone_z, self.drone_yaw)):
            self.odom_arrival = time.monotonic()
            self.odom_stamp = stamp
        else:
            self.odom_arrival = None

    def _callback_drone_state(self, msg):
        if msg.data != self.drone_state:
            rospy.loginfo('[飞行状态] %s -> %s (%d)',
                          FLIGHT_NAMES.get(self.drone_state, str(self.drone_state)),
                          FLIGHT_NAMES.get(msg.data, str(msg.data)), msg.data)
        super()._callback_drone_state(msg)
        self.flight_arrival = time.monotonic()


class VisuallandqilinNode:
    """One selected-point flight per invocation, using one shared demo config."""

    def __init__(self):
        points = rospy.get_param('~landing_points', rospy.get_param('/landing_points', []))
        self.index = rospy.get_param('~demo_target_index', 0)
        self.point_xyz, self.point_quat = select_point(points, self.index)
        rospy.loginfo('[初始化] 选择点位 p%d，world xyz=%s，quaternion=%s',
                      self.index + 1, self.point_xyz, self.point_quat)
        self.visual_state = None
        self.session_started = False
        self.landing_seen = False
        self.drone_basic = DemoDrone()
        self.dog_basic = DogBasic()
        ns = self.drone_basic.robot_ns + '/visual_landing'
        self.trigger = rospy.Publisher(ns + '/trigger', Empty, queue_size=1)
        self.cancel = rospy.Publisher(ns + '/cancel', Empty, queue_size=1)
        self.state_sub = rospy.Subscriber(ns + '/state', UInt8, self._state_callback, queue_size=1)
        self.communication = BridgePreflight()
        rospy.on_shutdown(self.cancel_session)

    def _state_callback(self, msg):
        if msg.data != self.visual_state:
            name = VISUAL_NAMES[msg.data] if 0 <= msg.data < len(VISUAL_NAMES) else 'UNKNOWN'
            rospy.loginfo('[视觉状态] %s (%d)', name, msg.data)
        self.visual_state = msg.data

    def _number(self, name, default):
        value = float(rospy.get_param('~' + name, default))
        if not math.isfinite(value) or value <= 0:
            raise ValueError('%s must be finite and positive' % name)
        return value

    def _fresh(self, arrival):
        return arrival is not None and time.monotonic() - arrival <= self._number(
            'telemetry_timeout_s', 1.0)

    def _wait(self, condition, timeout, label):
        rospy.loginfo('[等待] %s，最多 %.1f s。', label, timeout)
        deadline = time.monotonic() + timeout
        next_log = time.monotonic() + 2.0
        while not rospy.is_shutdown():
            if condition():
                rospy.loginfo('[完成] %s。', label)
                return True
            if time.monotonic() >= deadline:
                rospy.logerr('Timed out waiting for %s (%.1f s).', label, timeout)
                return False
            if time.monotonic() >= next_log:
                rospy.loginfo('[等待] %s，剩余 %.1f s，flight_state=%s，visual_state=%s。',
                              label, max(0.0, deadline - time.monotonic()),
                              self.drone_basic.drone_state, self.visual_state)
                next_log = time.monotonic() + 2.0
            time.sleep(0.05)
        return False

    def _hovering(self):
        drone = self.drone_basic
        return self._fresh(drone.flight_arrival) and drone.drone_state == HOVER

    def _send_target_and_wait(self, xyz, quat):
        drone = self.drone_basic
        yaw = math.atan2(2 * (quat[3] * quat[2] + quat[0] * quat[1]),
                         1 - 2 * (quat[1] ** 2 + quat[2] ** 2))
        distance_limit = self._number('drone_distance_threshold', 0.10)
        yaw_limit = self._number('target_yaw_threshold', 0.10)
        stall_timeout = self._number('target_no_progress_timeout_s', 3.0)
        progress_distance = self._number('target_progress_distance_m', 0.02)
        progress_yaw = self._number('target_progress_yaw_rad', 0.02)
        timeout = self._number('demo_target_timeout', 20.0)
        retries = max(0, int(rospy.get_param('~demo_target_retries', 3)))

        def errors():
            distance = math.sqrt(sum((actual - target) ** 2 for actual, target in zip(
                (drone.drone_x, drone.drone_y, drone.drone_z), xyz)))
            yaw_error = math.atan2(math.sin(drone.drone_yaw - yaw),
                                   math.cos(drone.drone_yaw - yaw))
            return distance, abs(yaw_error)

        def publish(attempt):
            rospy.loginfo('[航点发送 %d/%d] world xyz=%s，yaw=%.3f rad；发送完整 pose + trigger。',
                          attempt + 1, retries + 1, xyz, yaw)
            drone.drone_target('world', *xyz, *quat)

        if not self._fresh(drone.odom_arrival) or not self._hovering() or self.session_started:
            rospy.logerr('[航点中止] 需要新鲜里程计和 HOVER，且视觉控制器尚未接管。')
            return False
        best_distance, best_yaw = errors()
        attempt = 0
        publish(attempt)
        last_progress = last_send = time.monotonic()
        next_log = last_send
        while not rospy.is_shutdown():
            now = time.monotonic()
            if not self._fresh(drone.odom_arrival) or not self._hovering() or self.session_started:
                rospy.logerr('[航点中止] 遥测超时、飞行状态改变或视觉接管；停止重发。')
                return False
            distance, yaw_error = errors()
            if distance < distance_limit and yaw_error < yaw_limit:
                rospy.loginfo('[航点到达] xyz=(%.3f, %.3f, %.3f)，距离误差=%.3f m，yaw误差=%.3f rad。',
                              drone.drone_x, drone.drone_y, drone.drone_z, distance, yaw_error)
                return True
            if now >= next_log:
                rospy.loginfo('[航点反馈] 当前=(%.3f, %.3f, %.3f)，目标=%s，距离=%.3f m，'
                              'yaw误差=%.3f rad，%.1f s 无明显进展。',
                              drone.drone_x, drone.drone_y, drone.drone_z, xyz,
                              distance, yaw_error, now - last_progress)
                next_log = now + 1.0
            if (best_distance - distance >= progress_distance
                    or (distance < distance_limit and best_yaw - yaw_error >= progress_yaw)):
                best_distance, best_yaw = distance, yaw_error
                last_progress = now
            reason = None
            if now - last_progress >= stall_timeout:
                reason = '位置/yaw 误差持续 %.1f s 没有明显减小' % stall_timeout
            elif now - last_send >= timeout:
                reason = '单次航点等待超过 %.1f s' % timeout
            if reason:
                if attempt >= retries:
                    rospy.logerr('[航点失败] %s，已用完 %d 次重试。', reason, retries)
                    return False
                rospy.logwarn('[航点重试] %s，重发原目标。', reason)
                attempt += 1
                publish(attempt)
                last_progress = last_send = time.monotonic()
                best_distance, best_yaw = errors()
            time.sleep(0.05)
        return False

    def _hold(self, label, seconds):
        rospy.loginfo('[保持] %s，持续 %.1f s。', label, seconds)
        deadline = time.monotonic() + seconds
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if not self._fresh(self.drone_basic.odom_arrival) or not self._hovering():
                rospy.logerr('[保持中止] %s期间遥测超时或退出 HOVER。', label)
                return False
            time.sleep(0.05)
        return not rospy.is_shutdown()

    def demo(self):
        """Use the original outbound/return route without ground AprilTag motion."""
        drone = self.drone_basic
        rospy.loginfo('[步骤 1/9] 检查 UAV2GR、GR2UAV 高速通道全部话题。')
        if not self.communication.check():
            rospy.logerr('[起飞中止] 高速通信预检未通过。')
            return False
        rospy.loginfo('[步骤 2/9] 检查世界坐标里程计、飞行状态和视觉控制器。')
        if not self._wait(lambda: self._fresh(drone.odom_arrival)
                          and self._fresh(drone.flight_arrival),
                          self._number('odom_wait_timeout_s', 5.0), '世界坐标里程计和飞行状态'):
            return False
        if drone.drone_state not in (ARM_OFF, START, ARM_ON):
            rospy.logerr('[起飞中止] 需要起飞前状态 ARM_OFF、START 或 ARM_ON，当前状态=%d。',
                         drone.drone_state)
            return False
        if not self._wait(lambda: self.visual_state == IDLE
                          and self.trigger.get_num_connections() > 0
                          and self.cancel.get_num_connections() > 0,
                          self._number('controller_ready_timeout_s', 10.0), '视觉控制器 IDLE 和触发连接'):
            return False
        rospy.loginfo('[步骤 3/9] 准备机器狗并记录起飞位置。')
        if rospy.get_param('~stand_ground_robot', True) and not self.dog_basic.stand():
            rospy.logerr('[准备中止] 机器狗站立请求失败。')
            return False
        self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, 0)
        drone.record_takeoff_position(drone.drone_x, drone.drone_y, drone.drone_z, drone.drone_yaw)
        rospy.loginfo('[起飞位置] world xyz=(%.3f, %.3f, %.3f)，yaw=%.3f rad。',
                      drone.takeoff_x, drone.takeoff_y, drone.takeoff_z, drone.takeoff_yaw)
        rospy.loginfo('[步骤 4/9] 请人工启动电机；收到 ARM_ON (2) 后，程序将发送起飞指令。')
        if not self._wait(lambda: self._fresh(drone.flight_arrival) and drone.drone_state == ARM_ON,
                          self._number('manual_start_timeout_s', 60.0), '人工启动电机 ARM_ON (2)'):
            return False
        rospy.loginfo('[起飞] 已确认人工启动电机，发送 takeoff 指令。')
        drone.drone_takeoff()
        if not self._wait(self._hovering, self._number('takeoff_state_timeout_s', 30.0), '起飞后 HOVER (5)'):
            return False
        if not self._hold('起飞后稳定悬停', self._number('takeoff_settle_s', 6.0)):
            return False
        rospy.loginfo('[步骤 5/9] 飞到所选点 p%d，xyz=%s。', self.index + 1, self.point_xyz)
        if not self._send_target_and_wait(self.point_xyz, self.point_quat):
            return False
        if not self._hold('所选点停留', self._number('point_hold_s', 5.0)):
            return False
        hover = (drone.takeoff_x, drone.takeoff_y, self._number('demo_hover_z', 1.0))
        rospy.loginfo('[步骤 6/9] 返回起飞位置上方，world xyz=%s。', hover)
        if not self._send_target_and_wait(hover, (0, 0, 0, 1)):
            return False
        if not self._hold('返回位置悬停', self._number('hover_hold_s', 3.0)):
            return False
        final = (drone.takeoff_x, drone.takeoff_y,
                 drone.takeoff_z + self._number('demo_final_z_offset', 0.3))
        rospy.loginfo('[步骤 7/9] 移到视觉降落准备位置，world xyz=%s。', final)
        return self._send_target_and_wait(final, (0, 0, 0, 1))

    def _landing_result(self):
        """ALIGNED is alignment-only; only fresh JSK flight states prove landing."""
        drone = self.drone_basic
        if self._fresh(drone.flight_arrival):
            if drone.drone_state == LAND:
                if not self.landing_seen:
                    rospy.loginfo('[降落反馈] JSK 已进入 LAND (4)，等待电机关闭 ARM_OFF (0)。')
                self.landing_seen = True
            if self.landing_seen and drone.drone_state == ARM_OFF:
                self.session_started = False
                rospy.loginfo('[降落反馈] 已进入 ARM_OFF (0)，本次降落完成。')
                return True
        if not self.landing_seen and self.visual_state in (IDLE, ALIGNED, ABORTED):
            raise RuntimeError('Visual session stopped (state %s); automatic landing requires '
                               'UAV bringup descend:=true.' % self.visual_state)
        return False

    def run(self):
        """Send exactly one trigger, then monitor; never compete for UAV navigation."""
        if not self._hovering() or not self._fresh(self.drone_basic.odom_arrival):
            rospy.logerr('[视觉触发中止] 缺少新鲜遥测或无人机未处于 HOVER。')
            return False
        if self.visual_state != IDLE:
            rospy.logerr('Visual controller is no longer IDLE; demo aborted.')
            return False
        self.session_started = True
        rospy.loginfo('[步骤 8/9] 发送一次视觉降落 trigger，等待机载状态确认。')
        self.trigger.publish(Empty())
        if not self._wait(lambda: self.visual_state in (ALIGNING, VISION_LOST, DESCENDING, BRAKING),
                          self._number('trigger_ack_timeout_s', 5.0), '视觉触发确认'):
            return False
        rospy.loginfo('[步骤 9/9] 机载视觉控制器接管导航，等待对齐、下降和降落完成。')
        return self._wait(self._landing_result, self._number('visual_landing_timeout_s', 120.0),
                          '降落完成')

    def cancel_session(self):
        if self.session_started:
            rospy.logwarn('[退出] 发送 visual_landing/cancel，结束本程序触发的视觉会话。')
            self.cancel.publish(Empty())
            self.session_started = False


def main():
    rospy.init_node('visual_land_demo')
    if not rospy.get_param('~run_demo', False) or not rospy.get_param('~allow_takeoff', False):
        rospy.logerr('Set run_demo:=true and allow_takeoff:=true to run the flight demo.')
        raise SystemExit(2)
    node = None
    success = False
    try:
        node = VisuallandqilinNode()
        success = node.demo() and node.run()
    except (ValueError, KeyError, TypeError, RuntimeError, OSError) as error:
        rospy.logerr('Visual landing demo aborted: %s', error)
    except rospy.ROSInterruptException:
        pass
    finally:
        if node is not None:
            node.cancel_session()
            # Give the event-driven bridge a chance to consume the final cancel.
            time.sleep(0.2)
    if not success:
        rospy.logerr('[流程结束] 本次演示未完成，请检查上方的中止或超时原因。')
        raise SystemExit(1)
    rospy.loginfo('Visual landing demo completed.')

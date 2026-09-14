"""Event trigger implementation."""

import threading

import rospy
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped
from spinal.msg import ServoControlCmd, ServoStates
from std_msgs.msg import Bool, Empty, Int8, Int32, UInt8

from cooperation_landing.drone_basic_function import DroneBasic


class EventtriggerNode:
    def __init__(self):
        rospy.logdebug('Initializing network event bridge.')
        self.robot_ns = ('/' + str(rospy.get_param('~robot_ns', 'xuanwu')).strip('/')).rstrip('/')
        self.allow_remote_commands = bool(rospy.get_param('~allow_remote_commands', True))
        # Subscribe and publish.
        # rospy.Subscriber('/quadrotor/uav/nav/info', FlightNav, self._callback_nav_info)

        # self.pub_event = rospy.Publisher('/uavandgr/event', UInt8, queue_size=10)
        self.pub_drone_target = rospy.Publisher(
            self.robot_ns + '/target_pose', PoseStamped, queue_size=10
        )
        self.pub_drone_nav = rospy.Publisher(self.robot_ns + '/uav/nav', FlightNav, queue_size=10)
        self.pub_servo_target = rospy.Publisher(
            self.robot_ns + '/servo/target_states', ServoControlCmd, queue_size=10
        )

        # self.pub_drone_nav = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)
        # self.pub_takeoff = rospy.Publisher('/quadrotor/teleop_command/takeoff', Empty, queue_size=10)
        # self.pub_land = rospy.Publisher('/quadrotor/teleop_command/land', Empty, queue_size=10)

        self.drone_basic = DroneBasic()

        self.x_y_mode, self.z_mode = 0, 0
        self.target_x, self.target_y, self.target_z = 0.0, 0.0, 0.0
        self.target_ox, self.target_oy, self.target_oz, self.target_ow = 0.0, 0.0, 0.0, 1.0
        self.target_frame = 'world'
        self.target_pose_received = True
        self.yaw_nav_mode, self.target_omega_z, self.target_yaw = 0, 0.0, 0.0
        self._servo_target_indices = []
        self._servo_target_angles = []
        self.servo_index = 0
        self.servo_angle = 0
        self.servo_load = 0
        self.servo_error = 0
        self.servo_state_received = False
        self.servo_max_angles = rospy.get_param(self.robot_ns + '/servo_info/max_angles', 1400)
        self.servo_min_angles = rospy.get_param(self.robot_ns + '/servo_info/min_angles', -150)
        servo_max_load = rospy.get_param(self.robot_ns + '/servo_info/max_load', 350)
        self.grasp_contact_load_threshold = rospy.get_param(
            '~grasp_contact_load_threshold', servo_max_load - 50
        )
        self.grasp_contact_confirm_samples = int(rospy.get_param(
            '~grasp_contact_confirm_samples', 2
        ))
        self.grasp_release_offset = int(rospy.get_param('~grasp_release_offset', 10))
        if (self.grasp_contact_load_threshold <= 0
                or self.grasp_contact_confirm_samples < 1
                or self.grasp_release_offset < 0
                or self.servo_min_angles > self.servo_max_angles):
            raise ValueError('Invalid gripper contact protection parameters.')
        self._servo_lock = threading.RLock()
        self._servo_closing_active = False
        self._servo_contact_count = 0
        self._servo_contact_latched = False
        self._servo_error_latched = False
        self._seq = 0

        self._debug_last_requested_target = 0
        self._debug_last_published_target = 0
        self._debug_last_safe_target = -999999
        self._debug_command_direction = 0
        self._gripper_debug_publishers = {
            name: rospy.Publisher(self.robot_ns + '/gripper_debug/' + name,
                                  msg_type, queue_size=1)
            for name, msg_type in (
                ('closing_active', Bool),
                ('contact_count', UInt8),
                ('contact_latched', Bool),
                ('error_latched', Bool),
                ('load_threshold', Int32),
                ('confirm_samples', UInt8),
                ('last_requested_target', Int32),
                ('last_published_target', Int32),
                ('last_safe_target', Int32),
                ('command_direction', Int8),
                ('contact_condition', Bool),
            )
        }

        # Callbacks may run as soon as a subscription is registered.
        self._servo_states_sub = rospy.Subscriber(
            self.robot_ns + '/servo/states', ServoStates,
            self._callback_servo_states, queue_size=1,
        )
        self._target_pose_info_sub = rospy.Subscriber(
            self.robot_ns + '/target_pose/info',
            PoseStamped,
            self._callback_target_pose_info,
            queue_size=1,
        )
        self._target_pose_trigger_sub = rospy.Subscriber(
            self.robot_ns + '/target_pose/trigger',
            Empty,
            self._callback_target_pose_trigger,
            queue_size=1,
        )
        self._servo_target_states_info_sub = rospy.Subscriber(
            self.robot_ns + '/servo/target_states/info',
            ServoControlCmd,
            self._callback_servo_target_states_info,
            queue_size=1,
        )
        self._servo_target_states_trigger_sub = rospy.Subscriber(
            self.robot_ns + '/servo/target_states/trigger',
            Empty,
            self._callback_servo_target_states_trigger,
            queue_size=1,
        )
        self._servo_return_trigger_sub = rospy.Subscriber(
            self.robot_ns + '/servo/return/trigger',
            Empty,
            self._callback_servo_return_trigger,
            queue_size=1,
        )
        self._add_module_trigger_sub = rospy.Subscriber(
            self.robot_ns + '/add_extra_module/trigger',
            Empty,
            self._callback_add_module_trigger,
            queue_size=1,
        )
        self._remove_module_trigger_sub = rospy.Subscriber(
            self.robot_ns + '/remove_extra_module/trigger',
            Empty,
            self._callback_remove_module_trigger,
            queue_size=1,
        )

    def _commands_enabled(self, command_name):
        if self.allow_remote_commands:
            return True
        rospy.logwarn_throttle(
            5.0,
            'Ignoring remote %s command because ~allow_remote_commands is false.',
            command_name,
        )
        return False

    # def _callback_nav_info(self, msg):
    #     self.x_y_mode= msg.pos_xy_nav_mode
    #     self.target_x= msg.target_pos_x
    #     self.target_y= msg.target_pos_y
    #     self.z_mode= msg.pos_z_nav_mode
    #     self.target_z= msg.target_pos_z
    #     self.yaw_nav_mode = msg.yaw_nav_mode
    #     self.target_omega_z = msg.target_omega_z
    #     self.target_yaw = msg.target_yaw
    # Capture the navigation information from ~drone_ns/target_pose topic
    def _callback_target_pose_info(self, msg):
        self.target_frame = msg.header.frame_id or 'world'
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.target_z = msg.pose.position.z
        self.target_ox = msg.pose.orientation.x
        self.target_oy = msg.pose.orientation.y
        self.target_oz = msg.pose.orientation.z
        self.target_ow = msg.pose.orientation.w
        self.target_pose_received = True

    # Trigger the target_pose navigation topic of drone
    def _callback_target_pose_trigger(self, msg):
        if not self._commands_enabled('target-pose'):
            return
        if not self.target_pose_received:
            rospy.logwarn('Ignoring target trigger: no target pose has been received.')
            return
        self.drone_target_pose(
            self.target_x,
            self.target_y,
            self.target_z,
            self.target_ox,
            self.target_oy,
            self.target_oz,
            self.target_ow,
            frame=self.target_frame,
        )
        rospy.loginfo('Published the bridged target pose.')

    def _callback_nav_trigger(self, msg):
        if not self._commands_enabled('navigation'):
            return
        self.drone_nav_info(
            self.x_y_mode,
            self.target_x,
            self.target_y,
            self.z_mode,
            self.target_z,
            self.yaw_nav_mode,
            self.target_omega_z,
            self.target_yaw,
        )
        rospy.loginfo('Published the bridged navigation command.')

    # Keep remote requests separate from measured local feedback.
    def _callback_servo_target_states_info(self, msg):
        with self._servo_lock:
            self._servo_target_indices = list(msg.index)
            self._servo_target_angles = list(msg.angles)

    def _callback_servo_target_states_trigger(self, msg):
        if not self._commands_enabled('servo-target'):
            return
        # Preserve the existing info/trigger settling delay outside the safety lock.
        rospy.sleep(0.1)
        with self._servo_lock:
            if not self._servo_target_angles or not self._servo_target_indices:
                rospy.logwarn('Ignoring servo trigger: no complete target has been received.')
                return
            if len(self._servo_target_angles) != len(self._servo_target_indices):
                rospy.logwarn('Ignoring servo trigger: index/angle lengths differ (%d/%d).',
                              len(self._servo_target_indices), len(self._servo_target_angles))
                return
            self.servo_target_cmd(int(self._servo_target_indices[0]),
                                  int(self._servo_target_angles[0]))

    def _callback_servo_return_trigger(self, msg):
        if not self._commands_enabled('servo-return'):
            return
        with self._servo_lock:
            if not self.servo_state_received:
                rospy.logwarn('Ignoring servo return before receiving a local servo state.')
                return
            self._reset_servo_guard()
            self.servo_target_cmd(self.servo_index, self.servo_max_angles)

    def _clamp_servo_target(self, target_angle):
        target = int(max(self.servo_min_angles, min(self.servo_max_angles, target_angle)))
        if target != target_angle:
            rospy.logwarn_throttle(1.0, 'Gripper mechanical-limit clamp: %s -> %s.',
                                   target_angle, target)
        return target

    def _publish_local_servo_target_raw(self, target_index, target_angle):
        """Physical output, called only under _servo_lock after safety decisions."""
        command = ServoControlCmd()
        command.index = [target_index]
        command.angles = [self._clamp_servo_target(target_angle)]
        self.pub_servo_target.publish(command)
        self._debug_last_published_target = command.angles[0]

    def _publish_gripper_debug(self):
        """Publish a feedback-clocked snapshot under _servo_lock, after safety actions."""
        snapshot = {
            'closing_active': Bool(data=self._servo_closing_active),
            'contact_count': UInt8(data=self._servo_contact_count),
            'contact_latched': Bool(data=self._servo_contact_latched),
            'error_latched': Bool(data=self._servo_error_latched),
            'load_threshold': Int32(data=int(self.grasp_contact_load_threshold)),
            'confirm_samples': UInt8(data=self.grasp_contact_confirm_samples),
            'last_requested_target': Int32(data=self._debug_last_requested_target),
            'last_published_target': Int32(data=self._debug_last_published_target),
            'last_safe_target': Int32(data=self._debug_last_safe_target),
            'command_direction': Int8(data=self._debug_command_direction),
            'contact_condition': Bool(data=abs(self.servo_load) >= self.grasp_contact_load_threshold),
        }
        for name, message in snapshot.items():
            self._gripper_debug_publishers[name].publish(message)

    def _reset_servo_guard(self):
        """Only the explicit return trigger resets protection, under _servo_lock."""
        if self._servo_contact_latched or self._servo_error_latched:
            rospy.loginfo('Gripper explicit return clears the local protection latch.')
        self._servo_contact_latched = False
        self._servo_error_latched = False
        self._servo_contact_count = 0
        self._servo_closing_active = False

    def servo_target_cmd(self, target_index, target_angle):
        """The sole gate for local physical targets and bridged gripper requests."""
        with self._servo_lock:
            self._debug_last_requested_target = int(target_angle)
            self._debug_command_direction = 0
            if self.servo_state_received and target_index == self.servo_index:
                # Mirror the bounded request's direction, even when the gate rejects it.
                # Do not call the control clamp here: it also logs mechanical-limit events.
                debug_target = int(max(self.servo_min_angles, min(self.servo_max_angles, target_angle)))
                self._debug_command_direction = (
                    int(debug_target > self.servo_angle) - int(debug_target < self.servo_angle)
                )
            if not self.servo_state_received:
                rospy.logwarn_throttle(1.0, 'Ignoring servo target before receiving a local servo state.')
                return False
            if target_index != self.servo_index:
                rospy.logwarn_throttle(1.0, 'Ignoring servo target without matching local feedback.')
                return False
            if self._servo_contact_latched or self._servo_error_latched:
                # Even a larger target can be a delayed pre-contact closing request.
                rospy.logwarn_throttle(
                    1.0, 'Gripper target rejected: contact/error protection is latched; use return trigger.'
                )
                return False
            target_angle = self._clamp_servo_target(target_angle)
            if target_angle < self.servo_angle:
                if self.servo_error != 0:
                    self._abort_servo_error()
                    return False
                if not self._servo_closing_active:
                    self._servo_contact_count = 0
                self._servo_closing_active = True
            else:
                self._servo_closing_active = False
                self._servo_contact_count = 0
            self._publish_local_servo_target_raw(target_index, target_angle)
            return True

    def _abort_servo_error(self):
        """Cancel the stored goal and require opening/return before another close."""
        self._servo_error_latched = True
        self._servo_closing_active = False
        self._servo_contact_count = 0
        self._publish_local_servo_target_raw(self.servo_index, self.servo_angle)
        rospy.logerr('Gripper local servo error 0x%02x at position %s; closing aborted.',
                     self.servo_error, self.servo_angle)

    def _callback_servo_states(self, msg):
        if not msg.servos:
            rospy.logwarn_throttle(5.0, 'Received an empty local servo state message.')
            with self._servo_lock:
                self._publish_gripper_debug()
            return
        with self._servo_lock:
            state = msg.servos[0]
            self.servo_index = state.index
            self.servo_angle = state.angle
            self.servo_load = state.load
            self.servo_error = state.error
            self.servo_state_received = True
            if not self._servo_closing_active:
                self._publish_gripper_debug()
                return
            # Local ServoState.error is the driver's hardware_error_status_ bitmask.
            if self.servo_error != 0:
                self._abort_servo_error()
                self._publish_gripper_debug()
                return
            if abs(self.servo_load) >= self.grasp_contact_load_threshold:
                self._servo_contact_count += 1
            else:
                self._servo_contact_count = 0
            if self._servo_contact_count >= self.grasp_contact_confirm_samples:
                self._servo_contact_latched = True
                self._servo_closing_active = False
                safe_target = self._clamp_servo_target(self.servo_angle + self.grasp_release_offset)
                self._debug_last_safe_target = safe_target
                self._publish_local_servo_target_raw(self.servo_index, safe_target)
                rospy.loginfo(
                    'Gripper contact detected locally: load=%s raw, estimated_current=%.1f mA, '
                    'present_position=%s, safe_target=%s.', self.servo_load,
                    abs(self.servo_load) * 2.69, self.servo_angle, safe_target,
                )
            elif self.servo_angle <= self.servo_min_angles:
                self._servo_closing_active = False
                self._servo_contact_count = 0
                self._publish_local_servo_target_raw(self.servo_index, self.servo_angle)
                rospy.logwarn('Gripper reached minimum position %s without confirmed contact.',
                              self.servo_angle)
            self._publish_gripper_debug()

    # trigger for add extra module
    def _callback_add_module_trigger(self, msg):
        if not self._commands_enabled('add-module'):
            return
        rospy.sleep(0.1)
        self.drone_basic.call_add_extra_module(1, 'brick', 'main_body')

    # trigger for remove extra module
    def _callback_remove_module_trigger(self, msg):
        if not self._commands_enabled('remove-module'):
            return
        rospy.sleep(0.1)
        self.drone_basic.call_add_extra_module(-1, 'brick', 'main_body')

    def drone_nav_info(self, x_y_mode, x, y, z_mode, z, yaw_mode, omega_z, yaw):
        flight_nav_msg = FlightNav()
        flight_nav_msg.header.seq = self._seq
        self._seq += 1
        flight_nav_msg.header.stamp = rospy.Time.now()
        flight_nav_msg.header.frame_id = 'world'

        flight_nav_msg.control_frame = 0
        flight_nav_msg.target = 0
        flight_nav_msg.pos_xy_nav_mode = x_y_mode
        flight_nav_msg.target_pos_x = x
        flight_nav_msg.target_vel_x = 0.0
        flight_nav_msg.target_acc_x = 0.0
        flight_nav_msg.target_pos_y = y
        flight_nav_msg.target_vel_y = 0.0
        flight_nav_msg.target_acc_y = 0.0
        flight_nav_msg.yaw_nav_mode = yaw_mode
        flight_nav_msg.target_omega_z = omega_z
        flight_nav_msg.target_yaw = yaw
        flight_nav_msg.pos_z_nav_mode = z_mode
        flight_nav_msg.target_pos_z = z
        flight_nav_msg.target_vel_z = 0.0
        flight_nav_msg.target_pos_diff_z = 0.0

        self.pub_drone_nav.publish(flight_nav_msg)

    def drone_target_pose(self, x, y, z, ox, oy, oz, ow, frame='world'):
        """Forward a target in its declared frame; do not reinterpret coordinates."""
        drone_target_pose = PoseStamped()
        drone_target_pose.header.stamp = rospy.Time.now()
        drone_target_pose.header.frame_id = frame
        drone_target_pose.pose.position.x = x
        drone_target_pose.pose.position.y = y
        drone_target_pose.pose.position.z = z
        drone_target_pose.pose.orientation.x = ox
        drone_target_pose.pose.orientation.y = oy
        drone_target_pose.pose.orientation.z = oz
        drone_target_pose.pose.orientation.w = ow
        self.pub_drone_target.publish(drone_target_pose)


def main():
    """Run the ROS node."""
    rospy.init_node('event_trigger')
    print('event_trigger node started.')
    print('event_trigger node started.')
    print('event_trigger node started.')
    print('event_trigger node started.')
    print('event_trigger node started.')
    _node = EventtriggerNode()
    rospy.spin()

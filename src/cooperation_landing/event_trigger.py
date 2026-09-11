"""Event trigger implementation."""

import rospy
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped
from spinal.msg import ServoControlCmd
from std_msgs.msg import Empty

from cooperation_landing.drone_basic_function import DroneBasic
from cooperation_landing.gripper.gripper_move import GripperMoveNode


class EventtriggerNode:
    def __init__(self):
        rospy.logdebug('Initializing network event bridge.')
        self.robot_ns = ('/' + str(rospy.get_param('~robot_ns', 'xuanwu')).strip('/')).rstrip('/')
        self.allow_remote_commands = bool(rospy.get_param('~allow_remote_commands', False))
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

        self.gripper_move = GripperMoveNode()
        self.drone_basic = DroneBasic()

        self.x_y_mode, self.z_mode = 0, 0
        self.target_x, self.target_y, self.target_z = 0.0, 0.0, 0.0
        self.target_ox, self.target_oy, self.target_oz, self.target_ow = 0.0, 0.0, 0.0, 1.0
        self.target_frame = 'world'
        self.target_pose_received = False
        self.yaw_nav_mode, self.target_omega_z, self.target_yaw = 0, 0.0, 0.0
        self.servo_index = []
        self.servo_angle = []
        self._seq = 0

        # Callbacks may run as soon as a subscription is registered.
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

    # Get the /target_state_info from dog side
    def _callback_servo_target_states_info(self, msg):
        self.servo_index = msg.index
        self.servo_angle = msg.angles
        # print(f'{self.servo_index} {self.servo_angle}')

    # If the trigger from dog side published, the trigger for drone side will respond
    def _callback_servo_target_states_trigger(self, msg):
        if not self._commands_enabled('servo-target'):
            return
        rospy.sleep(0.1)
        if not self.servo_angle or not self.servo_index:
            rospy.logwarn('Ignoring servo trigger: no complete target has been received.')
            return
        if len(self.servo_angle) != len(self.servo_index):
            rospy.logwarn(
                'Ignoring servo trigger: index/angle lengths differ (%d/%d).',
                len(self.servo_index),
                len(self.servo_angle),
            )
            return
        servo_angle = int(self.servo_angle[0])
        self.gripper_move.servo_target_cmd(int(self.servo_index[0]), servo_angle)

    # Trigger for gripper to return zero point
    def _callback_servo_return_trigger(self, msg):
        if not self._commands_enabled('servo-return'):
            return
        rospy.sleep(0.2)
        self.gripper_move.return_zero()

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

    # def gripper_move_event(self, servo_index, servo_angle):
    #     servo_target_cmd = ServoControlCmd()
    #     servo_target_cmd.index = self.servo_index
    #     servo_target_cmd.angles = self.servo_angle
    #     time.sleep(0.1)
    #     self.pub_servo_target.publish(servo_target_cmd)
    #     print(f'servo_target_cmd:{servo_target_cmd}')
    #

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
    _node = EventtriggerNode()
    rospy.spin()

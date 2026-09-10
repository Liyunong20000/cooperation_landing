"""Interactive keyboard control for the ground robot.

The node supports the real robot's ``cmd_vel`` interface and Gazebo's
``ModelState`` interface.  Motion commands are fixed-speed and do not
accumulate between key presses.
"""

import math
import sys
import termios

import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
from std_srvs.srv import Trigger

from cooperation_landing.terminal import get_key

HELP = """
Instruction:

---------------------------

     q           w           e
(turn left)  (forward)  (turn right)

     a           s           d          j       k       l
(move left)  (backward) (move right)  (stop) (stand)  (sit)

     u                i                 o
(pitch 0 deg)   (pitch -15 deg)   (pitch -30 deg)

Please don't have caps lock on.
CTRL+C to quit
---------------------------
"""


def print_message(message, message_len=50):
    print(message.ljust(message_len) + '\r', end='', flush=True)


def call_trigger(service_name):
    """Call a Trigger service and return whether the command succeeded."""
    try:
        response = rospy.ServiceProxy(service_name, Trigger)()
    except rospy.ServiceException as exc:
        rospy.logerr('Service call to %s failed: %s', service_name, exc)
        return False

    if not response.success:
        rospy.logwarn('%s rejected the command: %s', service_name, response.message)
    return response.success


def body_pose_with_pitch_deg(pitch_deg):
    pose = Pose()
    pitch_rad = math.radians(pitch_deg)
    pose.orientation.y = math.sin(pitch_rad / 2.0)
    pose.orientation.w = math.cos(pitch_rad / 2.0)
    return pose


class KeyboardControl:
    """Translate key presses into robot or simulator motion commands."""

    def __init__(self):
        mode = rospy.get_param('~keyboard_mode', 1)
        self.real_robot = str(mode).lower() not in {'0', 'false', 'sim', 'simulation'}

        robot_ns = str(rospy.get_param('~ground_robot_ns', '/go1')).strip('/')
        self.robot_ns = f'/{robot_ns}' if robot_ns else ''
        self.xy_vel = max(
            0.0,
            float(rospy.get_param('~xy_vel', 0.2 if self.real_robot else 0.5)),
        )
        self.yaw_vel = max(
            0.0,
            float(rospy.get_param('~yaw_vel', 0.4 if self.real_robot else 2.5)),
        )
        self.model_name = rospy.get_param('~gazebo_model_name', 'go1_gazebo')
        self.reference_frame = rospy.get_param('~gazebo_reference_frame', 'base')

        self.body_pose_pub = rospy.Publisher(f'{self.robot_ns}/body_pose', Pose, queue_size=1)
        if self.real_robot:
            self.nav_pub = rospy.Publisher(f'{self.robot_ns}/cmd_vel', Twist, queue_size=1)
        else:
            self.nav_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

    def _publish_twist(self, x=0.0, y=0.0, yaw=0.0):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = yaw

        if self.real_robot:
            self.nav_pub.publish(twist)
            return

        model_state = ModelState()
        model_state.model_name = self.model_name
        model_state.reference_frame = self.reference_frame
        model_state.twist = twist
        self.nav_pub.publish(model_state)

    def stop(self):
        self._publish_twist()

    def publish_motion(self, key):
        commands = {
            'w': (self.xy_vel, 0.0, 0.0, 'send +x velocity command'),
            's': (-self.xy_vel, 0.0, 0.0, 'send -x velocity command'),
            'a': (0.0, self.xy_vel, 0.0, 'send +y velocity command'),
            'd': (0.0, -self.xy_vel, 0.0, 'send -y velocity command'),
            'q': (0.0, 0.0, self.yaw_vel, 'send +yaw velocity command'),
            'e': (0.0, 0.0, -self.yaw_vel, 'send -yaw velocity command'),
        }
        if key not in commands:
            return False

        x, y, yaw, message = commands[key]
        self._publish_twist(x, y, yaw)
        print_message(message)
        return True

    def run(self):
        if not sys.stdin.isatty():
            rospy.logfatal('keyboard_control requires an interactive terminal')
            return

        settings = termios.tcgetattr(sys.stdin)
        print(HELP)
        try:
            while not rospy.is_shutdown():
                key = get_key(settings)
                if key == '\x03':
                    break
                if self.publish_motion(key):
                    continue

                if key == 'j':
                    self.stop()
                    self.body_pose_pub.publish(body_pose_with_pitch_deg(0.0))
                    print_message('stop and reset body pose')
                elif key == 'k':
                    success = call_trigger(f'{self.robot_ns}/stand')
                    print_message('stand' if success else 'stand command failed')
                elif key == 'l':
                    success = call_trigger(f'{self.robot_ns}/sit')
                    print_message('sit' if success else 'sit command failed')
                elif key == 'u':
                    self.body_pose_pub.publish(body_pose_with_pitch_deg(0.0))
                    print_message('set body pitch to 0 degrees')
                elif key == 'i':
                    self.body_pose_pub.publish(body_pose_with_pitch_deg(-15.0))
                    print_message('set body pitch to -15 degrees')
                elif key == 'o':
                    self.body_pose_pub.publish(body_pose_with_pitch_deg(-30.0))
                    print_message('set body pitch to -30 degrees')
        finally:
            self.stop()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            print()


def main():
    rospy.init_node('keyboard_control')
    KeyboardControl().run()

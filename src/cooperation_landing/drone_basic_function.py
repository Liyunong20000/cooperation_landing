"""Drone basic function implementation."""

import time

import rospy
import tf.transformations as tft
from aerial_robot_model.srv import AddExtraModule
from aerial_robot_msgs.msg import FlightNav
from apriltag_ros.msg import AprilTagDetectionArray
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Inertia, PoseStamped, Transform
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, UInt8


class DroneBasic:
    """Aerial telemetry and command interface using ``~robot_ns`` (default: /xuanwu).

    Initialize rospy first. Command publication is asynchronous and does not
    acknowledge completion; the task controller must check telemetry.
    """

    def __init__(self):
        rospy.logdebug('Initializing aerial robot interface.')

        self.drone_x, self.drone_y, self.drone_z = 0.0, 0.0, 0.0
        self.drone_qx, self.drone_qy, self.drone_qz, self.drone_qw = 0.0, 0.0, 0.0, 1.0
        self.drone_roll, self.drone_pitch, self.drone_yaw = 0.0, 0.0, 0.0
        self.takeoff_x, self.takeoff_y, self.takeoff_z, self.takeoff_yaw = 0.0, 0.0, 0.0, 0.0
        self.tag_target_x, self.tag_target_y, self.tag_target_z = 0.0, 0.0, 0.0
        self.tag_target_qx, self.tag_target_qy, self.tag_target_qz, self.tag_target_qw = (
            0.0,
            0.0,
            0.0,
            1.0,
        )
        self.tag_target_roll, self.tag_target_pitch, self.tag_target_yaw = 0.0, 0.0, 0.0

        self.tag_info = None
        self.drone_state = 0
        self.odom_received = False

        self.robot_ns = ('/' + str(rospy.get_param('~robot_ns', 'xuanwu')).strip('/')).rstrip('/')

        # Subscribe and publish.

        self.pub_drone_nav = rospy.Publisher(self.robot_ns + '/uav/nav', FlightNav, queue_size=10)
        self.pub_drone_target = rospy.Publisher(
            self.robot_ns + '/target_pose/info', PoseStamped, queue_size=10
        )
        self.pub_drone_target_trigger = rospy.Publisher(
            self.robot_ns + '/target_pose/trigger', Empty, queue_size=10
        )

        self.pub_drone_start = rospy.Publisher(
            self.robot_ns + '/teleop_command/start', Empty, queue_size=10
        )
        self.pub_drone_add_module_trigger = rospy.Publisher(
            self.robot_ns + '/add_extra_module/trigger', Empty, queue_size=10
        )
        self.pub_drone_remove_module_trigger = rospy.Publisher(
            self.robot_ns + '/remove_extra_module/trigger', Empty, queue_size=10
        )
        self.pub_drone_takeoff = rospy.Publisher(
            self.robot_ns + '/teleop_command/takeoff', Empty, queue_size=10
        )
        self.pub_drone_land = rospy.Publisher(
            self.robot_ns + '/teleop_command/land', Empty, queue_size=10
        )

        self.extra_module = rospy.ServiceProxy(self.robot_ns + '/add_extra_module', AddExtraModule)

        # Callbacks may run as soon as a subscription is registered.
        self._drone_position_sub = rospy.Subscriber(
            self.robot_ns + '/uav/cog/odom', Odometry, self._callback_drone_position, queue_size=1
        )
        self._drone_state_sub = rospy.Subscriber(
            self.robot_ns + '/flight_state', UInt8, self._callback_drone_state, queue_size=1
        )
        self._tag_info_sub = rospy.Subscriber(
            self.robot_ns + '/tag_detections',
            AprilTagDetectionArray,
            self._callback_tag_info,
            queue_size=1,
        )

    # Get the drone position from ~/uav/cog/odom
    def _callback_drone_position(self, msg):
        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y
        self.drone_z = msg.pose.pose.position.z
        self.drone_qx = msg.pose.pose.orientation.x
        self.drone_qy = msg.pose.pose.orientation.y
        self.drone_qz = msg.pose.pose.orientation.z
        self.drone_qw = msg.pose.pose.orientation.w
        self.drone_roll, self.drone_pitch, self.drone_yaw = tft.euler_from_quaternion(
            [self.drone_qx, self.drone_qy, self.drone_qz, self.drone_qw]
        )
        self.odom_received = True

    def wait_for_odom(self, timeout_s=None):
        """Wait until at least one COG odometry message has arrived."""
        if self.odom_received:
            return True
        if timeout_s is None:
            timeout_s = rospy.get_param('~odom_wait_timeout_s', 5.0)
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.odom_received:
                return True
            rate.sleep()
        rospy.logerr('No odometry received on %s/uav/cog/odom.', self.robot_ns)
        return False

    # Get the drone state
    def _callback_drone_state(self, msg):
        self.drone_state = msg.data

    def _callback_tag_info(self, msg):
        self.tag_info = msg
        # print(f'self.tag_info: {self.tag_info}')

    # Get the tag info and value the variable
    def tag_position(self, data, target_id):
        if data is None:
            return False
        for det in data.detections:
            # print(f'{det}')
            # print(f'{det.id}')
            if target_id in det.id:
                pose = det.pose.pose.pose
                self.tag_target_x = pose.position.x
                self.tag_target_y = pose.position.y
                self.tag_target_z = pose.position.z
                self.tag_target_qx = pose.orientation.x
                self.tag_target_qy = pose.orientation.y
                self.tag_target_qz = pose.orientation.z
                self.tag_target_qw = pose.orientation.w
                self.tag_target_roll, self.tag_target_pitch, self.tag_target_yaw = (
                    tft.euler_from_quaternion(
                        [
                            self.tag_target_qx,
                            self.tag_target_qy,
                            self.tag_target_qz,
                            self.tag_target_qw,
                        ]
                    )
                )
                return True

        return False

        # print(f'x:{self.tag_target_x},y: {self.tag_target_y},z: {self.tag_target_z}')
        # print(f'x:{self.tag_target_roll},y: {self.tag_target_pitch},z: {self.tag_target_yaw}')

    # arming the drone
    def drone_start(self):  # Use to takeoff
        rospy.sleep(0.5)
        rospy.loginfo('Publishing start command...')
        empty_msg = Empty()
        self.pub_drone_start.publish(empty_msg)

    # Takeoff the drone
    def drone_takeoff(self):  # Use to takeoff
        rospy.sleep(0.5)
        rospy.loginfo('Publishing takeoff command...')
        empty_msg = Empty()
        self.pub_drone_takeoff.publish(empty_msg)

    # Land the drone
    def drone_land(self):  # Use to land
        rospy.sleep(0.5)
        rospy.loginfo('Publishing land command...')
        empty_msg = Empty()
        self.pub_drone_land.publish(empty_msg)

    # Drone navigation by /uav/nav topic
    def drone_nav(self, x, y, z):
        """Publish an absolute world-frame position target, in metres."""
        flight_nav_msg = FlightNav()
        # flight_nav_msg.header.seq = self._seq
        # self._seq += 1
        flight_nav_msg.header.stamp = rospy.Time.now()
        flight_nav_msg.header.frame_id = 'world'

        flight_nav_msg.control_frame = 0
        flight_nav_msg.target = 0
        flight_nav_msg.pos_xy_nav_mode = 2
        flight_nav_msg.target_pos_x = x
        flight_nav_msg.target_vel_x = 0.0
        flight_nav_msg.target_acc_x = 0.0
        flight_nav_msg.target_pos_y = y
        flight_nav_msg.target_vel_y = 0.0
        flight_nav_msg.target_acc_y = 0.0
        flight_nav_msg.yaw_nav_mode = 0
        flight_nav_msg.target_omega_z = 0.0
        flight_nav_msg.target_yaw = 0.0
        flight_nav_msg.pos_z_nav_mode = 2
        flight_nav_msg.target_pos_z = z
        flight_nav_msg.target_vel_z = 0.0
        flight_nav_msg.target_pos_diff_z = 0.0

        self.pub_drone_nav.publish(flight_nav_msg)

    # Drone navigation by /drone_ns/target_pose topic
    def drone_target(self, frame, x, y, z, ox, oy, oz, ow):
        """Send a bridged pose and trigger, with metres and an xyzw quaternion.

        ``frame`` is preserved by the event bridge. No TF conversion is applied.
        """
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
        rospy.sleep(0.1)
        self.pub_drone_target.publish(drone_target_pose)
        rospy.sleep(0.2)
        self.drone_target_trigger()

    # Send the trigger of navigation
    def drone_target_trigger(self):
        rospy.sleep(0.1)
        empty_msg = Empty()
        self.pub_drone_target_trigger.publish(empty_msg)

    # The function used to record takeoff position
    def record_takeoff_position(self, x, y, z, yaw):
        self.takeoff_x = x
        self.takeoff_y = y
        self.takeoff_z = z
        self.takeoff_yaw = yaw
        # print(f'takeoff position:{self.takeoff_x}, {self.takeoff_y},{self.takeoff_z}, {self.takeoff_yaw}')

    # After grasping some objects, add the module for the drone to calculate
    def call_add_extra_module(self, action, module_name, parent_link_name):
        try:
            transform = Transform()
            transform.translation.x = 0.0
            transform.translation.y = 0.0
            transform.translation.z = -0.145
            transform.rotation.x = 0.0
            transform.rotation.y = 0.0
            transform.rotation.z = 0.0
            transform.rotation.w = 1.0

            inertia = Inertia()
            inertia.m = 0.150
            inertia.com.x = 0.0
            inertia.com.y = 0.0
            inertia.com.z = 0.0
            inertia.ixx = 0.00009627321
            inertia.ixy = 0.0
            inertia.ixz = 0.0
            inertia.iyy = 0.00017161135
            inertia.iyz = 0.0
            inertia.izz = 0.00010055263

            response = self.extra_module(action, module_name, parent_link_name, transform, inertia)

            return response
        except rospy.ServiceException as e:
            rospy.logerr('AddExtraModule service call failed: %s', e)
            return None

    # Trigger of the drone to add an extra module
    def add_module_trigger(self):
        rospy.sleep(0.1)
        empty_msg = Empty()
        self.pub_drone_add_module_trigger.publish(empty_msg)

    # Trigger of the drone to remove the extra module
    def remove_module_trigger(self):
        rospy.sleep(0.1)
        empty_msg = Empty()
        self.pub_drone_remove_module_trigger.publish(empty_msg)


class DroneBasicSim:
    def __init__(self):
        self.model_name = rospy.get_param('~sim_aerial_model', 'xuanwu')
        model_states_topic = rospy.get_param('~model_states_topic', '/gazebo/model_states')

        self.sim_drone_x, self.sim_drone_y, self.sim_drone_z = 0.0, 0.0, 0.0
        self.sim_drone_qx, self.sim_drone_qy, self.sim_drone_qz, self.sim_drone_qw = (
            0.0,
            0.0,
            0.0,
            1.0,
        )
        self.sim_drone_roll, self.sim_drone_pitch, self.sim_drone_yaw = 0.0, 0.0, 0.0

        # Callbacks may run as soon as a subscription is registered.
        self._drone_position_sub = rospy.Subscriber(
            model_states_topic, ModelStates, self._callback_aerial_robot_pose, queue_size=1
        )

    def _callback_aerial_robot_pose(self, msg):

        if self.model_name not in msg.name:
            rospy.logwarn_throttle(
                5.0, 'DroneBasicSim: model %s is not available in Gazebo.', self.model_name
            )
            return
        index = msg.name.index(self.model_name)
        if index >= len(msg.pose):
            rospy.logwarn_throttle(
                5.0, 'Gazebo model states contain no pose for %s.', self.model_name
            )
            return
        pose = msg.pose[index]
        self.sim_drone_x = pose.position.x
        self.sim_drone_y = pose.position.y
        self.sim_drone_z = pose.position.z
        self.sim_drone_qx = pose.orientation.x
        self.sim_drone_qy = pose.orientation.y
        self.sim_drone_qz = pose.orientation.z
        self.sim_drone_qw = pose.orientation.w

        self.sim_drone_roll, self.sim_drone_pitch, self.sim_drone_yaw = tft.euler_from_quaternion(
            [self.sim_drone_qx, self.sim_drone_qy, self.sim_drone_qz, self.sim_drone_qw]
        )
        # print(f'{self.sim_drone_x},{self.sim_drone_y}')


def main():
    """Run the ROS node."""
    rospy.init_node('drone_basic', anonymous=True)
    _node = DroneBasic()
    rospy.spin()

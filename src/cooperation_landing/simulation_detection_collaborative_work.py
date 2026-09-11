"""Simulation detection collaborative work implementation."""

import math
import time

import numpy as np
import rospy
from aerial_robot_msgs.msg import FlightNav
from apriltag_ros.msg import AprilTagDetectionArray
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, UInt8


class CooperationNode:
    def __init__(self):
        rospy.logdebug('Initializing legacy collaborative simulation node.')

        self.lx, self.ly, self.lz = 0, 0, 0
        self.qx, self.qy, self.qz, self.qw = 0, 0, 0, 1
        self.april_x, self.april_y, self.april_z = 0.0, 0.0, 0.0
        self.april_qx, self.april_qy, self.april_qz, self.april_qw = 0.0, 0.0, 0.0, 1.0

        self.camera2base_x = 0.27
        self.camera2base_y = 0
        self.camera2base_z = 0.137
        self.valve2tag_x, self.valve2tag_y = 1.0, 1.0
        self.valve_x, self.valve_y = 0, 0
        self.april_valve_x, self.april_valve_y, self.april_valve_z = 0, 0, 0

        self.dog_x, self.dog_y, self.dog_z = 2.8, 2.8, 0.0
        self.drone_x, self.drone_y, self.drone_z = 0.0, 0.0, 0.0
        self.takeoff_x, self.takeoff_y, self.takeoff_z = 0.0, 0.0, 0.0

        self.data_array = 0
        self.find_valve_tag = 0
        self.D = 0
        self.time_rece = rospy.Time()
        self._seq = 0
        self.state = 0
        self.beginland = 0
        self.beginfollow = 0
        self.flag = 0

        drone_ns = ('/' + str(rospy.get_param('~robot_ns', 'quadrotor')).strip('/')).rstrip('/')
        ground_robot_ns = ('/' + str(rospy.get_param('~ground_robot_ns', 'go1')).strip('/')).rstrip(
            '/'
        )
        tag_topic = rospy.get_param('~tag_topic', '/tag_detections')
        event_topic = rospy.get_param('~event_topic', '/uavandgr/event')
        model_states_command_topic = rospy.get_param(
            '~gazebo_set_model_state_topic', '/gazebo/set_model_state'
        )

        # Subscribe and publish.
        rospy.Subscriber(tag_topic, AprilTagDetectionArray, self._callback_apriltag, queue_size=1)
        rospy.Subscriber(
            drone_ns + '/uav/cog/odom', Odometry, self._callback_position, queue_size=1
        )
        rospy.Subscriber(drone_ns + '/flight_state', UInt8, self._callback_state, queue_size=1)

        self.pub_drone_nav = rospy.Publisher(drone_ns + '/uav/nav', FlightNav, queue_size=10)
        self.pub_takeoff = rospy.Publisher(
            drone_ns + '/teleop_command/takeoff', Empty, queue_size=10
        )
        self.pub_land = rospy.Publisher(drone_ns + '/teleop_command/land', Empty, queue_size=10)

        self.pub_event = rospy.Publisher(event_topic, UInt8, queue_size=10)
        # simulation: unitree position

        self.pub_sim_pose = rospy.Publisher(model_states_command_topic, ModelState, queue_size=10)

        self.pub_qilin_vel = rospy.Publisher(ground_robot_ns + '/cmd_vel', Twist, queue_size=10)
        self.pub_qilin_pose = rospy.Publisher(ground_robot_ns + '/body_pose', Pose, queue_size=10)

        self.converge_interval = float(rospy.get_param('~converge_interval', 0.05))
        self.above_z = float(rospy.get_param('~above_z', 0.3))
        self.flight_state_timeout_s = max(
            0.1, float(rospy.get_param('~flight_state_timeout_s', 30.0))
        )
        self.tag_alignment_attempts = max(1, int(rospy.get_param('~tag_alignment_attempts', 50)))
        self.sim_ground_model = str(rospy.get_param('~sim_ground_model', 'unitree'))
        self.allow_takeoff = bool(rospy.get_param('~allow_takeoff', False))

        # rospy.set_param('/move_parameter', 2)
        # self.move_parameter = rospy.get_param("/move_parameter")
        # rospy.set_param('/pose_parameter', 0.05)
        # self.pose_parameter = rospy.get_param("/pose_parameter")

    def _callback_apriltag(self, data):
        rospy.Time.now()
        # print(f'apriltag:{current_time.to_sec()}')

        # get the apriltag`s position information compare with camera coordination
        if data.detections:
            self.find_valve_tag = self.find_target_tag(data.detections, 1)

            # rospy.loginfo("latest arigtarg timestamp: {}".format(data.header.stamp.to_sec()))
            a = data.detections[0]
            self.april_x = a.pose.pose.pose.position.x
            self.april_y = a.pose.pose.pose.position.y
            self.april_qx = a.pose.pose.pose.orientation.x
            self.april_qy = a.pose.pose.pose.orientation.y
            self.april_qz = a.pose.pose.pose.orientation.z
            self.april_qw = a.pose.pose.pose.orientation.w

            # b = data.detections[1]
            # self.april_valve_x = b.pose.pose.pose.position.x
            # self.april_valve_y = b.pose.pose.pose.position.y
            # if len(data.detections) > 1 :
            # print(f'222222222222')
            # else:
            #     print(f'111111111111')

            if self.beginfollow == 1:
                self.lx = -2 * self.april_y
                self.ly = 2 * self.april_x
                self.april_z = 0.05 * self.quaternion_to_euler_angle(
                    self.april_qx, self.april_qy, self.april_qz, self.april_qw
                )
                # self.qx = self.pose_parameter * self.april_qx
                # self.qy = self.pose_parameter * self.april_qy
                # self.qz = self.pose_parameter * self.april_qz
                # self.qw = self.pose_parameter * self.april_qw
                rospy.logdebug_throttle(
                    1.0, 'Ground-robot alignment velocity: x=%.3f y=%.3f', self.lx, self.ly
                )
                if abs(self.lx) < 5 and abs(self.ly) < 5:
                    self.qilin_cmd_vel(self.lx, self.ly, 0, 0, self.april_z)
                    # self.qilin_body_pose(self.qx, self.qy, self.qz, self.qw)
                    # self.qilin_body_pose(self.april_qx, self.april_qy, self.april_qz, self.april_qw)
        else:
            if self.beginfollow == 1:
                self.qilin_cmd_vel(0, 0, 0, 0, 0)

            # self.qilin_body_pose(0, 0, 0, 1)

    def _callback_position(self, odom_msg):
        self.drone_x = odom_msg.pose.pose.position.x
        self.drone_y = odom_msg.pose.pose.position.y
        self.drone_z = odom_msg.pose.pose.position.z

    def _callback_state(self, msg):
        self.state = msg.data

    def find_target_tag(self, data, target_id):
        for detection in data:
            if target_id in detection.id:
                self.april_valve_x = -detection.pose.pose.pose.position.y
                self.april_valve_y = detection.pose.pose.pose.position.x
                self.april_valve_z = detection.pose.pose.pose.position.z
                self.valve_x = self.april_valve_x + self.valve2tag_x
                self.valve_y = self.april_valve_y + self.valve2tag_y
                return 1
        return 0

    # drone takeoff
    def takeoff(self):
        if not self.allow_takeoff:
            rospy.logerr('Takeoff is disabled because ~allow_takeoff is false.')
            return False
        time.sleep(0.5)
        rospy.loginfo('Publishing takeoff command...')
        empty_msg = Empty()
        self.pub_takeoff.publish(empty_msg)
        return True

    # drone land
    def land(self):
        time.sleep(0.5)
        rospy.loginfo('Publishing land command...')
        empty_msg = Empty()
        self.pub_land.publish(empty_msg)

    def publish_event(self, x):
        event_msgs = UInt8()
        event_msgs.data = x
        self.pub_event.publish(event_msgs)

    def drone_landing_detection(self, i):
        r = rospy.Rate(i)
        number = i
        while not rospy.is_shutdown():
            number = number - 1
            if math.sqrt(self.april_x**2 + self.april_y**2) < 0.03 and abs(self.april_z) < 10:
                i = i - 1
            if number == 0:
                break
            r.sleep()
        self.flag = i

    def drone_landing_condition(self):
        while not rospy.is_shutdown():
            i = 1
            plus = 0
            self.flag = 0

            while i > 0:
                i = i - 1
                self.drone_landing_detection(10)
                plus = plus + self.flag
                rospy.logdebug('Landing detector accumulator: %d', plus)
            if plus == 0:
                self.beginfollow = 0
                self.publish_event(3)
                self.qilin_cmd_vel(0, 0, 0, 0, 0)
                rospy.loginfo('Ground robot aligned for landing.')

                break

    def sim_pose(self, px, py, ox, oy, oz, ow):

        sim_pose = ModelState()
        sim_pose.model_name = self.sim_ground_model
        sim_pose.pose.position.x = px
        sim_pose.pose.position.y = py
        sim_pose.pose.orientation.x = ox
        sim_pose.pose.orientation.y = oy
        sim_pose.pose.orientation.z = oz
        sim_pose.pose.orientation.w = ow
        sim_pose.reference_frame = 'world'
        self.pub_sim_pose.publish(sim_pose)

    def drone_nav_info(self, x, y, z):
        flight_nav_msg = FlightNav()
        flight_nav_msg.header.seq = self._seq
        self._seq += 1
        flight_nav_msg.header.stamp = rospy.Time.now()
        flight_nav_msg.header.frame_id = 'world'
        flight_nav_msg.pos_xy_nav_mode = 2
        flight_nav_msg.target_pos_x = x
        flight_nav_msg.target_pos_y = y
        flight_nav_msg.pos_z_nav_mode = 2
        flight_nav_msg.target_pos_z = z
        self.pub_drone_nav.publish(flight_nav_msg)

    def qilin_cmd_vel(self, lx, ly, ax, ay, az):
        qilin_cmd_vel = Twist()
        qilin_cmd_vel.linear.x = lx
        qilin_cmd_vel.linear.y = ly
        qilin_cmd_vel.angular.x = ax
        qilin_cmd_vel.angular.y = ay
        qilin_cmd_vel.angular.z = az

        self.pub_qilin_vel.publish(qilin_cmd_vel)

    def qilin_body_pose(self, qx, qy, qz, qw):
        qilin_body_pose = Pose()
        qilin_body_pose.orientation.x = qx
        qilin_body_pose.orientation.y = qy
        qilin_body_pose.orientation.z = qz
        qilin_body_pose.orientation.w = qw

        self.pub_qilin_pose.publish(qilin_body_pose)

    def quaternion_to_euler_angle(self, x, y, z, w):
        R = np.array(
            [
                [1 - 2 * y**2 - 2 * z**2, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y],
                [2 * x * y + 2 * w * z, 1 - 2 * x**2 - 2 * z**2, 2 * y * z - 2 * w * x],
                [2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x**2 - 2 * y**2],
            ]
        )
        # theta_x = math.degrees(np.arctan2(R[2, 1], R[2, 2]))
        # theta_y = math.degrees(np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2)))
        theta_z = math.degrees(np.arctan2(R[1, 0], R[0, 0]))
        return theta_z

    def come_back(self):
        deadline = time.monotonic() + self.flight_state_timeout_s
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.state == 5:
                break
            rospy.sleep(0.1)
        if self.state != 5:
            rospy.logerr('Timed out waiting for flight state 5 during return.')
            return False
        self.publish_event(2)
        rospy.loginfo('Moving above the takeoff height.')
        # self.converge(self.takeoff_x, self.takeoff_y, tz)
        rospy.sleep(7)
        self.beginfollow = 1
        rospy.loginfo('Starting cooperative following.')
        return True

    def tag_detection(self, step, scope):
        while not rospy.is_shutdown() and self.dog_x < scope:
            while not rospy.is_shutdown() and self.dog_y < scope:
                self.dog_y += step
                self.sim_pose(self.dog_x, self.dog_y, 0, 0, 0, 1)
                self.tag_detection_trigger()
                time.sleep(1)
            self.dog_x += step
            time.sleep(0.5)
            self.sim_pose(self.dog_x, self.dog_y, 0, 0, 0, 1)
            self.tag_detection_trigger()
            while not rospy.is_shutdown() and self.dog_y > 0:
                self.dog_y -= step
                self.sim_pose(self.dog_x, self.dog_y, 0, 0, 0, 1)
                self.tag_detection_trigger()
                time.sleep(1)
            self.dog_x += step
            self.sim_pose(self.dog_x, self.dog_y, 0, 0, 0, 1)
            self.tag_detection_trigger()
        while not rospy.is_shutdown() and self.dog_y < scope:
            self.dog_y += step
            self.sim_pose(self.dog_x, self.dog_y, 0, 0, 0, 1)
            self.tag_detection_trigger()
            time.sleep(1)
        self.dog_x = 0
        self.dog_y = 0
        self.sim_pose(self.dog_x, self.dog_y, 0, 0, 0, 1)
        self.tag_detection_trigger()

    def tag_detection_trigger(self):
        if self.find_valve_tag == 1:
            rospy.loginfo('Target AprilTag found.')
            return

    def tag_detection_gank(self):
        if self.find_valve_tag == 1:
            rospy.sleep(2)
            rospy.logdebug(
                'Valve-tag offset: x=%.3f y=%.3f', self.april_valve_x, self.april_valve_y
            )

            for _ in range(self.tag_alignment_attempts):
                if rospy.is_shutdown() or (
                    abs(self.april_valve_x) <= 0.1 and abs(self.april_valve_y) <= 0.1
                ):
                    break
                self.sim_pose(
                    self.dog_x + self.april_valve_x + self.camera2base_x,
                    self.dog_y + self.april_valve_y + self.camera2base_y,
                    0,
                    0,
                    0,
                    1,
                )
                self.dog_x = self.dog_x + self.april_valve_x + self.camera2base_x
                self.dog_y = self.dog_y + self.april_valve_y + self.camera2base_y
                rospy.sleep(0.1)
            else:
                rospy.logerr('Simulated tag alignment exceeded its attempt limit.')
                return False
            target_x = self.dog_x + self.april_valve_x + self.camera2base_x + self.valve2tag_x
            target_y = self.dog_y + self.april_valve_y + self.camera2base_y + self.valve2tag_y
            rospy.loginfo(
                'Publishing simulated drone target: x=%.3f y=%.3f z=%.3f',
                target_x,
                target_y,
                self.april_valve_z,
            )
            self.drone_nav_info(target_x, target_y, self.april_valve_z)
            return True
        rospy.logerr('Cannot align: the valve AprilTag is not visible.')
        return False

    def sim(self):
        if not self.takeoff():
            return False
        deadline = time.monotonic() + self.flight_state_timeout_s
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.state == 5:
                break
            rospy.sleep(0.1)
        if self.state != 5:
            rospy.logerr('Timed out waiting for flight state 5 after takeoff.')
            return False
        self.sim_pose(self.dog_x, self.dog_y, 0, 0, 0, 1)
        return self.tag_detection_gank()


def main():
    """Run the ROS node."""
    rospy.init_node('cooperation_simulation')
    if not rospy.get_param('~run_demo', False):
        rospy.logerr('Legacy simulation demo is disabled; set ~run_demo:=true to enable it.')
        raise SystemExit(2)
    if not rospy.get_param('~allow_takeoff', False):
        rospy.logerr('Takeoff is disabled; set ~allow_takeoff:=true after safety checks.')
        raise SystemExit(2)
    node = CooperationNode()
    rospy.sleep(1.0)
    if not node.sim():
        raise SystemExit(1)
    rospy.spin()

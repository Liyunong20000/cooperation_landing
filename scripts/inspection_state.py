#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import numpy as np
import time, math, threading
import smach
import smach_ros
import tf.transformations as tft
from appdirs import user_data_dir
from lxml.html import Classes

from std_msgs.msg import Empty, UInt8
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from tf.tfwtf import rostime_delta

from aerial_robot_msgs.msg import FlightNav
from apriltag_ros.msg import AprilTagDetectionArray


# The Cooperative inspection system by mini_quadrotor and Qilin
class Start(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['rm'])
        rospy.wait_for_service('/go1/sit')
        rospy.wait_for_service('/go1/stand')
        self.service_client_sit = rospy.ServiceProxy('/go1/sit', Trigger)
        self.service_client_stand = rospy.ServiceProxy('/go1/stand', Trigger)

        self.rm = 0

    def sit(self):
        try:
            response = self.service_client_sit()
            if response.success:
                rospy.loginfo('Sit command executed successfully')
            else:
                rospy.logwarn('Stand command failed: %s', response.message)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)

    def stand(self):
        try:
            response = self.service_client_stand()
            if response.success:
                rospy.loginfo('Stand command executed successfully')
            else:
                rospy.logwarn('Stand command failed: %s', response.message)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)

    def execute(self, userdata):
        self.rm = userdata.rm
        if self.rm == 1:
            self.stand()
        print(f'pass begin.')
        return 'succeeded'

class MarkerSearch(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['rm'], io_keys=['takeoff_position'])

        rospy.Subscriber('/gazebo/model_states', ModelStates, self._callback_dog_position)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)
        rospy.Subscriber('/quadrotor/uav/cog/odom', Odometry, self._callback_position)

        self.pub_qilin_vel = rospy.Publisher('/go1/cmd_vel', Twist, queue_size=10)
        self.pub_sim_pose = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

        self.rm = 0

        self.dog_x, self.dog_y, self.dog_z, self.dog_yaw = 0.0, 0.0, 0.0, 0.0
        self.drone_x, self.drone_y, self.drone_z, self.drone_roll, self.drone_pitch, self.drone_yaw = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        self.takeoff_x, self.takeoff_y, self.takeoff_z, self.takeoff_yaw = 0.0, 0.0, 0.0, 0.0
        self.find_valve_tag, self.find_drone_tag = 0, 0
        self.land_offset = 0.4
        self.position = 0
    def _callback_apriltag(self, data):
        current_time = rospy.Time.now()
        # print(f'apriltag:{current_time.to_sec()}')

        # get the apriltag`s position information compare with camera coordination
        if data.detections:
            self.find_valve_tag = self.find_target_tag(data.detections,1)
            self.find_drone_tag = self.find_target_tag(data.detections, 0)

        else:
            self.find_valve_tag = 0
            self.find_drone_tag = 0

    def _callback_position(self, odom_msg):
        self.drone_x = odom_msg.pose.pose.position.x
        self.drone_y = odom_msg.pose.pose.position.y
        self.drone_z = odom_msg.pose.pose.position.z
        self.drone_ori_x = odom_msg.pose.pose.orientation.x
        self.drone_ori_y = odom_msg.pose.pose.orientation.y
        self.drone_ori_z = odom_msg.pose.pose.orientation.z
        self.drone_ori_w = odom_msg.pose.pose.orientation.w



    def _callback_dog_position(self, data):
        self.dog_x= data.pose[2].position.x
        self.dog_y= data.pose[2].position.y
        self.dog_z= data.pose[2].position.z

    def find_target_tag(self,data,target_id):
        b = len(data)
        a = 0
        while a < b:
            c = target_id in data[a].id
            if c:
                return 1
            else:
                return 0

    def qilin_cmd_vel(self, lx, ly, ax, ay, az):
        qilin_cmd_vel = Twist()
        qilin_cmd_vel.linear.x = lx
        qilin_cmd_vel.linear.y = ly
        qilin_cmd_vel.angular.x = ax
        qilin_cmd_vel.angular.y = ay
        qilin_cmd_vel.angular.z = az

        self.pub_qilin_vel.publish(qilin_cmd_vel)

    def sim_pose(self, px, py, ox, oy, oz, ow):

        sim_pose = ModelState()
        sim_pose.model_name = 'unitree'
        sim_pose.pose.position.x = px
        sim_pose.pose.position.y = py
        sim_pose.pose.orientation.x = ox
        sim_pose.pose.orientation.y = oy
        sim_pose.pose.orientation.z = oz
        sim_pose.pose.orientation.w = ow
        sim_pose.reference_frame = 'world'
        self.pub_sim_pose.publish(sim_pose)

    def record_takeoff_position(self):
        self.takeoff_x = self.drone_x
        self.takeoff_y = self.drone_y
        self.takeoff_z = self.drone_z
        self.takeoff_yaw = self.drone_yaw
        print(f'takeoff position:{self.takeoff_x}, {self.takeoff_y},{self.takeoff_z}, {self.takeoff_yaw}')

    def execute(self, userdata):
        self.rm = userdata.rm
        if self.rm == 1:
            while not self.find_valve_tag:
                self.qilin_cmd_vel(0.2, 0, 0, 0, 0)
            time.sleep(0.5)
            self.qilin_cmd_vel(0, 0, 0, 0, 0)
            time.sleep(2)
            self.record_takeoff_position()
            userdata.takeoff_position = np.array([self.takeoff_x, self.takeoff_y, self.takeoff_z, self.takeoff_yaw])
            return 'succeeded'

        else:
            while not self.find_valve_tag:
                self.position += 0.5
                self.sim_pose(self.position, 0, 0, 0, 0, 1)
                time.sleep(1)
            userdata.takeoff_position = np.array([self.position, self.dog_y, self.land_offset, 0.0])
            print(f'This is takeoff_position:{userdata.takeoff_position}')
            return 'succeeded'

class TargetCalculation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],input_keys=['takeoff_position'],io_keys=['target_position'])

        rospy.Subscriber('/quadrotor/uav/cog/odom', Odometry, self._callback_position)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)

        self.pub_land = rospy.Publisher('/quadrotor/teleop_command/land', Empty, queue_size=10)

        self.valve2tag_x, self.valve2tag_y, self.valve2tag_z= 0.0, 2.0, 1.5
        self.gear_offset = 0.5
        self.valve_x, self.valve_y, self.valve_z = 0.0, 0.0, 0.0
        self.april_valve_x, self.april_valve_y, self.april_valve_z, self.april_valve_yaw = 0.0, 0.0, 0.0, 0.0
        self.takeoff_x, self.takeoff_y, self.takeoff_z, self.takeoff_yaw = 0.0, 0.0, 0.0, 0.0
        self.drone_x, self.drone_y, self.drone_z, self.drone_roll, self.drone_pitch, self.drone_yaw = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        self.drone_ori_x, self.drone_ori_y, self.drone_ori_z, self.drone_ori_w = 0.0, 0.0, 0.0, 0.0

    def _callback_position(self, odom_msg):
        self.drone_x = odom_msg.pose.pose.position.x
        self.drone_y = odom_msg.pose.pose.position.y
        self.drone_z = odom_msg.pose.pose.position.z
        self.drone_ori_x = odom_msg.pose.pose.orientation.x
        self.drone_ori_y = odom_msg.pose.pose.orientation.y
        self.drone_ori_z = odom_msg.pose.pose.orientation.z
        self.drone_ori_w = odom_msg.pose.pose.orientation.w
        self.drone_roll = tft.euler_from_quaternion([self.drone_ori_x, self.drone_ori_y, self.drone_ori_z, self.drone_ori_w])[0]
        self.drone_pitch = tft.euler_from_quaternion([self.drone_ori_x, self.drone_ori_y, self.drone_ori_z, self.drone_ori_w])[1]
        self.drone_yaw = tft.euler_from_quaternion([self.drone_ori_x, self.drone_ori_y, self.drone_ori_z, self.drone_ori_w])[2]

        # print(f'drone_yaw:{self.drone_roll}, {self.drone_pitch},{self.drone_yaw}')
        if self.drone_z <-0.5 or self.drone_z > 3 or abs(self.drone_roll) > 45 or abs(self.drone_pitch) > 45:
            rospy.loginfo("Wrong state! land!")
            self.land()
            exit()

    def _callback_apriltag(self, data):
        current_time = rospy.Time.now()
        # print(f'apriltag:{current_time.to_sec()}')

        # get the apriltag`s position information compare with camera coordination
        if data.detections:
            self.find_valve_tag = self.find_target_tag(data.detections,1)
        else:
            self.find_valve_tag = 0


    def find_target_tag(self,data,target_id):
        b = len(data)
        a = 0

        while a < b:
            c = target_id in data[a].id
            # print(f'{a}')
            if c:
                if target_id == 1:
                    self.april_valve_x = -data[a].pose.pose.pose.position.y
                    self.april_valve_y = data[a].pose.pose.pose.position.x
                    self.april_valve_z = data[a].pose.pose.pose.position.z
                    self.april_valve_yaw = tft.euler_from_quaternion([data[a].pose.pose.pose.orientation.x,
                                                         data[a].pose.pose.pose.orientation.y,
                                                         data[a].pose.pose.pose.orientation.z,
                                                         data[a].pose.pose.pose.orientation.w])[2]
                return 1
            else:
                return 0

    def land(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing land command...")
        empty_msg = Empty()
        self.pub_land.publish(empty_msg)


    def execute(self, userdata):
        # self.record_takeoff_position()
        self.takeoff_x = userdata.takeoff_position[0]
        self.takeoff_y = userdata.takeoff_position[1]
        self.takeoff_z = userdata.takeoff_position[2]
        self.takeoff_yaw = userdata.takeoff_position[3]
        print(f'{self.april_valve_x,self.april_valve_y,self.april_valve_z}')
        self.valve_x= self.takeoff_x - self.april_valve_x - self.valve2tag_x
        self.valve_y= self.takeoff_y - self.april_valve_y - self.valve2tag_y
        self.valve_z= self.april_valve_z + self.gear_offset - self.valve2tag_z
        # userdata.takeoff_position = np.array([self.takeoff_x,self.takeoff_y,self.takeoff_z,self.takeoff_yaw])
        userdata.target_position = np.array([self.valve_x,self.valve_y,self.valve_z])

        print(f'This is the target_position:{userdata.target_position}')
        print(type(userdata.target_position))
        time.sleep(1)
        return 'succeeded'

class UavTakeoff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        rospy.Subscriber('/quadrotor/flight_state', UInt8, self._callback_state)

        self.pub_takeoff = rospy.Publisher('/quadrotor/teleop_command/takeoff', Empty, queue_size=10)
        self.pub_arm = rospy.Publisher("/quadrotor/teleop_command/start", Empty, queue_size=1)
        self.pub_land = rospy.Publisher('/quadrotor/teleop_command/land', Empty, queue_size=10)

        self.drone_x, self.drone_y, self.drone_z, self.drone_roll, self.drone_pitch, self.drone_yaw = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        self.drone_ori_x, self.drone_ori_y, self.drone_ori_z, self.drone_ori_w = 0.0, 0.0, 0.0, 0.0
        self.state= 0
        self.user_input = None

    def takeoff(self):
        time.sleep(0.5)
        if (-10 < self.drone_x <10) and (-10 < self.drone_y <10) and (-0.5 < self.drone_z <3):
            rospy.loginfo("Publishing takeoff command...")
            empty_msg = Empty()
            self.pub_arm.publish(empty_msg)
            time.sleep(0.1)
            self.pub_takeoff.publish(empty_msg)
        else:
            rospy.loginfo("Don`t takeoff, state is wrong!!!")
    def _callback_state(self, msg):
        self.state = msg.data



        # print(f'drone_yaw:{self.drone_roll}, {self.drone_pitch},{self.drone_yaw}')
        # if self.drone_z <-0.5 or self.drone_z > 3 or abs(self.drone_roll) > 45 or abs(self.drone_pitch) > 45:
        #     rospy.loginfo("Wrong state! land!")
        #     self.land()
        #     exit()
    def land(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing land command...")
        empty_msg = Empty()
        self.pub_land.publish(empty_msg)

    def execute(self, userdata):
        print(f'takeoff?')
        self.user_input = input()
        if self.user_input == 'y':
            print(f'yes')
            self.takeoff()
            while not rospy.is_shutdown():
                if self.state == 5:
                    break
                time.sleep(0.1)
            return 'succeeded'
        else:
            return 'failed'

class FlyTarget(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['target_position', 'rm'])
        self.pub_drone_target_info = rospy.Publisher('/quadrotor/target_pose/info', PoseStamped, queue_size=10)
        self.pub_drone_target_trigger = rospy.Publisher('/quadrotor/target_pose/trigger', Empty, queue_size=10)
        self.pub_drone_target_sim = rospy.Publisher('/quadrotor/target_pose', PoseStamped, queue_size=10)

        # self.pub_drone_nav_info = rospy.Publisher('/quadrotor/uav/nav/info', FlightNav, queue_size=10)
        # self.pub_drone_nav_trigger = rospy.Publisher('/quadrotor/uav/nav/trigger', Empty, queue_size=10)


        self.rm = 0
        self.valve_x, self.valve_y, self.valve_z = 0.0, 0.0, 0.0
        self.user_input = None
    # def drone_nav_trigger(self):
    #     time.sleep(0.5)
    #     rospy.loginfo("Publishing nav trigger command...")
    #     empty_msg = Empty()
    #     self.pub_drone_nav_trigger.publish(empty_msg)
    #
    # def drone_nav_info(self, x_y_mode, x, y, z_mode, z, yaw_mode, omega_z, yaw):
    #     flight_nav_msg = FlightNav()
    #     flight_nav_msg.header.seq = self._seq
    #     self._seq += 1
    #     flight_nav_msg.header.stamp = rospy.Time.now()
    #     flight_nav_msg.header.frame_id = 'world'
    #
    #     flight_nav_msg.control_frame = 0
    #     flight_nav_msg.target = 0
    #     flight_nav_msg.pos_xy_nav_mode = x_y_mode
    #     flight_nav_msg.target_pos_x = x
    #     flight_nav_msg.target_vel_x = 0.0
    #     flight_nav_msg.target_acc_x = 0.0
    #     flight_nav_msg.target_pos_y = y
    #     flight_nav_msg.target_vel_y = 0.0
    #     flight_nav_msg.target_acc_y = 0.0
    #     flight_nav_msg.yaw_nav_mode = yaw_mode
    #     flight_nav_msg.target_omega_z = omega_z
    #     flight_nav_msg.target_yaw = yaw
    #     flight_nav_msg.pos_z_nav_mode = z_mode
    #     flight_nav_msg.target_pos_z = z
    #     flight_nav_msg.target_vel_z = 0.0
    #     flight_nav_msg.target_pos_diff_z = 0.0
    #
    #     self.pub_drone_nav_info.publish(flight_nav_msg)
    #     rospy.sleep(0.5)
    #     self.drone_nav_trigger()
    #
    # def drone_nav_info_sim(self, x, y, z):
    #     flight_nav_msg = FlightNav()
    #     flight_nav_msg.header.seq = self._seq
    #     self._seq += 1
    #     flight_nav_msg.header.stamp = rospy.Time.now()
    #     flight_nav_msg.header.frame_id = 'world'
    #     flight_nav_msg.pos_xy_nav_mode = 2
    #     flight_nav_msg.target_pos_x = x
    #     flight_nav_msg.target_pos_y = y
    #     flight_nav_msg.pos_z_nav_mode = 2
    #     flight_nav_msg.target_pos_z = z
    #     self.pub_drone_nav.publish(flight_nav_msg)

    def drone_target_pose(self,x,y,z,ox,oy,oz,ow):
        drone_target_pose = PoseStamped()
        drone_target_pose.header.frame_id = 'world'
        drone_target_pose.pose.position.x = x
        drone_target_pose.pose.position.y = y
        drone_target_pose.pose.position.z = z
        drone_target_pose.pose.orientation.x = ox
        drone_target_pose.pose.orientation.y = oy
        drone_target_pose.pose.orientation.z = oz
        drone_target_pose.pose.orientation.w = ow

        self.pub_drone_target_info.publish(drone_target_pose)
        rospy.sleep(1.5)
        self.drone_target_pose_trigger()

    def drone_target_pose_trigger(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing nav trigger command...")
        empty_msg = Empty()
        self.pub_drone_target_trigger.publish(empty_msg)

    def drone_target_pose_sim(self,x,y,z,ox,oy,oz,ow):
        drone_target_pose = PoseStamped()
        drone_target_pose.header.frame_id = 'world'
        drone_target_pose.pose.position.x = x
        drone_target_pose.pose.position.y = y
        drone_target_pose.pose.position.z = z
        drone_target_pose.pose.orientation.x = ox
        drone_target_pose.pose.orientation.y = oy
        drone_target_pose.pose.orientation.z = oz
        drone_target_pose.pose.orientation.w = ow

        self.pub_drone_target_sim.publish(drone_target_pose)

    def execute(self, userdata):
        print(f'{userdata.target_position[0]}, {userdata.target_position[1]}, {userdata.target_position[2]}')
        # print(f'{userdata.target_position}')
        self.rm= userdata.rm
        self.valve_x, self.valve_y, self.valve_z = userdata.target_position[0], userdata.target_position[1], userdata.target_position[2]
        print(f'simulation go to 1!')
        if self.rm == 1:
            self.user_input = input()
            if self.user_input == 'y':
                print(f'yes')
                self.drone_target_pose(self.valve_x, self.valve_y,self.valve_z,0,0,0,1)
        else:
            self.drone_target_pose_sim(self.valve_x, self.valve_y,self.valve_z,0,0,0,1)

        time.sleep(5)

        return 'succeeded'
class Inspection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['target_position', 'rm'])
        rospy.Subscriber('/quadrotor/uav/cog/odom', Odometry, self._callback_position)
        # self.pub_drone_nav = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)
        #
        # self.pub_drone_nav_info = rospy.Publisher('/quadrotor/uav/nav/info', FlightNav, queue_size=10)
        # self.pub_drone_nav_trigger = rospy.Publisher('/quadrotor/uav/nav/trigger', Empty, queue_size=10)

        self.pub_drone_target_info = rospy.Publisher('/quadrotor/target_pose/info', PoseStamped, queue_size=10)
        self.pub_drone_target_trigger = rospy.Publisher('/quadrotor/target_pose/trigger', Empty, queue_size=10)
        self.pub_drone_target_sim = rospy.Publisher('/quadrotor/target_pose', PoseStamped, queue_size=10)

        self.drone_x, self.drone_y, self.drone_z, self.drone_roll, self.drone_pitch, self.drone_yaw = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        self.drone_ori_x, self.drone_ori_y, self.drone_ori_z, self.drone_ori_w = 0.0, 0.0, 0.0, 0.0
        self.valve_x, self.valve_y, self.valve_z = 0.0, 0.0, 0.0
        self.tolerance_yaw = 0.1
        self.rm= 0
    # def drone_nav_trigger(self):
    #     time.sleep(0.5)
    #     rospy.loginfo("Publishing nav trigger command...")
    #     empty_msg = Empty()
    #     self.pub_drone_nav_trigger.publish(empty_msg)
    #
    # def drone_nav_info(self, x_y_mode, x, y, z_mode, z, yaw_mode, omega_z, yaw):
    #     flight_nav_msg = FlightNav()
    #     flight_nav_msg.header.seq = self._seq
    #     self._seq += 1
    #     flight_nav_msg.header.stamp = rospy.Time.now()
    #     flight_nav_msg.header.frame_id = 'world'
    #
    #     flight_nav_msg.control_frame = 0
    #     flight_nav_msg.target = 0
    #     flight_nav_msg.pos_xy_nav_mode = x_y_mode
    #     flight_nav_msg.target_pos_x = x
    #     flight_nav_msg.target_vel_x = 0.0
    #     flight_nav_msg.target_acc_x = 0.0
    #     flight_nav_msg.target_pos_y = y
    #     flight_nav_msg.target_vel_y = 0.0
    #     flight_nav_msg.target_acc_y = 0.0
    #     flight_nav_msg.yaw_nav_mode = yaw_mode
    #     flight_nav_msg.target_omega_z = omega_z
    #     flight_nav_msg.target_yaw = yaw
    #     flight_nav_msg.pos_z_nav_mode = z_mode
    #     flight_nav_msg.target_pos_z = z
    #     flight_nav_msg.target_vel_z = 0.0
    #     flight_nav_msg.target_pos_diff_z = 0.0
    #
    #     self.pub_drone_nav_info.publish(flight_nav_msg)
    #     rospy.sleep(0.5)
    #     self.drone_nav_trigger()
    #
    # def drone_nav_info_sim(self, x_y_mode, x, y, z_mode, z, yaw_mode, omega_z, yaw):
    #     flight_nav_msg = FlightNav()
    #     flight_nav_msg.header.seq = self._seq
    #     self._seq += 1
    #     flight_nav_msg.header.stamp = rospy.Time.now()
    #     flight_nav_msg.header.frame_id = 'world'
    #
    #     flight_nav_msg.control_frame = 0
    #     flight_nav_msg.target = 0
    #     flight_nav_msg.pos_xy_nav_mode = x_y_mode
    #     flight_nav_msg.target_pos_x = x
    #     flight_nav_msg.target_vel_x = 0.0
    #     flight_nav_msg.target_acc_x = 0.0
    #     flight_nav_msg.target_pos_y = y
    #     flight_nav_msg.target_vel_y = 0.0
    #     flight_nav_msg.target_acc_y = 0.0
    #     flight_nav_msg.yaw_nav_mode = yaw_mode
    #     flight_nav_msg.target_omega_z = omega_z
    #     flight_nav_msg.target_yaw = yaw
    #     flight_nav_msg.pos_z_nav_mode = z_mode
    #     flight_nav_msg.target_pos_z = z
    #     flight_nav_msg.target_vel_z = 0.0
    #     flight_nav_msg.target_pos_diff_z = 0.0
    #
    #     self.pub_drone_nav.publish(flight_nav_msg)


    def _callback_position(self, odom_msg):
        self.drone_x = odom_msg.pose.pose.position.x
        self.drone_y = odom_msg.pose.pose.position.y
        self.drone_z = odom_msg.pose.pose.position.z
        self.drone_ori_x = odom_msg.pose.pose.orientation.x
        self.drone_ori_y = odom_msg.pose.pose.orientation.y
        self.drone_ori_z = odom_msg.pose.pose.orientation.z
        self.drone_ori_w = odom_msg.pose.pose.orientation.w
        self.drone_roll = tft.euler_from_quaternion([self.drone_ori_x, self.drone_ori_y, self.drone_ori_z, self.drone_ori_w])[0]
        self.drone_pitch = tft.euler_from_quaternion([self.drone_ori_x, self.drone_ori_y, self.drone_ori_z, self.drone_ori_w])[1]
        self.drone_yaw = tft.euler_from_quaternion([self.drone_ori_x, self.drone_ori_y, self.drone_ori_z, self.drone_ori_w])[2]

    def drone_target_pose(self, x, y, z, ox, oy, oz, ow):
        drone_target_pose = PoseStamped()
        drone_target_pose.header.frame_id = 'world'
        drone_target_pose.pose.position.x = x
        drone_target_pose.pose.position.y = y
        drone_target_pose.pose.position.z = z
        drone_target_pose.pose.orientation.x = ox
        drone_target_pose.pose.orientation.y = oy
        drone_target_pose.pose.orientation.z = oz
        drone_target_pose.pose.orientation.w = ow

        self.pub_drone_target_info.publish(drone_target_pose)
        rospy.sleep(1.5)
        self.drone_target_pose_trigger()

    def drone_target_pose_trigger(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing nav trigger command...")
        empty_msg = Empty()
        self.pub_drone_target_trigger.publish(empty_msg)

    def drone_target_pose_sim(self, x, y, z, ox, oy, oz, ow):
        drone_target_pose = PoseStamped()
        drone_target_pose.header.frame_id = 'world'
        drone_target_pose.pose.position.x = x
        drone_target_pose.pose.position.y = y
        drone_target_pose.pose.position.z = z
        drone_target_pose.pose.orientation.x = ox
        drone_target_pose.pose.orientation.y = oy
        drone_target_pose.pose.orientation.z = oz
        drone_target_pose.pose.orientation.w = ow

        self.pub_drone_target_sim.publish(drone_target_pose)

    def execute(self, userdata):

        self.rm= userdata.rm
        self.valve_x, self.valve_y, self.valve_z = userdata.target_position[0], userdata.target_position[1], \
        userdata.target_position[2]
        if self.rm == 1:
            # self.drone_target_pose(self.valve_x, self.valve_y, self.valve_z, 0, 0, 0.707, 0.707)
            # while not rospy.is_shutdown():
            #     if abs(self.drone_yaw-1.57)< self.tolerance_yaw:
            #         break
            #     time.sleep(0.5)
            time.sleep(5)
            self.drone_target_pose(self.valve_x, self.valve_y, self.valve_z, 0, 0, 0.707107, 0.707107)
            time.sleep(5)
            self.drone_target_pose(self.valve_x, self.valve_y, self.valve_z, 0, 0, 1, 0)
            time.sleep(8)
            self.drone_target_pose(self.valve_x, self.valve_y, self.valve_z, 0, 0, -0.707107, 0.707107)
            time.sleep(5)
            self.drone_target_pose(self.valve_x, self.valve_y, self.valve_z, 0, 0, 0, 1)
            time.sleep(5)
            self.drone_target_pose(self.valve_x, self.valve_y, self.valve_z, 0, 0, 0.707107, 0.707107)
            time.sleep(5)
            self.drone_target_pose(self.valve_x, self.valve_y, self.valve_z, 0, 0, 1, 0)
            time.sleep(5)
            # self.drone_target_pose(self.valve_x, self.valve_y, self.valve_z, 0, 0, -0.707, 0.707)
            # time.sleep(10)
            # self.drone_target_pose(self.valve_x, self.valve_y, self.valve_z, 0, 0, 0, 1)
            # time.sleep(10)

        else:
            # self.drone_target_pose_sim(self.valve_x, self.valve_y, self.valve_z, 0, 0, 0, 1)
            # while not rospy.is_shutdown():
            #     if abs(self.drone_x- self.valve_x) < 0.05 and abs(self.drone_y - self.valve_y) < 0.05:
            #         break
            time.sleep(5)

            self.drone_target_pose_sim(self.valve_x, self.valve_y, self.valve_z, 0, 0, 0.707, 0.707)
            # while not rospy.is_shutdown():
            #     if abs(self.drone_yaw - 1.57) < self.tolerance_yaw:
            #         break
            time.sleep(5)
            self.drone_target_pose_sim(self.valve_x, self.valve_y, self.valve_z, 0, 0, 1, 0)
            # while not rospy.is_shutdown():
            #     if abs(self.drone_yaw - 3.14) < self.tolerance_yaw:
            #         break
            time.sleep(5)
            self.drone_target_pose_sim(self.valve_x, self.valve_y, self.valve_z, 0, 0, -0.707, 0.707)
            # while not rospy.is_shutdown():
            #     if abs(self.drone_yaw + 1.57) < self.tolerance_yaw:
            #         break
            time.sleep(5)
            self.drone_target_pose_sim(self.valve_x, self.valve_y, self.valve_z, 0, 0, 0, 1)
            # while not rospy.is_shutdown():
            #     if abs(self.drone_yaw) < self.tolerance_yaw:
            #         break
            time.sleep(5)
            self.drone_target_pose_sim(self.valve_x, self.valve_y, self.valve_z, 0, 0, 0.707, 0.707)
            # while not rospy.is_shutdown():
            #     if abs(self.drone_yaw - 1.57) < self.tolerance_yaw:
            #         break
            time.sleep(5)
            self.drone_target_pose_sim(self.valve_x, self.valve_y, self.valve_z, 0, 0, 1, 0)
            # while not rospy.is_shutdown():
            #     if abs(self.drone_yaw - 3.14) < self.tolerance_yaw:
            #         break
            time.sleep(5)
            self.drone_target_pose_sim(self.valve_x, self.valve_y, self.valve_z, 0, 0, -0.707, 0.707)
            # while not rospy.is_shutdown():
            #     if abs(self.drone_yaw + 1.57) < self.tolerance_yaw:
            #         break
            time.sleep(5)
            self.drone_target_pose_sim(self.valve_x, self.valve_y, self.valve_z, 0, 0, 0, 1)
            # while not rospy.is_shutdown():
            #     if abs(self.drone_yaw) < self.tolerance_yaw:
            #         break
            time.sleep(5)
        return 'succeeded'
class FlyBack(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],input_keys=['takeoff_position', 'rm'])
        # self.pub_drone_nav_info = rospy.Publisher('/quadrotor/uav/nav/info', FlightNav, queue_size=10)
        # self.pub_drone_nav_trigger = rospy.Publisher('/quadrotor/uav/nav/trigger', Empty, queue_size=10)
        # self.pub_drone_nav = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)

        self.pub_drone_target_info = rospy.Publisher('/quadrotor/target_pose/info', PoseStamped, queue_size=10)
        self.pub_drone_target_trigger = rospy.Publisher('/quadrotor/target_pose/trigger', Empty, queue_size=10)
        self.pub_drone_target_sim = rospy.Publisher('/quadrotor/target_pose', PoseStamped, queue_size=10)

        self.takeoff_x, self.takeoff_y, self.takeoff_z, self.takeoff_yaw = 0.0, 0.0, 0.0, 0.0
        self.rm= 0
        self.land_offset = 0.4
    # def drone_nav_trigger(self):
    #     time.sleep(0.5)
    #     rospy.loginfo("Publishing nav trigger command...")
    #     empty_msg = Empty()
    #     self.pub_drone_nav_trigger.publish(empty_msg)
    #
    # def drone_nav_info(self, x_y_mode, x, y, z_mode, z, yaw_mode, omega_z, yaw):
    #     flight_nav_msg = FlightNav()
    #     flight_nav_msg.header.stamp = rospy.Time.now()
    #     flight_nav_msg.header.frame_id = 'world'
    #
    #     flight_nav_msg.control_frame = 0
    #     flight_nav_msg.target = 0
    #     flight_nav_msg.pos_xy_nav_mode = x_y_mode
    #     flight_nav_msg.target_pos_x = x
    #     flight_nav_msg.target_vel_x = 0.0
    #     flight_nav_msg.target_acc_x = 0.0
    #     flight_nav_msg.target_pos_y = y
    #     flight_nav_msg.target_vel_y = 0.0
    #     flight_nav_msg.target_acc_y = 0.0
    #     flight_nav_msg.yaw_nav_mode = yaw_mode
    #     flight_nav_msg.target_omega_z = omega_z
    #     flight_nav_msg.target_yaw = yaw
    #     flight_nav_msg.pos_z_nav_mode = z_mode
    #     flight_nav_msg.target_pos_z = z
    #     flight_nav_msg.target_vel_z = 0.0
    #     flight_nav_msg.target_pos_diff_z = 0.0
    #
    #     self.pub_drone_nav.publish(flight_nav_msg)

        # self.pub_drone_nav_info.publish(flight_nav_msg)
        # rospy.sleep(0.5)
        # self.drone_nav_trigger()

    def drone_target_pose(self, x, y, z, ox, oy, oz, ow):
        drone_target_pose = PoseStamped()
        drone_target_pose.header.frame_id = 'world'
        drone_target_pose.pose.position.x = x
        drone_target_pose.pose.position.y = y
        drone_target_pose.pose.position.z = z
        drone_target_pose.pose.orientation.x = ox
        drone_target_pose.pose.orientation.y = oy
        drone_target_pose.pose.orientation.z = oz
        drone_target_pose.pose.orientation.w = ow

        self.pub_drone_target_info.publish(drone_target_pose)
        rospy.sleep(1.5)
        self.drone_target_pose_trigger()

    def drone_target_pose_trigger(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing nav trigger command...")
        empty_msg = Empty()
        self.pub_drone_target_trigger.publish(empty_msg)

    def drone_target_pose_sim(self, x, y, z, ox, oy, oz, ow):
        drone_target_pose = PoseStamped()
        drone_target_pose.header.frame_id = 'world'
        drone_target_pose.pose.position.x = x
        drone_target_pose.pose.position.y = y
        drone_target_pose.pose.position.z = z
        drone_target_pose.pose.orientation.x = ox
        drone_target_pose.pose.orientation.y = oy
        drone_target_pose.pose.orientation.z = oz
        drone_target_pose.pose.orientation.w = ow

        self.pub_drone_target_sim.publish(drone_target_pose)

    def execute(self, userdata):
        self.takeoff_x = userdata.takeoff_position[0]
        self.takeoff_y = userdata.takeoff_position[1]
        self.takeoff_z = userdata.takeoff_position[2]
        self.rm = userdata.rm
        print(f'This is the fly back position: {self.takeoff_x}, {self.takeoff_y}, {self.takeoff_z}')
        time.sleep(1)
        if self.rm ==1:
            self.drone_target_pose( self.takeoff_x, self.takeoff_y, self.takeoff_z + self.land_offset, 0, 0, 1,0)
            time.sleep(10)
            # self.drone_target_pose(self.takeoff_x, self.takeoff_y, self.takeoff_z + self.land_offset, 0, 0, 1, 0)
            # time.sleep(5)
        else:
            self.drone_target_pose_sim( self.takeoff_x, self.takeoff_y,  self.takeoff_z + self.land_offset, 0, 0, 0,1)
        return 'succeeded'
class VisibilityAdjustment(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['takeoff_position','rm'])

        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self._callback_dog_position)

        # self.pub_drone_nav = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)
        # self.pub_drone_nav_info = rospy.Publisher('/quadrotor/uav/nav/info', FlightNav, queue_size=10)
        # self.pub_drone_nav_trigger = rospy.Publisher('/quadrotor/uav/nav/trigger', Empty, queue_size=10)

        self.pub_drone_target_info = rospy.Publisher('/quadrotor/target_pose/info', PoseStamped, queue_size=10)
        self.pub_drone_target_trigger = rospy.Publisher('/quadrotor/target_pose/trigger', Empty, queue_size=10)
        self.pub_drone_target_sim = rospy.Publisher('/quadrotor/target_pose', PoseStamped, queue_size=10)

        self.pub_sim_pose = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        self.pub_qilin_vel = rospy.Publisher('/go1/cmd_vel', Twist, queue_size=10)

        self.takeoff_x, self.takeoff_y, self.takeoff_z, self.takeoff_yaw = 0.0, 0.0, 0.0, 0.0
        self.april_drone_x, self.april_drone_y, self.april_drone_z, self.april_drone_yaw = 0.0, 0.0, 0.0, 0.0
        self.april_valve_x, self.april_valve_y, self.april_valve_z, self.april_valve_yaw = 0.0, 0.0, 0.0, 0.0
        self.dog_x, self.dog_y, self.dog_z, self.dog_yaw = 0.0, 0.0, 0.0, 0.0
        self.lx, self.ly = 0.0, 0.0
        self.land_offset = 0.4

        self.beginfollow = 0
        self.find_drone_tag = 0
        self.timer_detector_maker = 0
        self.rm = 0
        self.miss_first_time, self.miss_second_time = rospy.Time.now(), rospy.Time.now()


    def _callback_apriltag(self, data):
        current_time = rospy.Time.now()
        # print(f'apriltag:{current_time.to_sec()}')

        # get the apriltag`s position information compare with camera coordination
        if data.detections:
            self.find_valve_tag = self.find_target_tag(data.detections, 1)
            self.find_drone_tag = self.find_target_tag(data.detections, 0)
            if self.find_drone_tag == 1:
                if self.beginfollow == 1:
                    if self.rm == 1:
                        self.align_dog_with_drone()
                    else:
                        self.align_dog_with_drone_sim()


        else:
            self.find_valve_tag = 0
            self.find_drone_tag = 0
            if self.beginfollow == 1:
                self.miss_second_time = rospy.Time.now()
                duration = self.miss_second_time - self.miss_first_time
                # print(f'{duration.to_sec()}')

                if duration.to_sec() > 0.5:
                    self.qilin_cmd_vel(0, 0, 0, 0, 0)
                    self.miss_first_time = self.miss_second_time

    def _callback_dog_position(self, data):
        self.dog_x= data.pose[2].position.x
        self.dog_y= data.pose[2].position.y
        self.dog_z= data.pose[2].position.z

    def find_target_tag(self,data,target_id):
        b = len(data)
        a = 0

        while a < b:
            c = target_id in data[a].id
            # print(f'{a}')
            if c:
                if target_id == 0:
                    self.april_drone_x = -data[a].pose.pose.pose.position.y
                    self.april_drone_y = data[a].pose.pose.pose.position.x
                    self.april_drone_z = data[a].pose.pose.pose.position.z
                    self.april_drone_yaw = tft.euler_from_quaternion([data[a].pose.pose.pose.orientation.x,
                                                                          data[a].pose.pose.pose.orientation.y,
                                                                          data[a].pose.pose.pose.orientation.z,
                                                                          data[a].pose.pose.pose.orientation.w])[2]

                if target_id == 1:
                    self.april_valve_x = -data[a].pose.pose.pose.position.y
                    self.april_valve_y = data[a].pose.pose.pose.position.x
                    self.april_valve_z = data[a].pose.pose.pose.position.z
                    self.april_valve_yaw = tft.euler_from_quaternion([data[a].pose.pose.pose.orientation.x,
                                                         data[a].pose.pose.pose.orientation.y,
                                                         data[a].pose.pose.pose.orientation.z,
                                                         data[a].pose.pose.pose.orientation.w])[2]

                return 1
            else:
                return 0


    def sim_pose(self, px, py, ox, oy, oz, ow):

        sim_pose = ModelState()
        sim_pose.model_name = 'unitree'
        sim_pose.pose.position.x = px
        sim_pose.pose.position.y = py
        sim_pose.pose.orientation.x = ox
        sim_pose.pose.orientation.y = oy
        sim_pose.pose.orientation.z = oz
        sim_pose.pose.orientation.w = ow
        sim_pose.reference_frame = 'world'
        self.pub_sim_pose.publish(sim_pose)

    def qilin_cmd_vel(self, lx, ly, ax, ay, az):
        qilin_cmd_vel = Twist()
        qilin_cmd_vel.linear.x = lx
        qilin_cmd_vel.linear.y = ly
        qilin_cmd_vel.angular.x = ax
        qilin_cmd_vel.angular.y = ay
        qilin_cmd_vel.angular.z = az

        self.pub_qilin_vel.publish(qilin_cmd_vel)

    def align_dog_with_drone(self):
        self.lx = 1.2 * self.april_drone_x
        self.ly = 1.2 * self.april_drone_y
        self.april_z = 0.03 *self.april_drone_yaw
        if self.april_z > 0.3:
            self.april_z = 0.3
        if self.april_z < -0.3:
            self.april_z = -0.3
        # print("align speed lx, ly, :", self.lx, self.ly, self.april_z)
        # print(f'apriltag_time:{apriltag_time.to_sec()}')
        if abs(self.lx) < 2 and abs(self.ly) < 2:
            navigation_time = rospy.Time.now()
            # print(f'navigation_time:{navigation_time.to_sec()}')
            self.qilin_cmd_vel(self.lx, self.ly, 0, 0, self.april_z)

    def align_dog_with_drone_sim(self):
        if self.april_drone_x >0.2 and self.april_drone_y >0.2:
            print(f'This is the dog:{self.dog_x},{self.dog_y},april_drone:{self.april_drone_x},{self.april_drone_y}')
            self.lx = self.april_drone_x + self.dog_x
            self.ly = self.april_drone_y + self.dog_y
            self.sim_pose(self.lx, self.ly, 0, 0, 0, 1)
            print(f"This is the align:{self.lx},{self.ly}")


    def timer_detector(self,time,number):
        begin = rospy.Time.now()
        begin = begin.to_sec()
        finish = rospy.Time.now()
        finish = finish.to_sec()
        print(f'{begin},{finish}')
        self.timer_detector_marker = number
        r = rospy.Rate(number)
        while finish - begin < time:
            finish = rospy.Time.now()
            finish = finish.to_sec()
            if self.find_drone_tag == 1:
                self.timer_detector_marker -= 1
                print(f'{self.timer_detector_marker}')
            r.sleep()
        if self.timer_detector_marker <= 0:
            return 1
        else:
            return 0

    def drone_target_pose(self, x, y, z, ox, oy, oz, ow):
        drone_target_pose = PoseStamped()
        drone_target_pose.header.frame_id = 'world'
        drone_target_pose.pose.position.x = x
        drone_target_pose.pose.position.y = y
        drone_target_pose.pose.position.z = z
        drone_target_pose.pose.orientation.x = ox
        drone_target_pose.pose.orientation.y = oy
        drone_target_pose.pose.orientation.z = oz
        drone_target_pose.pose.orientation.w = ow

        self.pub_drone_target_info.publish(drone_target_pose)
        rospy.sleep(1.5)
        self.drone_target_pose_trigger()

    def drone_target_pose_trigger(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing nav trigger command...")
        empty_msg = Empty()
        self.pub_drone_target_trigger.publish(empty_msg)

    def drone_target_pose_sim(self, x, y, z, ox, oy, oz, ow):
        drone_target_pose = PoseStamped()
        drone_target_pose.header.frame_id = 'world'
        drone_target_pose.pose.position.x = x
        drone_target_pose.pose.position.y = y
        drone_target_pose.pose.position.z = z
        drone_target_pose.pose.orientation.x = ox
        drone_target_pose.pose.orientation.y = oy
        drone_target_pose.pose.orientation.z = oz
        drone_target_pose.pose.orientation.w = ow

        self.pub_drone_target_sim.publish(drone_target_pose)


    # def drone_nav_trigger(self):
    #     time.sleep(0.5)
    #     rospy.loginfo("Publishing nav trigger command...")
    #     empty_msg = Empty()
    #     self.pub_drone_nav_trigger.publish(empty_msg)
    #
    # def drone_nav_info(self, x_y_mode, x, y, z_mode, z, yaw_mode, omega_z, yaw):
    #     flight_nav_msg = FlightNav()
    #     flight_nav_msg.header.stamp = rospy.Time.now()
    #     flight_nav_msg.header.frame_id = 'world'
    #
    #     flight_nav_msg.control_frame = 0
    #     flight_nav_msg.target = 0
    #     flight_nav_msg.pos_xy_nav_mode = x_y_mode
    #     flight_nav_msg.target_pos_x = x
    #     flight_nav_msg.target_vel_x = 0.0
    #     flight_nav_msg.target_acc_x = 0.0
    #     flight_nav_msg.target_pos_y = y
    #     flight_nav_msg.target_vel_y = 0.0
    #     flight_nav_msg.target_acc_y = 0.0
    #     flight_nav_msg.yaw_nav_mode = yaw_mode
    #     flight_nav_msg.target_omega_z = omega_z
    #     flight_nav_msg.target_yaw = yaw
    #     flight_nav_msg.pos_z_nav_mode = z_mode
    #     flight_nav_msg.target_pos_z = z
    #     flight_nav_msg.target_vel_z = 0.0
    #     flight_nav_msg.target_pos_diff_z = 0.0
    #
    #     # self.pub_drone_nav_info.publish(flight_nav_msg)
    #     # rospy.sleep(0.5)
    #     # self.drone_nav_trigger()
    #
    #     self.pub_drone_nav.publish(flight_nav_msg)


    def scanning(self,x,y,z,c):
        if self.rm == 1:
            self.drone_target_pose(x + c, y, z, 0, 0, 0, 0)
            print(f'1')
            time.sleep(3.5)
            if self.timer_detector(3, 3):
                print(f'find, return')
                return
            self.drone_target_pose(x, y + c, z, 0, 0, 0, 0)
            print(f'2')
            time.sleep(3.5)
            if self.timer_detector(3, 3):
                print(f'find, return')
                return
            self.drone_target_pose( x - c, y, z, 0, 0, 0,0)
            print(f'3')
            time.sleep(3.5)
            if self.timer_detector(3, 3):
                print(f'find, return')
                return
            self.drone_target_pose( x, y - c, z, 0, 0, 0, 0)
            print(f'4')
            time.sleep(3.5)
            if self.timer_detector(3, 3):
                print(f'find, return')
                return
        else:
            self.drone_target_pose_sim(x + c, y, z, 0, 0, 0, 1)
            print(f'1')
            time.sleep(1.5)
            if self.timer_detector(2, 3):
                print(f'find, return')
                return
            self.drone_target_pose_sim(x, y + c, z, 0, 0, 0, 1)
            print(f'2')
            time.sleep(1.5)
            if self.timer_detector(2, 3):
                print(f'find, return')
                return
            self.drone_target_pose_sim(x - c, y, z, 0, 0, 0, 1)
            print(f'3')
            time.sleep(1.5)
            if self.timer_detector(2, 3):
                print(f'find, return')
                return
            self.drone_target_pose_sim(x, y - c, z, 0, 0, 0, 1)
            print(f'4')
            time.sleep(1.5)
            if self.timer_detector(2, 3):
                print(f'find, return')
                return


    def execute(self, userdata):
        print('begin follow')

        self.takeoff_x = userdata.takeoff_position[0]
        self.takeoff_y = userdata.takeoff_position[1]
        self.takeoff_z = userdata.takeoff_position[2]
        self.rm = userdata.rm
        self.beginfollow = 1
        while not rospy.is_shutdown():
            # self.scanning(self.takeoff_x, self.takeoff_y, self.takeoff_z + self.land_offset, 0.2)
            self.scanning(self.takeoff_x, self.takeoff_y, self.takeoff_z + self.land_offset , 0.1)
            if self.timer_detector_marker <= 0:
                break
            # self.scanning(self.takeoff_x, self.takeoff_y, self.takeoff_z + self.land_offset, 0.3)
            self.scanning(self.takeoff_x, self.takeoff_y, self.takeoff_z + self.land_offset, 0.2)

            if self.timer_detector_marker <= 0:
                break
        time.sleep(5)
        return 'succeeded'

class AlignAndLand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'], input_keys=['rm'])

        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self._callback_dog_position)

        self.pub_land = rospy.Publisher('/quadrotor/teleop_command/land', Empty, queue_size=10)
        self.pub_qilin_vel = rospy.Publisher('/go1/cmd_vel', Twist, queue_size=10)
        self.pub_sim_pose = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

        self.miss_first_time, self.miss_second_time = rospy.Time.now(), rospy.Time.now()
        self.find_valve_tag = 0
        self.find_drone_tag = 0
        self.beginfollow = 0
        self.detection = 0.0
        self.flag = 0
        self.rm= 0
        self.timer_detector_marker = 5

        self.lx, self.ly, self.lz = 0, 0, 0
        self.april_x, self.april_y, self.april_z = 0.0, 0.0, 0.0
        self.april_drone_x, self.april_drone_y, self.april_drone_z, self.april_drone_yaw = 0.0, 0.0, 0.0, 0.0
        self.april_valve_x, self.april_valve_y, self.april_valve_z, self.april_valve_yaw = 0.0, 0.0, 0.0, 0.0
        self.dog_x, self.dog_y, self.dog_z, self.dog_yaw = 0.0, 0.0, 0.0, 0.0

    def _callback_apriltag(self, data):
        current_time = rospy.Time.now()
        # print(f'apriltag:{current_time.to_sec()}')
        # get the apriltag`s position information compare with camera coordination
        if data.detections:
            self.find_valve_tag = self.find_target_tag(data.detections,1)
            self.find_drone_tag = self.find_target_tag(data.detections, 0)
            if self.find_drone_tag == 1:
                if  self.beginfollow == 1:
                    if self.rm == 1:
                        self.align_dog_with_drone()
                    else:
                        self.align_dog_with_drone_sim()

        else:
            self.find_valve_tag = 0
            self.find_drone_tag = 0
            if self.beginfollow == 1:
                self.miss_second_time = rospy.Time.now()
                duration = self.miss_second_time - self.miss_first_time
                # print(f'{duration.to_sec()}')

                if duration.to_sec() > 0.5:
                    self.qilin_cmd_vel(0, 0, 0, 0, 0)
                    self.miss_first_time = self.miss_second_time

    def _callback_dog_position(self, data):
        self.dog_x= data.pose[2].position.x
        self.dog_y= data.pose[2].position.y
        self.dog_z= data.pose[2].position.z

    def sim_pose(self, px, py, ox, oy, oz, ow):

        sim_pose = ModelState()
        sim_pose.model_name = 'unitree'
        sim_pose.pose.position.x = px
        sim_pose.pose.position.y = py
        sim_pose.pose.orientation.x = ox
        sim_pose.pose.orientation.y = oy
        sim_pose.pose.orientation.z = oz
        sim_pose.pose.orientation.w = ow
        sim_pose.reference_frame = 'world'
        self.pub_sim_pose.publish(sim_pose)

    def find_target_tag(self,data,target_id):
        b = len(data)
        a = 0

        while a < b:
            c = target_id in data[a].id
            # print(f'{a}')
            if c:
                if target_id == 0:
                    self.april_drone_x = -data[a].pose.pose.pose.position.y
                    self.april_drone_y = data[a].pose.pose.pose.position.x
                    self.april_drone_z = data[a].pose.pose.pose.position.z
                    self.april_drone_yaw = tft.euler_from_quaternion([data[a].pose.pose.pose.orientation.x,
                                                                          data[a].pose.pose.pose.orientation.y,
                                                                          data[a].pose.pose.pose.orientation.z,
                                                                          data[a].pose.pose.pose.orientation.w])[2]

                if target_id == 1:
                    self.april_valve_x = -data[a].pose.pose.pose.position.y
                    self.april_valve_y = data[a].pose.pose.pose.position.x
                    self.april_valve_z = data[a].pose.pose.pose.position.z
                    self.april_valve_yaw = tft.euler_from_quaternion([data[a].pose.pose.pose.orientation.x,
                                                         data[a].pose.pose.pose.orientation.y,
                                                         data[a].pose.pose.pose.orientation.z,
                                                         data[a].pose.pose.pose.orientation.w])[2]

                return 1
            else:
                return 0

    def align_dog_with_drone(self):
        self.lx = 1.2 * self.april_drone_x
        self.ly = 1.2 * self.april_drone_y
        self.april_z = 0.03 *self.april_drone_yaw
        if self.april_z > 0.3:
            self.april_z = 0.3
        if self.april_z < -0.3:
            self.april_z = -0.3
        # print("align speed lx, ly, :", self.lx, self.ly, self.april_z)
        # print(f'apriltag_time:{apriltag_time.to_sec()}')
        if abs(self.lx) < 2 and abs(self.ly) < 2:
            navigation_time = rospy.Time.now()
            # print(f'navigation_time:{navigation_time.to_sec()}')
            self.qilin_cmd_vel(self.lx, self.ly, 0, 0, self.april_z)

    def align_dog_with_drone_sim(self):
        print(f"This is the align:{self.april_drone_x},{self.april_drone_y}")
        if abs(self.april_drone_x) >0.1 or abs(self.april_drone_y) >0.1:
            print(f'This is the dog:{self.dog_x},{self.dog_y},april_drone:{self.april_drone_x},{self.april_drone_y}')
            self.lx = self.april_drone_x + self.dog_x
            self.ly = self.april_drone_y + self.dog_y
            self.sim_pose(self.lx, self.ly, 0, 0, 0, 1)

    def land(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing land command...")
        empty_msg = Empty()
        self.pub_land.publish(empty_msg)

    def drone_landing_detection(self, i):
        r = rospy.Rate(i)
        number = i
        while not rospy.is_shutdown():
            number = number - 1
            if math.sqrt(self.april_drone_x ** 2 + self.april_drone_y ** 2) < 0.03 and abs(self.april_drone_yaw) < 0.5:
            # if math.sqrt(self.april_drone_x ** 2 + self.april_drone_y ** 2) < 0.5:
                i = i - 1
            if number == 0:
                break
            r.sleep()
        self.flag = i

    def drone_landing_condition(self):
        while not rospy.is_shutdown():
            if self.april_drone_x != 0:
                break
            time.sleep(0.1)

        i = 1
        plus = 0
        self.flag = 0

        while i > 0:
            i = i - 1
            self.drone_landing_detection(10)
            plus = plus + self.flag
            print(f'plus = {plus}')
        if plus == 0:
            self.beginfollow = 0
            # self.qilin_cmd_vel(0, 0, 0, 0, 0)
            self.land()
            print(f'landon')

    def qilin_cmd_vel(self, lx, ly, ax, ay, az):

        qilin_cmd_vel = Twist()
        qilin_cmd_vel.linear.x = lx
        qilin_cmd_vel.linear.y = ly
        qilin_cmd_vel.angular.x = ax
        qilin_cmd_vel.angular.y = ay
        qilin_cmd_vel.angular.z = az

        self.pub_qilin_vel.publish(qilin_cmd_vel)

    def execute(self, userdata):
        self.rm = userdata.rm
        self.beginfollow = 1
        current_time = rospy.Time.now()
        a = current_time.to_sec()
        b = current_time.to_sec()
        while b - a < 10:
            current_time = rospy.Time.now()
            b=current_time.to_sec()
            self.drone_landing_condition()
            if self.beginfollow== 0:
                return 'succeeded'
        return 'failed'
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
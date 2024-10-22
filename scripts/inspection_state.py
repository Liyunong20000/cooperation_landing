#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import numpy as np
import time, math, threading
import smach
import smach_ros
from appdirs import user_data_dir
from lxml.html import Classes

from std_msgs.msg import Empty, UInt8
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from tf.tfwtf import rostime_delta

from aerial_robot_msgs.msg import FlightNav
from apriltag_ros.msg import AprilTagDetectionArray
from docutils.utils.smartquotes import default_smartypants_attr
from wx.lib.flashwin import clsID


# The Cooperative inspection system by mini_quadrotor and Qilin
class Start(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded'])
        rospy.wait_for_service('/go1/sit')
        rospy.wait_for_service('/go1/stand')
        self.service_client_sit = rospy.ServiceProxy('/go1/sit', Trigger)
        self.service_client_stand = rospy.ServiceProxy('/go1/stand', Trigger)

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

        self.stand()
        return 'succeeded'

class MarkerSearch(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded'])

        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)

        self.pub_qilin_vel = rospy.Publisher('/go1/cmd_vel', Twist, queue_size=10)

        self.find_valve_tag, self.find_drone_tag = 0, 0

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

    def execute(self, userdata):
        while not self.find_valve_tag:
            self.qilin_cmd_vel(0.2, 0, 0, 0, 0)
        time.sleep(2)
        self.qilin_cmd_vel(0, 0, 0, 0, 0)
        return 'succeeded'

class TargetCalculation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],io_keys=['takeoff_x', 'takeoff_y', 'takeoff_z',
                                                                   'takeoff_yaw','target_x', 'target_y',
                                                                   'target_z'])

        rospy.Subscriber('/quadrotor/uav/cog/odom', Odometry, self._callback_position)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)

        self.pub_land = rospy.Publisher('/quadrotor/teleop_command/land', Empty, queue_size=10)

        self.valve2tag_x, self.valve2tag_y, self.valve2tag_z= 0.0, 2.0, 1.0
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
        self.drone_roll = self.quaternion_to_euler_angle(self.drone_ori_x, self.drone_ori_y, self.drone_ori_z, self.drone_ori_w)[0]
        self.drone_pitch = self.quaternion_to_euler_angle(self.drone_ori_x, self.drone_ori_y, self.drone_ori_z, self.drone_ori_w)[1]
        self.drone_yaw = self.quaternion_to_euler_angle(self.drone_ori_x, self.drone_ori_y, self.drone_ori_z, self.drone_ori_w)[2]

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

    def quaternion_to_euler_angle(self, x, y, z, w):
        R = np.array([[1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y],
                      [2 * x * y + 2 * w * z, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * w * x],
                      [2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x ** 2 - 2 * y ** 2]])
        theta_x = math.degrees(np.arctan2(R[2, 1], R[2, 2]))
        theta_y = math.degrees(np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2)))
        theta_z = math.degrees(np.arctan2(R[1, 0], R[0, 0]))
        theta = np.array([theta_x, theta_y, theta_z])
        return theta

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
                    self.april_valve_yaw = self.quaternion_to_euler_angle(data[a].pose.pose.pose.orientation.x,
                                                         data[a].pose.pose.pose.orientation.y,
                                                         data[a].pose.pose.pose.orientation.z,
                                                         data[a].pose.pose.pose.orientation.w)[2]
                return 1
            else:
                return 0

    def land(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing land command...")
        empty_msg = Empty()
        self.pub_land.publish(empty_msg)

    def record_takeoff_position(self):
        self.takeoff_x = self.drone_x
        self.takeoff_y = self.drone_y
        self.takeoff_z = self.drone_z
        self.takeoff_yaw = self.drone_yaw
        print(f'takeoff position:{self.takeoff_x}, {self.takeoff_y},{self.takeoff_z}, {self.takeoff_yaw}')

    def execute(self, userdata):
        self.record_takeoff_position()
        self.valve_x= self.takeoff_x + self.april_valve_x + self.valve2tag_x
        self.valve_y= self.takeoff_y + self.april_valve_y + self.valve2tag_y
        self.valve_z= self.april_valve_z - self.valve2tag_z
        userdata.takeoff_position = np.array([self.takeoff_x,self.takeoff_y,self.takeoff_z,self.takeoff_yaw])
        userdata.target_position = np.array([self.valve_x,self.valve_y,self.valve_z])

        time.sleep(1)

class UavTakeoff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.pub_takeoff = rospy.Publisher('/quadrotor/teleop_command/takeoff', Empty, queue_size=10)

        self.drone_x, self.drone_y, self.drone_z, self.drone_roll, self.drone_pitch, self.drone_yaw = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

        self.user_input = None

    def takeoff(self):
        time.sleep(0.5)
        if (-10 < self.drone_x <10) and (-10 < self.drone_y <10) and (-0.5 < self.drone_z <3):
            rospy.loginfo("Publishing takeoff command...")
            empty_msg = Empty()
            self.pub_takeoff.publish(empty_msg)
        else:
            rospy.loginfo("Don`t takeoff, state is wrong!!!")

    def execute(self, userdata):
        print(f'takeoff?')
        self.user_input = input()
        if self.user_input == 'y':
            print(f'yes')
            self.takeoff()
            return 'succeeded'
        else:
            return 'failed'

class FlyTarget(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['target_position'])
        self.pub_drone_nav = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)
        self.pub_drone_nav_info = rospy.Publisher('/quadrotor/uav/nav/info', FlightNav, queue_size=10)
        self.pub_drone_nav_trigger = rospy.Publisher('/quadrotor/uav/nav/trigger', Empty, queue_size=10)

        self._seq = 0
        self.valve_x, self.valve_y, self.valve_z = 0.0, 0.0, 0.0
    def drone_nav_trigger(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing nav trigger command...")
        empty_msg = Empty()
        self.pub_drone_nav_trigger.publish(empty_msg)

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

        self.pub_drone_nav_info.publish(flight_nav_msg)
        rospy.sleep(0.5)
        self.drone_nav_trigger()


    def execute(self, userdata):
        self.valve_x, self.valve_y, self.valve_z = userdata.target_position[0], userdata.target_position[1], userdata.target_position[2]
        self.drone_nav_info(2, self.valve_x, self.valve_y, 2, self.valve_z, 0, 0, 0)
        time.sleep(3)

class Inspection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.pub_drone_nav_info = rospy.Publisher('/quadrotor/uav/nav/info', FlightNav, queue_size=10)
        self.pub_drone_nav_trigger = rospy.Publisher('/quadrotor/uav/nav/trigger', Empty, queue_size=10)

    def drone_nav_trigger(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing nav trigger command...")
        empty_msg = Empty()
        self.pub_drone_nav_trigger.publish(empty_msg)

    def drone_nav_info(self, x_y_mode, x, y, z_mode, z, yaw_mode, omega_z, yaw):
        flight_nav_msg = FlightNav()
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

        self.pub_drone_nav_info.publish(flight_nav_msg)
        rospy.sleep(0.5)
        self.drone_nav_trigger()

    def execute(self, userdata):
        self.drone_nav_info(0, 0, 0, 0, 0, 4, 0.2, 90)
        time.sleep(2)
        self.drone_nav_info(0, 0, 0, 0, 0, 4, 0.2, 180)
        time.sleep(2)
        self.drone_nav_info(0, 0, 0, 0, 0, 4, 0.2, -90)
        time.sleep(2)
        self.drone_nav_info(0, 0, 0, 0, 0, 4, 0.2, 0)
        time.sleep(2)
        self.drone_nav_info(0, 0, 0, 0, 0, 4, 0.2, 90)
        time.sleep(2)
        self.drone_nav_info(0, 0, 0, 0, 0, 4, 0.2, 180)
        time.sleep(2)
        self.drone_nav_info(0, 0, 0, 0, 0, 4, 0.2, -90)
        time.sleep(2)
        self.drone_nav_info(0, 0, 0, 0, 0, 4, 0.2, 0)
        time.sleep(2)

class FlyBack(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],input_keys=['takeoff_position'])
        self.pub_drone_nav_info = rospy.Publisher('/quadrotor/uav/nav/info', FlightNav, queue_size=10)
        self.pub_drone_nav_trigger = rospy.Publisher('/quadrotor/uav/nav/trigger', Empty, queue_size=10)

        self.takeoff_x, self.takeoff_y, self.takeoff_z, self.takeoff_yaw = 0.0, 0.0, 0.0, 0.0
        self.land_offset = 0.4
    def drone_nav_trigger(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing nav trigger command...")
        empty_msg = Empty()
        self.pub_drone_nav_trigger.publish(empty_msg)

    def drone_nav_info(self, x_y_mode, x, y, z_mode, z, yaw_mode, omega_z, yaw):
        flight_nav_msg = FlightNav()
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

        self.pub_drone_nav_info.publish(flight_nav_msg)
        rospy.sleep(0.5)
        self.drone_nav_trigger()

    def execute(self, userdata):
        self.takeoff_x = userdata.takeoff_position[0]
        self.takeoff_y = userdata.takeoff_position[1]
        self.takeoff_z = userdata.takeoff_position[2]
        self.drone_nav_info(2, self.takeoff_x, self.takeoff_y, 2, self.takeoff_z+ self.land_offset, 0, 0, 0)

class VisibilityAdjustment(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['takeoff_position'])

        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)

        self.pub_drone_nav_info = rospy.Publisher('/quadrotor/uav/nav/info', FlightNav, queue_size=10)
        self.pub_drone_nav_trigger = rospy.Publisher('/quadrotor/uav/nav/trigger', Empty, queue_size=10)

        self.takeoff_x, self.takeoff_y, self.takeoff_z, self.takeoff_yaw = 0.0, 0.0, 0.0, 0.0
        self.land_offset = 0.4

        self.find_drone_tag = 0
        self.timer_detector_maker = 0
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

    def find_target_tag(self,data,target_id):
        b = len(data)
        a = 0

        while a < b:
            c = target_id in data[a].id
            # print(f'{a}')
            if c:
                if target_id == 0:
                    return 1
            else:
                return 0

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

    def drone_nav_trigger(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing nav trigger command...")
        empty_msg = Empty()
        self.pub_drone_nav_trigger.publish(empty_msg)

    def drone_nav_info(self, x_y_mode, x, y, z_mode, z, yaw_mode, omega_z, yaw):
        flight_nav_msg = FlightNav()
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

        self.pub_drone_nav_info.publish(flight_nav_msg)
        rospy.sleep(0.5)
        self.drone_nav_trigger()

    def scanning(self,x,y,z,c):
        self.drone_nav_info(2, x+c, y, 2, z, 0, 0, 0)
        print(f'1')
        time.sleep(1.5)
        if self.timer_detector(2,3):
            print(f'find, return')
            return
        self.drone_nav_info(2, x, y+c, 2, z, 0, 0, 0)
        print(f'2')
        time.sleep(1.5)
        if self.timer_detector(2,3):
            print(f'find, return')
            return
        self.drone_nav_info(2, x-c, y, 2, z, 0, 0, 0)
        print(f'3')
        time.sleep(1.5)
        if self.timer_detector(2,3):
            print(f'find, return')
            return
        self.drone_nav_info(2, x, y-c, 2, z, 0, 0, 0)
        print(f'4')
        time.sleep(1.5)
        if self.timer_detector(2,3):
            print(f'find, return')
            return

    def execute(self, userdata):
        print('begin follow')
        while not rospy.is_shutdown():
            self.scanning(self.takeoff_x, self.takeoff_y, self.takeoff_z + self.land_offset, 0.2)
            if self.timer_detector_marker <= 0:
                break
            self.scanning(self.takeoff_x, self.takeoff_y, self.takeoff_z + self.land_offset, 0.3)
            if self.timer_detector_marker <= 0:
                break
        return 'succeeded'

class AlignAndLand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','false'])

        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)

        self.pub_land = rospy.Publisher('/quadrotor/teleop_command/land', Empty, queue_size=10)
        self.pub_qilin_vel = rospy.Publisher('/go1/cmd_vel', Twist, queue_size=10)

        self.miss_first_time, self.miss_second_time = rospy.Time.now(), rospy.Time.now()
        self.find_valve_tag = 0
        self.find_drone_tag = 0
        self.beginfollow = 0
        self.detection = 0.0
        self.flag = 0
        self.timer_detector_marker = 5

        self.lx, self.ly, self.lz = 0, 0, 0
        self.april_x, self.april_y, self.april_z = 0.0, 0.0, 0.0
        self.april_drone_x, self.april_drone_y, self.april_drone_z, self.april_drone_yaw = 0.0, 0.0, 0.0, 0.0
        self.april_valve_x, self.april_valve_y, self.april_valve_z, self.april_valve_yaw = 0.0, 0.0, 0.0, 0.0

    def _callback_apriltag(self, data):
        current_time = rospy.Time.now()
        # print(f'apriltag:{current_time.to_sec()}')

        # get the apriltag`s position information compare with camera coordination
        if data.detections:
            self.find_valve_tag = self.find_target_tag(data.detections,1)
            self.find_drone_tag = self.find_target_tag(data.detections, 0)
            if self.find_drone_tag == 1:
                if  self.beginfollow == 1:
                    self.align_dog_with_drone()

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
                    self.april_drone_yaw = self.quaternion_to_euler_angle(data[a].pose.pose.pose.orientation.x,
                                                                          data[a].pose.pose.pose.orientation.y,
                                                                          data[a].pose.pose.pose.orientation.z,
                                                                          data[a].pose.pose.pose.orientation.w)[2]

                if target_id == 1:
                    self.april_valve_x = -data[a].pose.pose.pose.position.y
                    self.april_valve_y = data[a].pose.pose.pose.position.x
                    self.april_valve_z = data[a].pose.pose.pose.position.z
                    self.april_valve_yaw = self.quaternion_to_euler_angle(data[a].pose.pose.pose.orientation.x,
                                                         data[a].pose.pose.pose.orientation.y,
                                                         data[a].pose.pose.pose.orientation.z,
                                                         data[a].pose.pose.pose.orientation.w)[2]

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
            if math.sqrt(self.april_drone_x ** 2 + self.april_drone_y ** 2) < 0.03 and abs(self.april_drone_yaw) < 10:
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
        while not rospy.is_shutdown():
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
                self.qilin_cmd_vel(0, 0, 0, 0, 0)
                self.land()
                print(f'landon')
                break

    def quaternion_to_euler_angle(self, x, y, z, w):
        R = np.array([[1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y],
                      [2 * x * y + 2 * w * z, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * w * x],
                      [2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x ** 2 - 2 * y ** 2]])
        theta_x = math.degrees(np.arctan2(R[2, 1], R[2, 2]))
        theta_y = math.degrees(np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2)))
        theta_z = math.degrees(np.arctan2(R[1, 0], R[0, 0]))
        theta = np.array([theta_x, theta_y, theta_z])
        return theta

    def qilin_cmd_vel(self, lx, ly, ax, ay, az):
        qilin_cmd_vel = Twist()
        qilin_cmd_vel.linear.x = lx
        qilin_cmd_vel.linear.y = ly
        qilin_cmd_vel.angular.x = ax
        qilin_cmd_vel.angular.y = ay
        qilin_cmd_vel.angular.z = az

        self.pub_qilin_vel.publish(qilin_cmd_vel)

    def execute(self, userdata):
        self.beginfollow = 1
        a = rospy.get_time()
        if rospy.get_time() - a < 10:
            self.drone_landing_condition()
            return 'succeeded'
        else:
            return 'failed'
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])

    def execute(self, userdata):
        return 'succeeded'
class Finish(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted'])

    def execute(self, userdata):
        return 'preempted'
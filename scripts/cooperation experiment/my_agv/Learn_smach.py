#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import smach
import smach_ros
import time
import math
import numpy as np
import smach
import tf
from std_msgs.msg import Empty, UInt8
from aerial_robot_msgs.msg import FlightNav
from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo
from smach import StateMachine

# It is for  the Cooprative mission for UAV and ground robot
class Takeoff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeed'],output_keys=['takeoff_x', 'takeoff_y', 'takeoff_z'])
        # publisher
        self.pub_start = rospy.Publisher('/quadrotor/teleop_command/start', Empty, queue_size=10)
        self.pub_takeoff = rospy.Publisher('/quadrotor/teleop_command/takeoff', Empty, queue_size=10)
        # subscriber
        rospy.Subscriber('/quadrotor/uav/cog/odom', Odometry, self._callback_position)
    def _callback_position(self, odom_msg):
        self.drone_x = odom_msg.pose.pose.position.x
        self.drone_y = odom_msg.pose.pose.position.y
        self.drone_z = odom_msg.pose.pose.position.z
    def execute(self, userdata):
        rospy.loginfo('Executing state takeoff')

        # takeoff_x = self.drone_x
        # takeoff_y = self.drone_y
        # takeoff_z = self.drone_z
        # print(f'{takeoff_x},{takeoff_y},{takeoff_z}')
        empty_msg = Empty()
        self.pub_start.publish(empty_msg)
        time.sleep(3)
        self.pub_takeoff.publish(empty_msg)
        rospy.loginfo("Publishing takeoff command...")
        time.sleep(3)
        return 'succeed'
class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished', 'next'],input_keys=['input_count'], output_keys=['output_count'])
        # publisher
        self.pub_drone_nav = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)
        self._seq = 0
    def drone_nav_info(self, x, y, z):
        flight_nav_msg = FlightNav()
        flight_nav_msg.header.seq = self._seq
        self._seq += 1
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
    def execute(self, userdata):
        rospy.loginfo('Executing state Move')
        if userdata.input_count < 11:
            print(f'{userdata.input_count}')
            self.drone_nav_info(userdata.input_count,userdata.input_count,userdata.input_count)
            userdata.output_count = userdata.input_count
            time.sleep(2)
            return 'next'
        else:
            return 'finished'
class Count(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next'], input_keys=['input_count'], output_keys=['output_count'])
        # publisher

    def execute(self, userdata):
        rospy.loginfo('Executing state Count')
        userdata.output_count = userdata.input_count + 1
        return 'next'
class Landon(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeed'])
        # publisher
        self.pub_land = rospy.Publisher('/quadrotor/teleop_command/land', Empty, queue_size=10)
    def execute(self, userdata):
        rospy.loginfo('Executing state landon')
        empty_msg = Empty()
        self.pub_land.publish(empty_msg)
        return 'succeed'
# class Foo(smach.State):
#
#     def __init__(self,outcomes=['succeeded', 'failed']):
#         print(f'Hi, I am Cloud Cube')
#         rospy.init_node('land_smach', anonymous=True)
#
#         self.drone_x, self.drone_y, self.drone_z = 0.0, 0.0, 0.0
#         self.april_x, self.april_y, self.april_z = 0.0, 0.0, 0.0
#         self.D = 0
#
#         self.time_rece = rospy.Time()
#         self._seq = 0
#         self.state = 0
#         self.beginland = 0
#         self.beginfollow = 0
#         self.flag = 0
#
#         # Subscribe and publish.
#         rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)
#
#         rospy.Subscriber('/quadrotor/flight_state', UInt8, self._callback_state)
#
#
#
#
#         rospy.set_param('/above_z', 0.3)
#         self.above_z = rospy.get_param("/above_z")
#
#     def _callback_apriltag(self, data):
#         if data.detections:
#             # rospy.loginfo("????????????????????????????????detect tag!")
#             a = data.detections[0]
#             self.april_x = a.pose.pose.pose.position.x
#             self.april_y = a.pose.pose.pose.position.y
#             quaternion_x = a.pose.pose.pose.orientation.x
#             quaternion_y = a.pose.pose.pose.orientation.y
#             quaternion_z = a.pose.pose.pose.orientation.z
#             quaternion_w = a.pose.pose.pose.orientation.w
#
#             self.april_z = self.quaternion_to_euler_angle(quaternion_x,quaternion_y,quaternion_z,quaternion_w)
#             #print("euler_z (degree):", self.april_z)
#             # self.D = 1
#             if self.beginfollow == 1:
#
#                 self.lvol_x = self.move_parameter * self.april_y
#                 self.lvol_y = - self.move_parameter * self.april_x
#                 self.anglevol_z = 0.03 * self.move_parameter * self.april_z
#                 if abs(self.lvol_x) < 0.5 and abs(self.lvol_y) < 0.5:
#                     self.agv_nav_info(self.lvol_x, self.lvol_y, self.anglevol_z)
#         else:
#             self.D = 0
#             if self.beginfollow == 1:
#                 self.agv_nav_info(0, 0, 0)
#
#
#
#     # get the drone state (hover?)
#     def _callback_state(self, msg):
#         self.state = msg.data
#
#     # drone land
#     def land(self):
#         time.sleep(0.5)
#         rospy.loginfo("Publishing land command...")
#         empty_msg = Empty()
#         self.pub_land.publish(empty_msg)
#
#
#
#
#     def agv_nav_info(self, lx, ly, az):
#         agv_nav_msg = Twist()
#         agv_nav_msg.linear.x = lx
#         agv_nav_msg.linear.y = ly
#         agv_nav_msg.angular.z = az
#
#         self.pub_agv_nav.publish(agv_nav_msg)
#
#
#
#     def drone_landing_detection(self, i):
#         r = rospy.Rate(i)
#         number = i
#         while not rospy.is_shutdown():
#             number = number - 1
#             if math.sqrt(self.april_x ** 2 + self.april_y ** 2) < 0.03 and abs(self.april_z) < 10:
#                 i = i - 1
#             if number == 0:
#                 break
#             r.sleep()
#         self.flag = i
#
#     def drone_landing_condition(self):
#         while not rospy.is_shutdown():
#             i = 1
#             plus = 0
#             self.flag = 0
#
#             while i > 0:
#                 i = i - 1
#                 self.drone_landing_detection(10)
#                 plus = plus + self.flag
#                 print(f'plus = {plus}')
#             if plus == 0:
#                 self.beginfollow = 0
#                 self.land()
#                 print(f'landon')
#                 break
#
#     def quaternion_to_euler_angle(self, x, y, z, w):
#
#         R = np.array([[1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y],
#                       [2 * x * y + 2 * w * z, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * w * x],
#                       [2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x ** 2 - 2 * y ** 2]])
#         # theta_x = math.degrees(np.arctan2(R[2, 1], R[2, 2]))
#         # theta_y = math.degrees(np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2)))
#         theta_z = math.degrees(np.arctan2(R[1, 0], R[0, 0]))
#         return theta_z
#
#     def come_back(self):
#         while not rospy.is_shutdown():
#             if self.state == 5:
#                 break
#             time.sleep(0.1)
#         tz = self.takeoff_z + self.above_z
#         self.drone_nav_info(self.takeoff_x + 0.2, self.takeoff_y + 0.1, tz + 0.5)
#         time.sleep(4)
#         self.drone_nav_info(self.takeoff_x, self.takeoff_y, tz)
#         print(f'Move to above takeoff_Z')
#         # self.converge(self.takeoff_x, self.takeoff_y, tz)
#         time.sleep(1)
#         self.beginfollow = 1
#
#     # def move(self, x, y):
#     #     self.lvol_x = x
#     #     self.lvol_y = y
#     #     if abs(self.lvol_x) < 0.5 and abs(self.lvol_y) < 0.5:
#     #         self.agv_nav_info(self.lvol_x, self.lvol_y, 0)


def main():
    rospy.init_node('smach_test_demo')
    sm_top = smach.StateMachine(outcomes=['finish'])
    smach.UserData.sm_count = 1
    smach.UserData.takeoff_x = 1
    smach.UserData.takeoff_y = 1
    smach.UserData.takeoff_z = 1
    with sm_top:

        smach.StateMachine.add('TAKEOFF', Takeoff(), transitions={'succeed': 'SUB'}, remapping={'takeoff_x': 'takeoff_x', 'takeoff_y': 'takeoff_y', 'takeoff_z': 'takeoff_z'})

        sm_sub = smach.StateMachine(outcomes=['succeed'])

        with sm_sub:
            smach.StateMachine.add('MOVE', Move(), transitions={'finished': 'LANDON', 'next': 'COUNT'},remapping={'input_count': 'sm_count', 'output_count': 'sm_count' })
            smach.StateMachine.add('COUNT', Count(), transitions={'next': 'MOVE'}, remapping={'input_count': 'sm_count', 'output_count': 'sm_count' })
            smach.StateMachine.add('LANDON', Landon(), transitions={'succeed': 'succeed'})
        smach.StateMachine.add('SUB', sm_sub, transitions={'succeed': 'finish'})
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

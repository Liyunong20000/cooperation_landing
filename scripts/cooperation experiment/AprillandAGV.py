#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import time
import math
import numpy as np
import tf
from std_msgs.msg import Empty, UInt8
from aerial_robot_msgs.msg import FlightNav
from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo


# It is for  the Coopration for Mini_Quadrotor and MyAGV

# use the class to create a node

class AprillandagvNode:

    def __init__(self):  # This part will work when this node is used.
        print(f'Hi, I am Cloud Cube')
        rospy.init_node('Aprillandagv', anonymous=True)

        self.drone_x, self.drone_y, self.drone_z = 0.0, 0.0, 0.0
        self.takeoff_x, self.takeoff_y, self.takeoff_z = 0.0, 0.0, 0.0
        self.agv_x, self.agv_y, self.agv_z = 0.0, 0.0, 0.0
        self.lvol_x, self.lvol_y, self.lvol_z = 0.0, 0.0, 0.0
        self.anglevol_x, self.anglevol_y, self.anglevol_z = 0.0, 0.0, 0.0
        self.april_x, self.april_y, self.april_z = 0.0, 0.0, 0.0
        self.D = 0

        self.time_rece = rospy.Time()
        self._seq = 0
        self.state = 0
        self.beginland = 0
        self.beginfollow = 0
        self.flag = 0

        # Subscribe and publish.
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)
        rospy.Subscriber('/quadrotor/uav/cog/odom', Odometry, self._callback_position)
        rospy.Subscriber('/quadrotor/flight_state', UInt8, self._callback_state)
        self.pub_drone_nav = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)
        self.pub_takeoff = rospy.Publisher('/quadrotor/teleop_command/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('/quadrotor/teleop_command/land', Empty, queue_size=10)

        self.pub_agv_nav = rospy.Publisher('/cmd_vel', Twist, queue_size=10, tcp_nodelay=True)

        rospy.set_param('/move_parameter', 0.7)
        self.move_parameter = rospy.get_param("/move_parameter")
        rospy.set_param('/converge_interval', 0.05)
        self.converge_interval = rospy.get_param("/converge_interval")
        rospy.set_param('/above_z', 0.3)
        self.above_z = rospy.get_param("/above_z")

    def _callback_apriltag(self, data):
        if data.detections:
            # rospy.loginfo("????????????????????????????????detect tag!")
            a = data.detections[0]
            self.april_x = a.pose.pose.pose.position.x
            self.april_y = a.pose.pose.pose.position.y
            quaternion_x = a.pose.pose.pose.orientation.x
            quaternion_y = a.pose.pose.pose.orientation.y
            quaternion_z = a.pose.pose.pose.orientation.z
            quaternion_w = a.pose.pose.pose.orientation.w

            self.april_z = self.quaternion_to_euler_angle(quaternion_x,quaternion_y,quaternion_z,quaternion_w)
            #print("euler_z (degree):", self.april_z)
            # self.D = 1
            if self.beginfollow == 1:

                self.lvol_x = self.move_parameter * self.april_y
                self.lvol_y = - self.move_parameter * self.april_x
                self.anglevol_z = 0.03 * self.move_parameter * self.april_z
                if abs(self.lvol_x) < 0.5 and abs(self.lvol_y) < 0.5:
                    self.agv_nav_info(self.lvol_x, self.lvol_y, self.anglevol_z)
        else:
            self.D = 0
            if self.beginfollow == 1:
                self.agv_nav_info(0, 0, 0)

    # Get the position information of the drone
    def _callback_position(self, odom_msg):  
        self.drone_x = odom_msg.pose.pose.position.x
        self.drone_y = odom_msg.pose.pose.position.y
        self.drone_z = odom_msg.pose.pose.position.z

    # get the drone state (hover?)
    def _callback_state(self, msg):
        self.state = msg.data

    # drone takeoff
    def takeoff(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing takeoff command...")
        empty_msg = Empty()
        self.pub_takeoff.publish(empty_msg)
    
    # drone land
    def land(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing land command...")
        empty_msg = Empty()
        self.pub_land.publish(empty_msg)

    def converge(self, x, y, z):
        while not rospy.is_shutdown():
            if abs(self.drone_x - x) < self.converge_interval and abs(self.drone_y - y) < self.converge_interval and abs(self.drone_z - z) < self.converge_interval:  # reach hover state
                break
            time.sleep(0.1)

    # record the take off position
    def record_takeoff_position(self):
        self.takeoff_x = self.drone_x
        self.takeoff_y = self.drone_y
        self.takeoff_z = self.drone_z
        print(self.takeoff_x, self.takeoff_y)

    def agv_nav_info(self, lx, ly, az):
        agv_nav_msg = Twist()
        agv_nav_msg.linear.x = lx
        agv_nav_msg.linear.y = ly
        agv_nav_msg.angular.z = az

        self.pub_agv_nav.publish(agv_nav_msg)

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

    def drone_landing_detection(self, i):
        r = rospy.Rate(i)
        number = i
        while not rospy.is_shutdown():
            number = number - 1
            if abs(self.april_x) < 0.02 and abs(self.april_y) < 0.02 and abs(self.april_z) < 45:
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
                print(f'plus = {plus}')
            if plus == 0:
                self.beginfollow = 0
                self.land()
                print(f'landon')
                break

    def quaternion_to_euler_angle(self, x, y, z, w):

        R = np.array([[1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y],
                      [2 * x * y + 2 * w * z, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * w * x],
                      [2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x ** 2 - 2 * y ** 2]])
        # theta_x = math.degrees(np.arctan2(R[2, 1], R[2, 2]))
        # theta_y = math.degrees(np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2)))
        theta_z = math.degrees(np.arctan2(R[1, 0], R[0, 0]))
        return theta_z

    def come_back(self):
        while not rospy.is_shutdown():
            if self.state == 5:
                break
            time.sleep(0.1)
        tz = self.takeoff_z + self.above_z
        self.drone_nav_info(self.takeoff_x+0.03, self.takeoff_y, tz)
        print(f'Move to above takeoff_Z')
        self.converge(self.takeoff_x+0.03, self.takeoff_y, tz)
        time.sleep(1)
        self.beginfollow = 1

    # def move(self, x, y):
    #     self.lvol_x = x
    #     self.lvol_y = y
    #     if abs(self.lvol_x) < 0.5 and abs(self.lvol_y) < 0.5:
    #         self.agv_nav_info(self.lvol_x, self.lvol_y, 0)

if __name__ == '__main__':
    node = AprillandagvNode()
    time.sleep(3)
    node.record_takeoff_position()
    node.takeoff()
    node.come_back()

    node.drone_landing_condition()
    while not rospy.is_shutdown():
        rospy.spin()

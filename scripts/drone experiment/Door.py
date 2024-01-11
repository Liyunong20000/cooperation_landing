#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import time
import math
import tf2_ros
import tf
from std_msgs.msg import Empty, UInt8
from aerial_robot_msgs.msg import FlightNav
from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


# It is for  the first drone BoomBoom
# use the class to create a node

class DoorlandNode:

    def __init__(self):  # This part will work when this node is used.
        print(f'Hi, I am Cloud Cube.')
        rospy.init_node('Doorland', anonymous=True)

        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.Rx, self.Ry, self.Rz = 0.0, 0.0, 0.0
        self.Px, self.Py, self.Pz = 0.0, 0.0, 0.0
        self.Tx, self.Ty, self.Tz, self.Trz = 0.0, 0.0, 0.0, 0.0
        self.P = 0.6
        self._seq = 0
        self.state = 0

        # Subscribe and publish.
        #rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)
        rospy.Subscriber('/quadrotor/uav/cog/odom', Odometry, self._callback_position)
        rospy.Subscriber('/quadrotor/flight_state', UInt8, self._callback_state)
        self.pub_nav = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)
        self.pub_takeoff = rospy.Publisher('/quadrotor/teleop_command/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('/quadrotor/teleop_command/land', Empty, queue_size=10)


        #self.tf_buffer = tf2_ros.Buffer()
        # Initialize a TransformListener
        #self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # transformation

    def get_user_input(self):  # Use to input the number
        self.x = float(input("Enter x value: "))
        self.y = float(input("Enter y value: "))
        self.z = float(input("Enter z value: "))

    def takeoff(self):  # Use to land
        time.sleep(0.5)
        rospy.loginfo("Publishing takeoff command...")
        empty_msg = Empty()
        self.pub_takeoff.publish(empty_msg)
    def land(self):  # Use to land
        time.sleep(0.5)
        rospy.loginfo("Publishing land command...")
        empty_msg = Empty()
        self.pub_land.publish(empty_msg)

    def _callback_apriltag(self, data):
        # Get the information of the distance between the drone and the apriltag
        while not rospy.is_shutdown():
            try:
                transform = self.tf_buffer.lookup_transform('world', 'Target', rospy.Time.now(), rospy.Duration(0.1))
                self.Rx = transform.transform.translation.x
                self.Ry = transform.transform.translation.y
                self.Rz = transform.transform.translation.z

            except:
                continue

        # print(f'{self.Rx}  {self.Ry}  {self.Rz}')

    def _callback_position(self, odom_msg):  # Get the position information of the drone
        self.Px = odom_msg.pose.pose.position.x
        self.Py = odom_msg.pose.pose.position.y
        self.Pz = odom_msg.pose.pose.position.z

    def _callback_state(self, msg):
        self.state = msg.data

    def record_takeoff_position(self):
        self.Tx = self.Px
        self.Ty = self.Py
        self.Tz = self.Pz
        self.Trz = self.Tz + 0.4
        print(self.Px, self.Py)
        print(self.Tx, self.Ty)
    def nav_info(self, x, y, z) :
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

        self.pub_nav.publish(flight_nav_msg)


    def publish_flight_nav(self):  # Set a beginning point
        #print(f'Type the begin point')
        # Type the number
        #self.get_user_input()
        #self.nav_info(1, 0, 1.0)
        #time.sleep(3)
        #while not rospy.is_shutdown():
            #if -0.05 < self.Px < 0.05:
                #if -0.05 < self.Py - 0 < 0.05:
                    #print(f'I have arrived!')
                    #break
        self.nav_info(0.6, 0, 1.0)
        time.sleep(3)
        #self.nav_info(0.3, 0, 0.7)
        #time.sleep(3)


    def land_door(self):

        while not rospy.is_shutdown():
            if self.state == 5: # reach hover state
                break
            time.sleep(0.1)

        Tz = self.Tz + 0.4
        self.nav_info(self.Tx, self.Ty, Tz)
        print(f'Move to lower Z')
        while not rospy.is_shutdown():
            if self.Pz - Tz < 0.03:
                if math.sqrt((self.Px - self.Tx) ** 2 + (self.Py - self.Ty) ** 2) < 0.005:
                    print(f'Safe landing!')
                    print(f'X = {self.Px}, Y = {self.Py}')
                    self.land()
                    time.sleep(1)
                    sys.exit()


if __name__ == '__main__':
    node = DoorlandNode()
    time.sleep(3)
    node.record_takeoff_position()
    node.takeoff()
    node.land_door()
    while not rospy.is_shutdown():
        rospy.spin()

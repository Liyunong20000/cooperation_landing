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


# It is for  the Coopration for Mini_Quadrotor and MyAGV

# use the class to create a node

class AprillandagvNode:

    def __init__(self):  # This part will work when this node is used.
        print(f'Hi, I am Cloud Cube')
        rospy.init_node('Aprillandagv', anonymous=True)

        self.agv_x, self.agv_y, self.agv_z = 0.0, 0.0, 0.0
        self.drone_x, self.drone_y, self.drone_z = 0.0, 0.0, 0.0
        self.april_x, self.april_y, self.april_z = 0.0, 0.0, 0.0
        self.takeoff_x, self.takeoff_y, self.takeoff_Tz = 0.0, 0.0, 0.0
       
        self._seq = 0
        self.state = 0

        # Subscribe and publish.
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)
        rospy.Subscriber('/quadrotor/uav/cog/odom', Odometry, self._callback_position)
        rospy.Subscriber('/quadrotor/flight_state', UInt8, self._callback_state)
        self.pub_nav = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)
        self.pub_takeoff = rospy.Publisher('/quadrotor/teleop_command/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('/quadrotor/teleop_command/land', Empty, queue_size=10)
        # !!!订阅AGV位置的话题
        # !!!发布AGV移动的话题

        self.tf_buffer = tf2_ros.Buffer()
        # Initialize a TransformListener
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def _callback_apriltag(self, data):
        # get the apriltag`s position information compare with camera coordination
        # use transform to gain the
        transform = self.tf_buffer.lookup_transform('world', 'Target', rospy.Time.now(),rospy.Duration(0.1))
        self.april_x = transform.transform.translation.x
        self.april_y = transform.transform.translation.y
        self.april_z = transform.transform.translation.z

    
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

    
    # record the take off position
    def record_takeoff_position(self):
        self.takeoff_x = self.drone_x
        self.takeoff_y = self.drone_y
        self.takeoff_z = self.drone_z
        print(self.takeoff_x, self.takeoff_y)

    def nav_info(self, x, y, z):
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


    def land_camera(self):

        # self.get_user_input()
        while not rospy.is_shutdown():
            if self.state == 5: # reach hover state
                break
            time.sleep(0.1)

        tz = self.Tz + 0.3
        self.nav_info(self.Tx, self.Ty, tz)
        print(f'Move to higher Z')
        while not rospy.is_shutdown():
            if self.Pz - tz < 0.03:
                break
            time.sleep(0.1)
        print(f'Wait for 3 seconds!')
        time.sleep(3)

        while not rospy.is_shutdown():
            rx = self.Rx
            ry = self.Ry
            if rx != 0 and ry != 0:
                self.nav_info(rx, ry, tz)
                while not rospy.is_shutdown():
                    if (self.Px - rx) < 0.03 and (self.Py - ry) < 0.03:
                        break
                    time.sleep(0.1)
                break
            #break
        print(f'the apriltags position is rx ={rx}, ry = {ry}')
        while not rospy.is_shutdown():
            if math.sqrt((self.Px - rx) ** 2 + (self.Py - ry) ** 2) < 0.01:
                print(f'Safe landing!')
                print(f'Before landing on X = {self.Px}, Y = {self.Py}')
                self.land()
                time.sleep(1)
                print(f'After landing on X = {self.Px}, Y = {self.Py}')
                sys.exit()



if __name__ == '__main__':
    node = AprillandagvNode()
    time.sleep(3)
    # node.publish_flight_nav()
    node.record_takeoff_position()
    node.takeoff()
    node.land_camera()

    while not rospy.is_shutdown():
        rospy.spin()

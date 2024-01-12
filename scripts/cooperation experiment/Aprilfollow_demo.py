#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import time
import tf2_ros
import tf
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped


# It is for Apirltag following demo for MyAGV

# use the class to create a node

class AprilfollowNode:

    def __init__(self):  # This part will work when this node is used.
        print(f'Hi, I am Cloud Cube')
        rospy.init_node('Aprilfollow', anonymous=True)

        self.agv_x, self.agv_y, self.agv_z = 0.0, 0.0, 0.0
        self.lvol_x, self.lvol_y, self.lvol_z = 0.0, 0.0, 0.0
        self.avol_x, self.avol_y, self.avol_z = 0.0, 0.0, 0.0
        self.april_x, self.april_y, self.april_z = 0.0, 0.0, 0.0
        self._seq = 0

        # Subscribe and publish.
        #rospy.Subscriber('/odom', Odometry, self._callback_position)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)
        self.pub_nav = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #!!!订阅AGV位置的话题

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

        rate = rospy.Rate(10)
    # write a AGV nav function
    def agv_nav_info(self, lx, ly, lz, ax, ay, az):
        agv_nav_msg = Twist()
        agv_nav_msg.linear.x = lx
        agv_nav_msg.linear.y = ly
        agv_nav_msg.linear.z = lz

        agv_nav_msg.angular.x = ax
        agv_nav_msg.angular.y = ay
        agv_nav_msg.angular.z = az

        self.pub_nav.publish(agv_nav_msg)


    def follow_camera(self):
        while not rospy.is_shutdown():
            lvol_x = 0.5 * april_x
            lvol_y = 0.5 * april_y
            self.nav_info(self.lvol_x, self.lvol_y, self.lvol_z, self.avol_x, self.avol_y, self.avol_z)
            rate.sleep()


if __name__ == '__main__':
    node = AprilfollowNode()
    node.follow_camera()

    while not rospy.is_shutdown():
        rospy.spin()

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
        self.april_x, self.april_y, self.april_z = 0.0, 0.0, 0.0
       
        self._seq = 0
        self.state = 0

        # Subscribe and publish.
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)
        #!!!订阅AGV位置的话题
	    #!!!发布AGV移动的话题

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


    #写一个AGV的导航函数
    def agvnav_info(self, x, y, z):


    def follow_camera(self):
        while not rospy.is_shutdown():
            self.nav_info(self.april_x, self.april_y)



if __name__ == '__main__':
    node = AprilfollowNode()
    node.follow_camera()

    while not rospy.is_shutdown():
        rospy.spin()

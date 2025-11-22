#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from apriltag_ros.msg import AprilTagDetectionArray
import tf.transformations as tft
# from basic_function.basic_function import BasicNode
from basic_function import BasicNode

# use the class to create a node

class DogBasic:

    def __init__(self):  # This part will work when this node is used.

        # Subscribe and publish.

        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_tag_dog_info)

        self.pub_qilin_vel= rospy.Publisher('/go1/cmd_vel', Twist, queue_size=10)
        self.pub_qilin_pose = rospy.Publisher('/go1/body_pose', Pose, queue_size=10)
        #
        # rospy.wait_for_service('/go1/sit')
        # rospy.wait_for_service('/go1/stand')

        self.service_client_sit = rospy.ServiceProxy('/go1/sit', Trigger)
        self.service_client_stand = rospy.ServiceProxy('/go1/stand', Trigger)

        self.correction_tag_drone_x, self.correction_tag_drone_y, self.correction_tag_drone_z = 0.0, 0.0, 0.0
        self.correction_tag_drone_roll, self.correction_tag_drone_pitch, self.correction_tag_drone_yaw = 0.0, 0.0, 0.0

        self.correction_tag_13_x, self.correction_tag_13_y, self.correction_tag_13_z = 0.0, 0.0, 0.0
        self.correction_tag_13_roll, self.correction_tag_13_pitch, self.correction_tag_13_yaw = 0.0, 0.0, 0.0

        self.correction_tag_3_x, self.correction_tag_3_y, self.correction_tag_3_z = 0.0, 0.0, 0.0
        self.correction_tag_3_roll, self.correction_tag_3_pitch, self.correction_tag_3_yaw = 0.0, 0.0, 0.0

        self.correction_target_x, self.correction_target_y, self.correction_target_z = 0.0, 0.0, 0.0
        self.correction_target_roll, self.correction_target_pitch, self.correction_target_yaw = 0.0, 0.0, 0.0

        self.correction_err_x, self.correction_err_y, self.correction_err_z = 0.0, 0.0, 0.0
        self.correction_err_pitch, self.correction_err_roll, self.correction_err_yaw = 0.0, 0.0, 0.0

        self.basic= BasicNode()
    # Function for sit
    def sit(self):
        try:
            response = self.service_client_sit()
            if response.success:
                rospy.loginfo('Sit command executed successfully')
            else:
                rospy.logwarn('Stand command failed: %s', response.message)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)
    # Function for stand
    def stand(self):
        try:
            response = self.service_client_stand()
            if response.success:
                rospy.loginfo('Stand command executed successfully')
            else:
                rospy.logwarn('Stand command failed: %s', response.message)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)
    # velocity command
    def qilin_cmd_vel(self, lx, ly, ax, ay, az):
        qilin_cmd_vel = Twist()
        qilin_cmd_vel.linear.x = lx
        qilin_cmd_vel.linear.y = ly
        qilin_cmd_vel.angular.x = ax
        qilin_cmd_vel.angular.y = ay
        qilin_cmd_vel.angular.z = az

        self.pub_qilin_vel.publish(qilin_cmd_vel)
    # orientation command
    def qilin_body_pose(self, qx, qy, qz, qw):
        qilin_body_pose = Pose()
        qilin_body_pose.orientation.x = qx
        qilin_body_pose.orientation.y = qy
        qilin_body_pose.orientation.z = qz
        qilin_body_pose.orientation.w = qw
        rospy.sleep(0.1)
        self.pub_qilin_pose.publish(qilin_body_pose)
    # Get the target id and value the variable
    def tag_position_correction_tag_13(self,data,target_id):
        for det in data.detections:
            # print(f'{det}')
            # print(f'{det.id}')
            if target_id in det.id:
                pose = det.pose.pose.pose
                self.correction_tag_13_x = pose.position.x
                self.correction_tag_13_y = pose.position.y
                self.correction_tag_13_z = pose.position.z
                qx = pose.orientation.x
                qy = pose.orientation.y
                qz = pose.orientation.z
                qw = pose.orientation.w
                self.correction_tag_13_roll = \
                tft.euler_from_quaternion([qx, qy, qz, qw])[0]
                self.correction_tag_13_pitch = \
                tft.euler_from_quaternion([qx, qy, qz, qw])[1]
                self.correction_tag_13_yaw = \
                tft.euler_from_quaternion([qx, qy, qz, qw])[2]

    def tag_position_correction_tag_3(self,data,target_id):
        for det in data.detections:
            # print(f'{det}')
            # print(f'{det.id}')
            if target_id in det.id:
                pose = det.pose.pose.pose
                self.correction_tag_3_x = pose.position.x
                self.correction_tag_3_y = pose.position.y
                self.correction_tag_3_z = pose.position.z
                qx = pose.orientation.x
                qy = pose.orientation.y
                qz = pose.orientation.z
                qw = pose.orientation.w
                self.correction_tag_3_roll = \
                tft.euler_from_quaternion([qx, qy, qz, qw])[0]
                self.correction_tag_3_pitch = \
                tft.euler_from_quaternion([qx, qy, qz, qw])[1]
                self.correction_tag_3_yaw = \
                tft.euler_from_quaternion([qx, qy, qz, qw])[2]

    def _callback_tag_dog_info(self, msg):
        self.tag_dog_info = msg
        self.tag_position_correction_tag_13(self.tag_dog_info, 13)
        self.tag_position_correction_tag_3(self.tag_dog_info, 3)
        # if self.correction_tag_3_x == 0 and self.correction_tag_3_y == 0:
        #     return
        # self.correction_err_x = self.correction_tag_3_x - self.correction_tag_13_x +(self.basic.mocap_desk_x - self.basic.mocap_tag13_x)
        # self.correction_err_y = self.correction_tag_3_z + 0.3 - self.correction_tag_13_z - 0.15 + (self.basic.mocap_desk_y - self.basic.mocap_tag13_y)
        # self.correction_err_z = self.correction_tag_3_y + 0.15 + (self.basic.mocap_desk_z - self.basic.mocap_tag13_z) + 0.3 + 0.2 - (self.correction_tag_3_y + 0.1488)
        # # 0.2m offset between lidar and mocap coordinate in case of crush, 0.3m above desk
        # self.correction_err_yaw = self.correction_tag_3_pitch - self.correction_tag_13_pitch + (self.basic.mocap_desk_yaw - self.basic.mocap_tag13_yaw)
        # self.correction_tag_3_x, self.correction_tag_3_y, self.correction_tag_3_z = 0.0, 0.0, 0.0
        # self.correction_tag_3_roll, self.correction_tag_3_pitch, self.correction_tag_3_yaw = 0.0, 0.0, 0.0

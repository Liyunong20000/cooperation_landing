#!/usr/bin/env python
import time
import sys, os
import rospy

sys.path.append("/home/lyn/ros/chimera_ros_ws/src/cooperation_landing/scripts/simulation")
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

from sim_links_attachment import *
import tf.transformations as tft
from gazebo_msgs.msg import ModelState, ModelStates

class SimbasicNode:
    def __init__(self):
        self.pub_sim_pose = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

        self.loop_rate = rospy.Rate(100)
        self.ground_robot_ns = "go1_gazebo"
        self.links_attachment = AttachlinksNode()
    def sim_robot_pose(self, robot_name, x, y, z, ox, oy, oz, ow):
        sim_pose = ModelState()
        sim_pose.model_name = robot_name
        sim_pose.pose.position.x = x
        sim_pose.pose.position.y = y
        sim_pose.pose.position.z = z
        sim_pose.pose.orientation.x = ox
        sim_pose.pose.orientation.y = oy
        sim_pose.pose.orientation.z = oz
        sim_pose.pose.orientation.w = ow
        sim_pose.reference_frame = 'world'
        time.sleep(0.1)
        self.pub_sim_pose.publish(sim_pose)

    def sim_robot_twist(self, robot_name, lx, ly, lz, rx, ry, rz):
        sim_pose = ModelState()
        sim_pose.model_name = robot_name
        sim_pose.twist.linear.x = lx
        sim_pose.twist.linear.y = ly
        sim_pose.twist.linear.z = lz
        sim_pose.twist.angular.x = rx
        sim_pose.twist.angular.y = ry
        sim_pose.twist.angular.z = rz
        sim_pose.reference_frame = 'base'
        time.sleep(0.1)
        self.pub_sim_pose.publish(sim_pose)

    def _callback_apriltag(self, msg):
        self.msg_apriltag = msg

    def find_target_tag(self, data, target_id):
        T = 0
        # print(f'11111111')
        # if len(data.detections) == 0:
        #     return

        for det in data.detections:
            # print(f'{det}')
            # print(f'{det.id}')
            if target_id in det.id:
                pose = det.pose.pose.pose
                x = pose.position.x
                y = pose.position.y
                z = pose.position.z
                qx = pose.orientation.x
                qy = pose.orientation.y
                qz = pose.orientation.z
                qw = pose.orientation.w

                q = [qx, qy, qz, qw]
                t = [x, y, z]
                T = tft.quaternion_matrix(q)
                T[:3, 3] = t
                # print(f'{T}')
                return T
if __name__ == "__main__":
    rospy.init_node('attachlinks', anonymous=True)
    node = SimbasicNode()
    time.sleep(8)
    node.sim_robot_pose( 'xuanwu' , 0.3, 0, 0.4, 0 , 0, 1, 0)
    node.links_attachment.sim_attach_links("xuanwu", "root", "go1_gazebo", "base")

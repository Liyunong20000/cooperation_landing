"""Simulation: sim basic implementation."""

import time

import rospy
import tf.transformations as tft
from aerial_robot_model.srv import AddExtraModule
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Inertia, Transform

from cooperation_landing.simulation.sim_links_attachment import AttachlinksNode


class SimbasicNode:
    def __init__(self):
        set_model_state_topic = rospy.get_param(
            '~gazebo_set_model_state_topic', '/gazebo/set_model_state'
        )
        self.extra_module_service = rospy.get_param(
            '~extra_module_service', '/xuanwu/add_extra_module'
        )
        self.pub_sim_pose = rospy.Publisher(set_model_state_topic, ModelState, queue_size=10)
        self.loop_rate = rospy.Rate(100)
        self.ground_robot_ns = rospy.get_param('~sim_ground_model', 'go1_gazebo')
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
        if data is None:
            return None
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
        return None

    def call_add_extra_module(self, action, module_name, parent_link_name):
        try:
            self.extra_module = rospy.ServiceProxy(self.extra_module_service, AddExtraModule)

            transform = Transform()
            transform.translation.x = 0.0
            transform.translation.y = 0.0
            transform.translation.z = 0.1212
            transform.rotation.x = 0.0
            transform.rotation.y = 0.0
            transform.rotation.z = 0.0
            transform.rotation.w = 1.0

            inertia = Inertia()
            inertia.m = 0.0001
            inertia.com.x = 0.0
            inertia.com.y = 0.0
            inertia.com.z = 0.01556
            inertia.ixx = 0.00008009893
            inertia.ixy = 0.0
            inertia.ixz = 0.0
            inertia.iyy = 0.00019812289
            inertia.iyz = 0.0
            inertia.izz = 0.000205079

            response = self.extra_module(action, module_name, parent_link_name, transform, inertia)

            return response
        except rospy.ServiceException as e:
            rospy.logerr('AddExtraModule service call failed: %s', e)
            return None


def main():
    """Run the ROS node."""
    rospy.init_node('attachlinks', anonymous=True)
    node = SimbasicNode()
    time.sleep(8)
    node.sim_robot_pose('xuanwu', 0.3, 0, 0.4, 0, 0, 1, 0)
    node.links_attachment.sim_attach_links('xuanwu', 'root', 'go1_gazebo', 'base')

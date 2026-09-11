"""Simulation: sim links attachment implementation."""

import time

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest


class AttachlinksNode:
    def __init__(self):
        service_timeout_s = max(
            0.1, float(rospy.get_param('~link_attacher_service_timeout_s', 10.0))
        )
        attach_service = rospy.get_param(
            '~link_attacher_attach_service', '/link_attacher_node/attach'
        )
        detach_service = rospy.get_param(
            '~link_attacher_detach_service', '/link_attacher_node/detach'
        )
        rospy.loginfo('Creating ServiceProxy to %s', attach_service)
        self.attach_srv = rospy.ServiceProxy(attach_service, Attach)
        self.attach_srv.wait_for_service(timeout=service_timeout_s)
        rospy.loginfo('Created ServiceProxy to %s', attach_service)

        rospy.loginfo('Creating ServiceProxy to %s', detach_service)
        self.detach_srv = rospy.ServiceProxy(detach_service, Attach)
        self.detach_srv.wait_for_service(timeout=service_timeout_s)
        rospy.loginfo('Created ServiceProxy to %s', detach_service)

    def sim_attach_links(self, model1, link1, model2, link2):
        rospy.loginfo(f'Attaching {model1} and {model2}')
        req = AttachRequest()
        req.model_name_1 = model1
        req.link_name_1 = link1
        req.model_name_2 = model2
        req.link_name_2 = link2

        self.attach_srv.call(req)

    def sim_detach_links(self, model1, link1, model2, link2):
        rospy.loginfo(f'detaching {model1} and {model2}')
        req = AttachRequest()
        req.model_name_1 = model1
        req.link_name_1 = link1
        req.model_name_2 = model2
        req.link_name_2 = link2

        self.detach_srv.call(req)


def main():
    """Run the ROS node."""
    rospy.init_node('attachlinks', anonymous=True)
    node = AttachlinksNode()
    model1 = rospy.get_param('~sim_links_attacher/model1', 'go1_gazebo/base')
    link1 = rospy.get_param('~sim_links_attacher/link1', 'link')
    model2 = rospy.get_param('~sim_links_attacher/model2', 'xuanwu')
    link2 = rospy.get_param('~sim_links_attacher/link2', 'link')
    node.sim_attach_links(model1, link1, model2, link2)
    time.sleep(0.1)

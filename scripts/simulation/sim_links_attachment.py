#!/usr/bin/env python
import time
import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


class AttachlinksNode:
    def __init__(self):
        rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                        Attach)
        self.attach_srv.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

        rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                        Attach)
        self.detach_srv.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

    def sim_attach_links(self, model1, link1, model2, link2):
        rospy.loginfo(f"Attaching {model1} and {model2}")
        req = AttachRequest()
        req.model_name_1 = model1
        req.link_name_1 = link1
        req.model_name_2 = model2
        req.link_name_2 = link2

        self.attach_srv.call(req)

    def sim_detach_links(self, model1, link1, model2, link2):
        rospy.loginfo(f"detaching {model1} and {model2}")
        req = AttachRequest()
        req.model_name_1 = model1
        req.link_name_1 = link1
        req.model_name_2 = model2
        req.link_name_2 = link2

        self.detach_srv.call(req)
if __name__ == "__main__":
    rospy.init_node('attachlinks', anonymous=True)
    node = AttachlinksNode()
    model1 = rospy.get_param("~sim_links_attacher/model1", "go1_gazebo/base")
    link1 = rospy.get_param("~sim_links_attacher/link1", "link")
    model2 = rospy.get_param("~sim_links_attacher/model2", "xuanwu")
    link2 = rospy.get_param("~sim_links_attacher/link2", "link")
    node.sim_attach_links(model1,link1,model2,link2)
    time.sleep(0.1)
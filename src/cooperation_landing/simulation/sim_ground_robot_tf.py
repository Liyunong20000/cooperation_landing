"""Simulation: sim ground robot tf implementation."""

import time

import rospy
import tf
from gazebo_msgs.msg import ModelStates


class SimgroundrobotNode:
    def __init__(self):
        self.robot_name = rospy.get_param('~robot_name', 'go1_gazebo')
        self.child_frame = rospy.get_param('~child_frame', 'base')
        self.parent_frame = rospy.get_param('~parent_frame', 'world')
        model_states_topic = rospy.get_param('~model_states_topic', '/gazebo/model_states')

        self.tf_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber(model_states_topic, ModelStates, self._callback_ground_robot_pose)

    def _callback_ground_robot_pose(self, msg):
        if self.robot_name not in msg.name:
            rospy.logwarn_throttle(
                5.0, 'SimgroundrobotNode: model %s is not available in Gazebo.', self.robot_name
            )
            return
        index = msg.name.index(self.robot_name)
        if index >= len(msg.pose):
            rospy.logwarn_throttle(
                5.0, 'Gazebo model states contain no pose for %s.', self.robot_name
            )
            return
        pose = msg.pose[index]
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        ox = pose.orientation.x
        oy = pose.orientation.y
        oz = pose.orientation.z
        ow = pose.orientation.w
        self.tf_broadcaster.sendTransform(
            (x, y, z),
            (ox, oy, oz, ow),
            rospy.Time.now(),
            self.child_frame,
            self.parent_frame,
        )


def main():
    """Run the ROS node."""
    rospy.init_node('simgroundrobot', anonymous=True)
    _node = SimgroundrobotNode()
    time.sleep(0.1)
    rospy.spin()

#!/usr/bin/env python
import time

#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
import math
import tf
import tf.transformations as tft
from geometry_msgs.msg import Quaternion
import sys

from tf.transformations import quaternion_matrix


class SimgroundrobotNode:
    def __init__(self):
        self.robot_name = "go1_gazebo"

        self.tf_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber('/gazebo/model_states', ModelStates, self._callback_ground_robot_pose)

    def _callback_ground_robot_pose(self, msg):
        try:
            index = msg.name.index("go1_gazebo")  # Find index of the robot
            pose = msg.pose[index]
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            ox = pose.orientation.x
            oy = pose.orientation.y
            oz = pose.orientation.z
            ow = pose.orientation.w
            quaternion = pose.orientation
            quat = tft.euler_from_quaternion([ox, oy, oz, ow])
            self.tf_broadcaster.sendTransform(
                    (x, y, z),
                    (ox, oy, oz, ow),
                    rospy.Time.now(),
                    "/base",
                    "world"
            )
            rospy.loginfo("go1_gazebo position: x=%.2f, y=%.2f, z=%.2f",
                      x, y, z)

        except ValueError:
                rospy.logwarn("go1_gazebo not found in model_states.")

if __name__ == "__main__":
    rospy.init_node('simgroundrobot', anonymous=True)
    node = SimgroundrobotNode()
    time.sleep(0.1)
    while not rospy.is_shutdown():
        rospy.spin()

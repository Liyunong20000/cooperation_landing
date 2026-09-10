"""Basic function implementation."""

import rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped


class BasicNode:
    def __init__(self):

        # Subscribe and publish.

        self.mocap_brick_x, self.mocap_brick_y, self.mocap_brick_z = 0.0, 0.0, 0.0
        self.mocap_brick_qx, self.mocap_brick_qy, self.mocap_brick_qz, self.mocap_brick_qw = (
            0.0,
            0.0,
            0.0,
            1.0,
        )
        self.mocap_brick_roll, self.mocap_brick_pitch, self.mocap_brick_yaw = 0.0, 0.0, 0.0

        self.mocap_desk_x, self.mocap_desk_y, self.mocap_desk_z = 0.0, 0.0, 0.0
        self.mocap_desk_qx, self.mocap_desk_qy, self.mocap_desk_qz, self.mocap_desk_qw = (
            0.0,
            0.0,
            0.0,
            1.0,
        )
        self.mocap_desk_roll, self.mocap_desk_pitch, self.mocap_desk_yaw = 0.0, 0.0, 0.0

        self.mocap_tag12_x, self.mocap_tag12_y, self.mocap_tag12_z = 0.0, 0.0, 0.0
        self.mocap_tag12_qx, self.mocap_tag12_qy, self.mocap_tag12_qz, self.mocap_tag12_qw = (
            0.0,
            0.0,
            0.0,
            1.0,
        )
        self.mocap_tag12_roll, self.mocap_tag12_pitch, self.mocap_tag12_yaw = 0.0, 0.0, 0.0

        self.mocap_tag13_x, self.mocap_tag13_y, self.mocap_tag13_z = 0.0, 0.0, 0.0
        self.mocap_tag13_qx, self.mocap_tag13_qy, self.mocap_tag13_qz, self.mocap_tag13_qw = (
            0.0,
            0.0,
            0.0,
            1.0,
        )
        self.mocap_tag13_roll, self.mocap_tag13_pitch, self.mocap_tag13_yaw = 0.0, 0.0, 0.0

        self.mocap_xuanwu_x, self.mocap_xuanwu_y, self.mocap_xuanwu_z = 0.0, 0.0, 0.0
        self.mocap_xuanwu_qx, self.mocap_xuanwu_qy, self.mocap_xuanwu_qz, self.mocap_xuanwu_qw = (
            0.0,
            0.0,
            0.0,
            1.0,
        )
        self.mocap_xuanwu_roll, self.mocap_xuanwu_pitch, self.mocap_xuanwu_yaw = 0.0, 0.0, 0.0

        # Callbacks may run as soon as a subscription is registered.
        self._brick_mocap_sub = rospy.Subscriber(
            rospy.get_param('~brick_mocap_topic', '/qilin/mocap_node/brick/mocap/pose'),
            PoseStamped,
            self._callback_brick_mocap,
            queue_size=1,
        )
        self._desk_mocap_sub = rospy.Subscriber(
            rospy.get_param('~desk_mocap_topic', '/qilin/mocap_node/desk/mocap/pose'),
            PoseStamped,
            self._callback_desk_mocap,
            queue_size=1,
        )
        self._tag12_mocap_sub = rospy.Subscriber(
            rospy.get_param('~tag12_mocap_topic', '/qilin/mocap_node/tag12/mocap/pose'),
            PoseStamped,
            self._callback_tag12_mocap,
            queue_size=1,
        )
        self._tag13_mocap_sub = rospy.Subscriber(
            rospy.get_param('~tag13_mocap_topic', '/qilin/mocap_node/tag13/mocap/pose'),
            PoseStamped,
            self._callback_tag13_mocap,
            queue_size=1,
        )
        self._xuanwu_mocap_sub = rospy.Subscriber(
            rospy.get_param('~aerial_mocap_topic', '/xuanwu/mocap/pose'),
            PoseStamped,
            self._callback_xuanwu_mocap,
            queue_size=1,
        )

    def _generic_pose_callback(self, msg, prefix):
        """
        Generic callback for pose messages to update position and orientation attributes.
        """
        pose = msg.pose
        setattr(self, f'mocap_{prefix}_x', pose.position.x)
        setattr(self, f'mocap_{prefix}_y', pose.position.y)
        setattr(self, f'mocap_{prefix}_z', pose.position.z)
        setattr(self, f'mocap_{prefix}_qx', pose.orientation.x)
        setattr(self, f'mocap_{prefix}_qy', pose.orientation.y)
        setattr(self, f'mocap_{prefix}_qz', pose.orientation.z)
        setattr(self, f'mocap_{prefix}_qw', pose.orientation.w)
        roll, pitch, yaw = tft.euler_from_quaternion(
            [
                getattr(self, f'mocap_{prefix}_qx'),
                getattr(self, f'mocap_{prefix}_qy'),
                getattr(self, f'mocap_{prefix}_qz'),
                getattr(self, f'mocap_{prefix}_qw'),
            ]
        )
        setattr(self, f'mocap_{prefix}_roll', roll)
        setattr(self, f'mocap_{prefix}_pitch', pitch)
        setattr(self, f'mocap_{prefix}_yaw', yaw)

    def _callback_brick_mocap(self, msg):
        self._generic_pose_callback(msg, 'brick')

    def _callback_desk_mocap(self, msg):
        self._generic_pose_callback(msg, 'desk')

    def _callback_tag12_mocap(self, msg):
        self._generic_pose_callback(msg, 'tag12')

    def _callback_tag13_mocap(self, msg):
        self._generic_pose_callback(msg, 'tag13')

    def _callback_xuanwu_mocap(self, msg):
        self._generic_pose_callback(msg, 'xuanwu')


def main():
    """Run the ROS node."""
    rospy.init_node('Basic', anonymous=True)
    rospy.sleep(0.1)
    _node = BasicNode()
    rospy.spin()

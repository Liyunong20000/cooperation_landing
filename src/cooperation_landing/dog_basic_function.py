"""Dog basic function implementation."""

import rospy
import tf.transformations as tft
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose, Twist
from std_srvs.srv import Trigger


class DogBasic:
    """Ground robot command interface; initialize rospy before constructing it.

    ``~ground_robot_ns`` selects the absolute robot namespace (default: /go1).
    Commands use body axes x forward, y left, z up. Service methods return
    acceptance status; publishing a command does not confirm physical motion.
    """

    def __init__(self):

        # Subscribe and publish.
        self.robot_ns = ('/' + str(rospy.get_param('~ground_robot_ns', 'go1')).strip('/')).rstrip(
            '/'
        )
        tag_topic = rospy.get_param('~ground_tag_topic', self.robot_ns + '/tag_detections')

        self.pub_qilin_vel = rospy.Publisher(self.robot_ns + '/cmd_vel', Twist, queue_size=10)
        self.pub_qilin_pose = rospy.Publisher(self.robot_ns + '/body_pose', Pose, queue_size=10)
        #
        # rospy.wait_for_service('/go1/sit')
        # rospy.wait_for_service('/go1/stand')

        self.service_client_sit = rospy.ServiceProxy(self.robot_ns + '/sit', Trigger)
        self.service_client_stand = rospy.ServiceProxy(self.robot_ns + '/stand', Trigger)
        self.service_wait_timeout_s = max(
            0.0, float(rospy.get_param('~ground_service_wait_timeout_s', 2.0))
        )

        self.correction_tag_drone_x, self.correction_tag_drone_y, self.correction_tag_drone_z = (
            0.0,
            0.0,
            0.0,
        )
        (
            self.correction_tag_drone_roll,
            self.correction_tag_drone_pitch,
            self.correction_tag_drone_yaw,
        ) = 0.0, 0.0, 0.0

        self.correction_tag_13_x, self.correction_tag_13_y, self.correction_tag_13_z = 0.0, 0.0, 0.0
        self.correction_tag_13_roll, self.correction_tag_13_pitch, self.correction_tag_13_yaw = (
            0.0,
            0.0,
            0.0,
        )

        self.correction_tag_3_x, self.correction_tag_3_y, self.correction_tag_3_z = 0.0, 0.0, 0.0
        self.correction_tag_3_roll, self.correction_tag_3_pitch, self.correction_tag_3_yaw = (
            0.0,
            0.0,
            0.0,
        )

        self.correction_target_x, self.correction_target_y, self.correction_target_z = 0.0, 0.0, 0.0
        self.correction_target_roll, self.correction_target_pitch, self.correction_target_yaw = (
            0.0,
            0.0,
            0.0,
        )

        self.correction_err_x, self.correction_err_y, self.correction_err_z = 0.0, 0.0, 0.0
        self.correction_err_pitch, self.correction_err_roll, self.correction_err_yaw = 0.0, 0.0, 0.0

        # Callbacks may run as soon as a subscription is registered.
        self._tag_dog_info_sub = rospy.Subscriber(
            tag_topic, AprilTagDetectionArray, self._callback_tag_dog_info, queue_size=1
        )

    # Function for sit
    def sit(self):
        """Request sitting; return False if discovery or the service fails."""
        try:
            rospy.wait_for_service(self.robot_ns + '/sit', timeout=self.service_wait_timeout_s)
            response = self.service_client_sit()
            if response.success:
                rospy.loginfo('Sit command executed successfully')
            else:
                rospy.logwarn('Sit command failed: %s', response.message)
                return False
            return True
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logerr('Service call failed: %s', e)
            return False

    # Function for stand
    def stand(self):
        """Request standing; return False if discovery or the service fails."""
        try:
            rospy.wait_for_service(self.robot_ns + '/stand', timeout=self.service_wait_timeout_s)
            response = self.service_client_stand()
            if response.success:
                rospy.loginfo('Stand command executed successfully')
            else:
                rospy.logwarn('Stand command failed: %s', response.message)
                return False
            return True
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logerr('Service call failed: %s', e)
            return False

    # velocity command
    def qilin_cmd_vel(self, lx, ly, ax, ay, az):
        """Publish body velocity in m/s (lx, ly) and rad/s (ax, ay, az).

        The caller is responsible for command limits and stopping the robot.
        """
        qilin_cmd_vel = Twist()
        qilin_cmd_vel.linear.x = lx
        qilin_cmd_vel.linear.y = ly
        qilin_cmd_vel.angular.x = ax
        qilin_cmd_vel.angular.y = ay
        qilin_cmd_vel.angular.z = az

        self.pub_qilin_vel.publish(qilin_cmd_vel)

    # orientation command
    def qilin_body_pose(self, qx, qy, qz, qw):
        """Publish a body orientation as a normalized quaternion in xyzw order."""
        qilin_body_pose = Pose()
        qilin_body_pose.orientation.x = qx
        qilin_body_pose.orientation.y = qy
        qilin_body_pose.orientation.z = qz
        qilin_body_pose.orientation.w = qw
        rospy.sleep(0.1)
        self.pub_qilin_pose.publish(qilin_body_pose)

    # Get the target id and value the variable
    def tag_position_correction_tag_13(self, data, target_id):
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
                (
                    self.correction_tag_13_roll,
                    self.correction_tag_13_pitch,
                    self.correction_tag_13_yaw,
                ) = tft.euler_from_quaternion([qx, qy, qz, qw])

    def tag_position_correction_tag_3(self, data, target_id):
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
                (
                    self.correction_tag_3_roll,
                    self.correction_tag_3_pitch,
                    self.correction_tag_3_yaw,
                ) = tft.euler_from_quaternion([qx, qy, qz, qw])

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

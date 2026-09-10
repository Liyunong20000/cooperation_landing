"""Gripper: gripper move implementation."""

import rospy
from spinal.msg import ServoControlCmd, ServoStates
from std_msgs.msg import Empty


class GripperMoveNode:
    def __init__(self):
        rospy.logdebug('Initializing gripper controller.')
        self.servo_index = 0
        self.servo_angle = 0.0
        self.servo_temp = 0
        self.servo_load = 0.0
        self.servo_error = 0
        self.servo_state_received = False
        self.servo_target_index = 0
        self.servo_target_angles = 0
        self.robot_ns = ('/' + str(rospy.get_param('~robot_ns', 'xuanwu')).strip('/')).rstrip('/')
        self.servo_max_angles = rospy.get_param(self.robot_ns + '/servo_info/max_angles', 1400)
        self.servo_min_angles = rospy.get_param(self.robot_ns + '/servo_info/min_angles', -150)
        self.servo_max_load = rospy.get_param(self.robot_ns + '/servo_info/max_load', 350)
        # Subscribe and publish.

        self.pub_servo_target = rospy.Publisher(
            self.robot_ns + '/servo/target_states', ServoControlCmd, queue_size=10
        )
        self.pub_servo_target_qilin = rospy.Publisher(
            self.robot_ns + '/servo/target_states/info', ServoControlCmd, queue_size=10
        )
        self.pub_servo_target_qilin_trigger = rospy.Publisher(
            self.robot_ns + '/servo/target_states/trigger', Empty, queue_size=10
        )
        self.pub_servo_return_qilin_trigger = rospy.Publisher(
            self.robot_ns + '/servo/return/trigger', Empty, queue_size=10
        )

        # Callbacks may run as soon as a subscription is registered.
        self._servo_states_sub = rospy.Subscriber(
            self.robot_ns + '/servo/states', ServoStates, self._callback_servo_states, queue_size=1
        )

    def _callback_servo_states(self, msg):
        if not msg.servos:
            rospy.logwarn_throttle(5.0, 'Received an empty servo state message.')
            return
        self.servo_index = msg.servos[0].index
        self.servo_angle = msg.servos[0].angle
        self.servo_temp = msg.servos[0].temp
        self.servo_load = msg.servos[0].load
        self.servo_error = msg.servos[0].error
        self.servo_state_received = True
        # print(f'{self.servo_index, self.servo_angle, self.servo_temp, self.servo_load, self.servo_error}')

    def servo_target_cmd(self, target_index, target_angle):
        servo_target_cmd = ServoControlCmd()
        servo_target_cmd.index = [target_index]
        servo_target_cmd.angles = [target_angle]
        rospy.sleep(0.1)
        self.pub_servo_target.publish(servo_target_cmd)
        rospy.logdebug('Published local servo target: %s', servo_target_cmd)

    def servo_target_cmd_qilin(self, target_index, target_angle):
        servo_target_cmd = ServoControlCmd()
        servo_target_cmd.index = [target_index]
        servo_target_cmd.angles = [target_angle]

        self.pub_servo_target_qilin.publish(servo_target_cmd)
        rospy.logdebug('Published bridged servo target: %s', servo_target_cmd)
        rospy.sleep(0.2)
        self.grasp_qilin_trigger()

    def return_zero(self):
        rospy.sleep(0.1)
        self.servo_target_cmd(0, self.servo_max_angles)
        rospy.sleep(0.5)

    def return_zero_qilin(self):
        rospy.sleep(0.1)
        self.servo_target_cmd_qilin(0, self.servo_max_angles)
        rospy.sleep(0.5)

    def grasp(self, servo_index, angle_feed):
        if not self.servo_state_received:
            rospy.logerr('Cannot grasp before receiving a servo state.')
            return False
        rospy.sleep(0.02)
        r = rospy.Rate(3)
        if self.servo_error == 1:
            rospy.logerr('Servo %s reported an error; grasp aborted.', servo_index)
            return False
        deadline = rospy.Time.now() + rospy.Duration(rospy.get_param('~grasp_timeout_s', 10.0))
        try:
            while not rospy.is_shutdown() and self.servo_load > -(self.servo_max_load - 50):
                if rospy.Time.now() >= deadline:
                    rospy.logwarn('Servo %s grasp timed out.', servo_index)
                    return False
                self.servo_target_index = servo_index
                self.servo_target_angles = self.servo_angle - angle_feed
                self.servo_target_cmd(self.servo_target_index, self.servo_target_angles)
                r.sleep()
        except KeyboardInterrupt:
            pass

        rospy.loginfo('Servo %s reached the configured load threshold.', servo_index)
        return True

    def grasp_qilin(self, servo_index, angle_feed):
        if not self.servo_state_received:
            rospy.logerr('Cannot grasp before receiving a servo state.')
            return False
        rospy.sleep(0.02)
        r = rospy.Rate(3)
        deadline = rospy.Time.now() + rospy.Duration(rospy.get_param('~grasp_timeout_s', 10.0))
        try:
            while not rospy.is_shutdown() and self.servo_load > -(self.servo_max_load - 50):
                if rospy.Time.now() >= deadline:
                    rospy.logwarn('Servo %s grasp timed out.', servo_index)
                    return False
                self.servo_target_index = servo_index
                self.servo_target_angles = self.servo_angle - angle_feed
                self.servo_target_cmd_qilin(self.servo_target_index, self.servo_target_angles)
                r.sleep()
        except KeyboardInterrupt:
            pass

        rospy.loginfo('Servo %s reached the configured load threshold.', servo_index)
        return True

    def grasp_qilin_trigger(self):
        rospy.sleep(0.1)
        empty_msg = Empty()
        self.pub_servo_target_qilin_trigger.publish(empty_msg)

    def return_qilin_trigger(self):
        rospy.sleep(0.1)
        empty_msg = Empty()
        self.pub_servo_return_qilin_trigger.publish(empty_msg)


def main():
    """Run the ROS node."""
    rospy.init_node('gripper_move', anonymous=True)

    node = GripperMoveNode()
    if rospy.get_param('~run_demo', False):
        rospy.logwarn('Running the gripper demonstration because ~run_demo is true.')
        node.return_zero()
        node.grasp(0, 50)
    rospy.spin()

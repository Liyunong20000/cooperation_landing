"""High-level gripper requests and monitoring; UAV event_trigger owns actuator safety."""

import threading

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
        self.grasp_contact_load_threshold = rospy.get_param(
            '~grasp_contact_load_threshold', self.servo_max_load - 50
        )
        self._state_lock = threading.Lock()
        self._grasp_observation_count = 0
        # Both local clients and Qilin use the existing UAV command gate.
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
        # Monitoring only: never send a corrective target from remote feedback.
        with self._state_lock:
            self.servo_index = msg.servos[0].index
            self.servo_angle = msg.servos[0].angle
            self.servo_temp = msg.servos[0].temp
            self.servo_load = msg.servos[0].load
            self.servo_error = msg.servos[0].error
            self.servo_state_received = True
            # Task observation only; each new callback contributes at most one sample.
            if self.servo_error == 0 and abs(self.servo_load) >= self.grasp_contact_load_threshold:
                self._grasp_observation_count = min(2, self._grasp_observation_count + 1)
            else:
                self._grasp_observation_count = 0

    def _clamp_target(self, target_angle):
        return int(max(self.servo_min_angles, min(self.servo_max_angles, target_angle)))

    def servo_target_cmd(self, target_index, target_angle):
        """Request a target through event_trigger, including local keyboard/demo use."""
        return self.servo_target_cmd_qilin(target_index, target_angle)

    def servo_target_cmd_qilin(self, target_index, target_angle):
        servo_target_cmd = ServoControlCmd()
        servo_target_cmd.index = [target_index]
        # Xuanwu's event bridge applies the feedback guard to this forwarded target.
        servo_target_cmd.angles = [self._clamp_target(target_angle)]

        self.pub_servo_target_qilin.publish(servo_target_cmd)
        rospy.logdebug('Published bridged servo target: %s', servo_target_cmd)
        rospy.sleep(0.2)
        self.grasp_qilin_trigger()

    def return_zero(self):
        self.return_qilin_trigger()
        rospy.sleep(0.5)

    def return_zero_qilin(self):
        return self.return_zero()

    def grasp(self, servo_index, angle_feed):
        """Observe task success; the UAV protects contact even if this feedback is delayed."""
        if angle_feed <= 0:
            rospy.logerr('Grasp angle_feed must be positive.')
            return False
        r = rospy.Rate(3)
        deadline = rospy.Time.now() + rospy.Duration(rospy.get_param('~grasp_timeout_s', 10.0))
        try:
            while not rospy.is_shutdown():
                with self._state_lock:
                    received = self.servo_state_received
                    angle, error = self.servo_angle, self.servo_error
                    contact_observed = self._grasp_observation_count >= 2
                if not received:
                    rospy.logerr('Cannot grasp before receiving a servo state.')
                    return False
                if error != 0:
                    rospy.logerr('Servo %s error 0x%02x; grasp task aborted.', servo_index, error)
                    return False
                if contact_observed:
                    rospy.loginfo('Servo %s grasp contact observed in task feedback.', servo_index)
                    return True
                if angle <= self.servo_min_angles:
                    rospy.logwarn('Servo %s minimum-angle grasp abort at %s.', servo_index, angle)
                    return False
                if rospy.Time.now() >= deadline:
                    rospy.logwarn('Servo %s grasp task timed out.', servo_index)
                    return False
                self.servo_target_index = servo_index
                self.servo_target_angles = self._clamp_target(angle - angle_feed)
                self.servo_target_cmd(servo_index, self.servo_target_angles)
                r.sleep()
        except (KeyboardInterrupt, rospy.ROSInterruptException):
            rospy.logwarn('Servo %s grasp task interrupted.', servo_index)
        # Stopping this task never disables the UAV's independent local safety gate.
        return False

    def grasp_qilin(self, servo_index, angle_feed):
        return self.grasp(servo_index, angle_feed)

    def grasp_qilin_trigger(self):
        rospy.sleep(0.1)
        empty_msg = Empty()
        self.pub_servo_target_qilin_trigger.publish(empty_msg)

    def return_qilin_trigger(self):
        with self._state_lock:
            self._grasp_observation_count = 0
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

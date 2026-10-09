"""Check UAV2GR message arrivals on the ground robot's local ROS master."""

import math
import threading
import time

import rospy
from roslib.message import get_message_class


def load_uav_topics():
    """Derive the local topic inventory from the two UAV2GR message definitions."""
    topics = []
    for name in ('cooperation_landing/UAV2GR_1', 'cooperation_landing/UAV2GR_2'):
        message = get_message_class(name)
        if message is None:
            raise ValueError('Cannot load bridge message ' + name)
        for field, msg_type in zip(message.__slots__, message._slot_types):
            data_class = get_message_class(msg_type)
            if data_class is None:
                raise ValueError('Cannot load topic message ' + msg_type)
            topics.append(('/' + field.replace('__', '/'), data_class))
    return topics


class BridgePreflight:
    """Require one local message per UAV2GR topic without querying the UAV master."""

    def __init__(self, timeout_default=20.0):
        self.topics = load_uav_topics()
        self.window = float(rospy.get_param('~bridge_check_timeout_s', timeout_default))
        self.interval = float(rospy.get_param('~bridge_check_interval_s', 1.0))
        if not all(math.isfinite(value) and value > 0 for value in
                   (self.window, self.interval)):
            raise ValueError('Bridge check timeouts must be finite and positive')
        self._lock = threading.Lock()
        self.arrivals = {}
        self.probes = {}

    def _callback(self, _message, topic):
        with self._lock:
            self.arrivals[topic] += 1

    def _status(self, topic):
        with self._lock:
            received = self.arrivals[topic] > 0
        if received:
            return 'PASS: at least one message received'
        if self.probes[topic].get_num_connections() == 0:
            return 'WAIT: no local publisher connection'
        return 'WAIT: no message received yet'

    def check(self):
        rospy.loginfo('[Communication preflight] Checking %d local UAV2GR topics.', len(self.topics))
        with self._lock:
            self.arrivals = {topic: 0 for topic, _ in self.topics}
        self.probes = {}
        deadline = time.monotonic() + self.window
        previous = {}
        try:
            for topic, data_class in self.topics:
                self.probes[topic] = rospy.Subscriber(
                    topic, data_class, self._callback, callback_args=topic, queue_size=1)
            while not rospy.is_shutdown():
                now = time.monotonic()
                report = {topic: self._status(topic) for topic, _ in self.topics}
                for topic, status in report.items():
                    if previous.get(topic) != status:
                        rospy.loginfo('[Communication preflight] %s -> %s', topic, status)
                previous = report
                if report and all(status.startswith('PASS:') for status in report.values()):
                    rospy.loginfo('[Communication preflight] Received at least one message on every UAV2GR topic.')
                    return True
                if now >= deadline:
                    for topic, status in report.items():
                        if not status.startswith('PASS:'):
                            rospy.logerr('[Communication preflight] %s -> %s', topic, status)
                    return False
                time.sleep(self.interval)
            return False
        finally:
            for probe in self.probes.values():
                probe.unregister()

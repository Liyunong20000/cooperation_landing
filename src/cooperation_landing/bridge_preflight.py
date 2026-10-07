"""Read-only checks of both ROS masters and Silverhammer topic traffic."""

import http.client
import time
import xml.etree.ElementTree as ET
import xmlrpc.client
from pathlib import Path

import rospkg
import rospy
from roslib.message import get_message_class


class TimeoutTransport(xmlrpc.client.Transport):
    def __init__(self, timeout):
        super().__init__()
        self.timeout = timeout

    def make_connection(self, host):
        return http.client.HTTPConnection(host, timeout=self.timeout)


def rpc(uri, method, *args, timeout=1.0):
    with xmlrpc.client.ServerProxy(uri, transport=TimeoutTransport(timeout)) as proxy:
        code, message, value = getattr(proxy, method)(rospy.get_name(), *args)
    if code != 1:
        raise RuntimeError(message)
    return value


def node_name(namespace, name):
    return '/' + '/'.join(part for part in (namespace.strip('/'), name.strip('/')) if part)


def load_channels(ground_ns='', uav_ns=''):
    """Use the launch files and generated fields as the complete topic inventory."""
    root = Path(rospkg.RosPack().get_path('cooperation_landing')) / 'launch'

    def nodes(filename):
        result = []
        for node in ET.parse(root / filename).getroot().findall('node'):
            params = {p.get('name'): p.get('value') for p in node.findall('param')}
            result.append((node.get('name'), node.get('type'), params))
        return result

    ground, uav = nodes('GR2UAV.launch'), nodes('UAV2GR.launch')
    channels = []
    for name, kind, params in ground:
        if 'highspeed' not in kind:
            continue
        outgoing = 'streamer' in kind
        port = params['to_port' if outgoing else 'receive_port']
        peer = next(node for node in uav if node[2].get(
            'receive_port' if outgoing else 'to_port') == port)
        sender = node_name(ground_ns, name) if outgoing else node_name(uav_ns, peer[0])
        receiver = node_name(uav_ns, peer[0]) if outgoing else node_name(ground_ns, name)
        message = get_message_class(params['message'])
        if message is None:
            raise ValueError('Cannot load bridge message ' + params['message'])
        topics = [('/' + field.replace('__', '/'), msg_type)
                  for field, msg_type in zip(message.__slots__, message._slot_types)]
        channels.append(dict(sender=sender, receiver=receiver, outgoing=outgoing,
                             topics=topics, message=params['message']))
    return channels


class BridgePreflight:
    """Check traffic without publishing flight, servo, or landing commands."""

    def __init__(self):
        ground_ns = rospy.get_param('~bridge_ground_ns', '')
        self.channels = load_channels(ground_ns, rospy.get_param('~bridge_uav_ns', ''))
        remote = rospy.get_param('~uav_master_uri', '')
        if not remote:
            ip = rospy.get_param(node_name(ground_ns, 'highspeed_streamer_1') + '/to_ip', '')
            if not ip:
                raise ValueError('Set ~uav_master_uri or start GR2UAV.launch first')
            remote = 'http://%s:11311/' % ip
        self.masters = (rospy.get_master().getUri()[2], remote)
        if self.masters[0].rstrip('/') == remote.rstrip('/'):
            raise ValueError('Ground and UAV bridge checks require separate ROS masters')
        self.timeout = float(rospy.get_param('~bridge_rpc_timeout_s', 1.0))
        self.window = float(rospy.get_param('~bridge_check_timeout_s', 20.0))
        self.interval = float(rospy.get_param('~bridge_check_interval_s', 1.0))
        if not all(0 < value < float('inf') for value in
                   (self.timeout, self.window, self.interval)):
            raise ValueError('Bridge check timeouts must be finite and positive')
        self.probes = []
        self.baseline = {}

    def _snapshot(self, uri):
        def call(method, *args):
            return rpc(uri, method, *args, timeout=self.timeout)
        publishers, subscribers, _ = call('getSystemState')
        return dict(publishers=dict(publishers), subscribers=dict(subscribers),
                    types=dict(call('getTopicTypes')), call=call, nodes={})

    def _node(self, master, name):
        if name not in master['nodes']:
            uri = master['call']('lookupNode', name)
            master['nodes'][name] = dict(
                bus=rpc(uri, 'getBusInfo', timeout=self.timeout),
                stats=rpc(uri, 'getBusStats', timeout=self.timeout))
        return master['nodes'][name]

    @staticmethod
    def _connected(node, topic, direction):
        return any(row[2] == direction and row[4] == topic and row[5]
                   for row in node['bus'])

    def _check_topic(self, channel, topic, msg_type, masters):
        source, dest = masters if channel['outgoing'] else masters[::-1]
        sender, receiver = channel['sender'], channel['receiver']
        if sender not in source['subscribers'].get(topic, []):
            return 'FAIL: 发送端缺少该话题订阅'
        if receiver not in dest['publishers'].get(topic, []):
            return 'FAIL: 接收端缺少该话题发布者'
        if source['types'].get(topic) != msg_type or dest['types'].get(topic) != msg_type:
            return 'FAIL: 两端消息类型不匹配'
        send_node, recv_node = self._node(source, sender), self._node(dest, receiver)
        send_port = source['call']('getParam', sender + '/to_port')
        recv_port = dest['call']('getParam', receiver + '/receive_port')
        if int(send_port) != int(recv_port):
            return 'FAIL: 两端 UDP 端口不匹配'
        # Command payloads are idle until requested; visual state is a latched source.
        command = topic.endswith(('/target_pose/info', '/servo/target_states/info'))
        if command and not self._connected(recv_node, topic, 'o'):
            return 'FAIL: 指令接收端未连接机载消费节点'
        if not command and (not source['publishers'].get(topic)
                            or not self._connected(send_node, topic, 'i')):
            return 'FAIL: 源数据发布者未连接发送端'
        sent = sum(conn[1] for row in send_node['stats'][1] if row[0] == topic
                   for conn in row[1] if conn[-1])
        received = sum(row[1] for row in recv_node['stats'][0] if row[0] == topic)
        key = (sender, receiver, topic)
        before = self.baseline.get(key, (sent, received))
        self.baseline[key] = (sent, received)
        if sent < before[0] or received < before[1]:
            return 'WAIT: 桥接计数器重置，需要重新观察数据'
        latched = topic.endswith('/visual_landing/state')
        if received <= before[1] or (not command and sent <= before[0] and not (latched and sent > 0)):
            return 'WAIT: 尚未观察到源数据/接收端发布更新'
        if command:
            return 'READY: 接收端持续发布；源端指令数据按需发送'
        return 'PASS: 源数据和接收端发布均在更新'

    def check(self):
        self.baseline.clear()
        rospy.loginfo('[通信预检] Ground master=%s, UAV master=%s', *self.masters)
        # Ensure every received field has a subscriber, including unused telemetry.
        self.probes = [rospy.Subscriber(topic, rospy.AnyMsg, lambda msg: None, queue_size=1)
                       for channel in self.channels if not channel['outgoing']
                       for topic, _ in channel['topics']]
        deadline = time.monotonic() + self.window
        previous = {}
        try:
            while not rospy.is_shutdown():
                report = {}
                try:
                    masters = tuple(self._snapshot(uri) for uri in self.masters)
                    for channel in self.channels:
                        for topic, msg_type in channel['topics']:
                            key = ('GR2UAV' if channel['outgoing'] else 'UAV2GR') + ' ' + topic
                            try:
                                report[key] = self._check_topic(channel, topic, msg_type, masters)
                            except (OSError, xmlrpc.client.Error, RuntimeError) as error:
                                report[key] = 'FAIL: ' + str(error)
                except (OSError, xmlrpc.client.Error, RuntimeError) as error:
                    report['ROS master'] = 'FAIL: ' + str(error)
                for topic, status in report.items():
                    if previous.get(topic) != status:
                        rospy.loginfo('[通信预检] %s -> %s', topic, status)
                previous = report
                if report and all(status.startswith(('PASS:', 'READY:')) for status in report.values()):
                    rospy.loginfo('[通信预检] 高速通道检查通过；事件话题不在检查范围内。')
                    return True
                if time.monotonic() >= deadline:
                    for topic, status in report.items():
                        if not status.startswith(('PASS:', 'READY:')):
                            rospy.logerr('[通信预检] %s -> %s', topic, status)
                    return False
                time.sleep(self.interval)
            return False
        finally:
            for probe in self.probes:
                probe.unregister()

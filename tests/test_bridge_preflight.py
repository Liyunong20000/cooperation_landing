"""Exercise both sides of the real bridge inventory with simulated ROS bus stats."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from cooperation_landing import bridge_preflight as bridge


@pytest.fixture
def env(monkeypatch):
    checker = bridge.BridgePreflight.__new__(bridge.BridgePreflight)
    checker.channels = bridge.load_channels()
    checker.masters = ('http://ground:11311/', 'http://uav:11311/')
    checker.timeout, checker.window, checker.interval = 0.1, 0.2, 0.05
    checker.baseline, checker.probes = {}, []
    clock = SimpleNamespace(now=100.0)
    snapshots = [dict(publishers={}, subscribers={}, types={}, nodes={}, ports={}) for _ in range(2)]

    def setup_node(master, name, port):
        master['ports'][name] = port
        master['nodes'].setdefault(name, dict(bus=[], stats=[[], [], []]))
        return master['nodes'][name]

    for port, channel in enumerate(checker.channels, start=8192):
        source, dest = snapshots if channel['outgoing'] else snapshots[::-1]
        sender = setup_node(source, channel['sender'], port)
        receiver = setup_node(dest, channel['receiver'], port)
        for topic, msg_type in channel['topics']:
            source['publishers'][topic] = ['/sensor_source']
            source['subscribers'][topic] = [channel['sender']]
            dest['publishers'][topic] = [channel['receiver']]
            dest['subscribers'][topic] = ['/consumer']
            source['types'][topic] = dest['types'][topic] = msg_type
            sender['bus'].append([1, '/sensor_source', 'i', 'TCPROS', topic, True])
            receiver['bus'].append([2, '/consumer', 'o', 'TCPROS', topic, True])
            sender['stats'][1].append([topic, [[1, 100, 1, -1, True]]])
            receiver['stats'][0].append([topic, 100, [[2, 100, 1, True]]])

    for master in snapshots:
        master['call'] = lambda method, param, m=master: m['ports'][param.rsplit('/', 1)[0]]
    checker._snapshot = lambda uri: snapshots[checker.masters.index(uri)]
    subscribers = []

    def subscriber(*args, **kwargs):
        sub = Mock()
        subscribers.append(sub)
        return sub

    def advance(dt):
        clock.now += dt
        for master in snapshots:
            for node in master['nodes'].values():
                for row in node['stats'][0]:
                    row[1] += 100
                for topic, connections in node['stats'][1]:
                    if not topic.endswith('/visual_landing/state'):
                        connections[0][1] += 100

    monkeypatch.setattr(bridge.time, 'monotonic', lambda: clock.now)
    monkeypatch.setattr(bridge.time, 'sleep', advance)
    monkeypatch.setattr(bridge.rospy, 'Subscriber', subscriber)
    for name in ('loginfo', 'logerr'):
        monkeypatch.setattr(bridge.rospy, name, Mock())
    monkeypatch.setattr(bridge.rospy, 'is_shutdown', lambda: False)
    return SimpleNamespace(checker=checker, snapshots=snapshots, clock=clock, advance=advance,
                           subscribers=subscribers)


def test_all_nine_high_speed_fields_and_no_events():
    channels = bridge.load_channels()
    assert len(channels) == 4
    assert sum(len(channel['topics']) for channel in channels) == 9
    topics = [topic for channel in channels for topic, _ in channel['topics']]
    assert '/qilin/tag_detections' in topics
    assert '/xuanwu/uav/cog/odom' in topics
    assert not any(topic.endswith(('/trigger', '/cancel', '/takeoff', '/land', '/start'))
                   for topic in topics)


def test_two_way_data_flow_passes_and_probes_are_removed(env):
    assert env.checker.check()
    assert len(env.subscribers) == 6
    for sub in env.subscribers:
        sub.unregister.assert_called_once()


def test_registered_publishers_without_packets_do_not_pass(env, monkeypatch):
    monkeypatch.setattr(bridge.time, 'sleep', lambda dt: setattr(env.clock, 'now', env.clock.now + dt))
    assert not env.checker.check()


def test_missing_remote_publisher_fails_before_flight(env):
    env.snapshots[1]['publishers'].pop('/qilin/tag_detections')
    assert not env.checker.check()


def test_mismatched_port_fails_even_when_topic_names_match(env):
    channel = env.checker.channels[1]
    env.snapshots[1]['ports'][channel['receiver']] += 1
    assert not env.checker.check()


def test_message_type_mismatch_fails(env):
    env.snapshots[0]['types']['/xuanwu/uav/cog/odom'] = 'geometry_msgs/PoseStamped'
    assert not env.checker.check()


def test_command_payload_requires_receiver_traffic_but_can_be_idle(env):
    channel = env.checker.channels[1]
    topic, msg_type = channel['topics'][0]
    for row in env.snapshots[0]['nodes'][channel['sender']]['stats'][1]:
        row[1][0][1] = 0
    assert env.checker._check_topic(channel, topic, msg_type, env.snapshots).startswith('WAIT:')
    env.advance(0.05)
    assert env.checker._check_topic(channel, topic, msg_type, env.snapshots).startswith('READY:')
    # Earlier positive evidence cannot keep passing after the link stops.
    assert env.checker._check_topic(channel, topic, msg_type, env.snapshots).startswith('WAIT:')


def test_uav_command_consumer_must_be_connected(env):
    channel = env.checker.channels[1]
    env.snapshots[1]['nodes'][channel['receiver']]['bus'].clear()
    assert not env.checker.check()


def test_unreachable_master_reports_failure_and_cleans_probes(env):
    def unavailable(uri):
        raise OSError('UAV master unreachable')

    env.checker._snapshot = unavailable
    assert not env.checker.check()
    assert all(sub.unregister.call_count == 1 for sub in env.subscribers)

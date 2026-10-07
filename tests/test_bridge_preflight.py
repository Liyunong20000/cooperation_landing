"""Check local UAV2GR subscriptions with simulated message arrivals."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from cooperation_landing import bridge_preflight as bridge


@pytest.fixture
def env(monkeypatch):
    params = {'~bridge_check_timeout_s': 0.3, '~bridge_check_interval_s': 0.05}
    clock = SimpleNamespace(now=100.0)
    subscriptions = {}
    blocked = set()
    disconnected = set()

    def subscribe(topic, data_class, callback, callback_args, **kwargs):
        sub = Mock(get_num_connections=lambda: int(topic not in disconnected))
        subscriptions[topic] = SimpleNamespace(sub=sub, data_class=data_class,
                                               callback=callback, callback_args=callback_args)
        return sub

    def emit(topic):
        entry = subscriptions[topic]
        entry.callback(entry.data_class(), entry.callback_args)

    def advance(dt):
        clock.now += dt
        for topic in subscriptions:
            if topic not in blocked and topic not in disconnected:
                emit(topic)

    monkeypatch.setattr(bridge.rospy, 'get_param', lambda key, default: params.get(key, default))
    monkeypatch.setattr(bridge.rospy, 'Subscriber', subscribe)
    monkeypatch.setattr(bridge.rospy, 'get_master', Mock(side_effect=AssertionError('No explicit master query expected')))
    monkeypatch.setattr(bridge.rospy, 'Publisher', Mock(side_effect=AssertionError('No test command expected')))
    for name in ('loginfo', 'logerr'):
        monkeypatch.setattr(bridge.rospy, name, Mock())
    monkeypatch.setattr(bridge.rospy, 'is_shutdown', lambda: False)
    monkeypatch.setattr(bridge.time, 'monotonic', lambda: clock.now)
    monkeypatch.setattr(bridge.time, 'sleep', advance)
    checker = bridge.BridgePreflight()
    return SimpleNamespace(checker=checker, clock=clock, subscriptions=subscriptions, params=params,
                           blocked=blocked, disconnected=disconnected, emit=emit, advance=advance,
                           subscribe=subscribe)


def test_inventory_contains_only_the_six_uav2gr_fields():
    topics = dict(bridge.load_uav_topics())
    assert {topic: data_class._type for topic, data_class in topics.items()} == {
        '/xuanwu/uav/cog/odom': 'nav_msgs/Odometry',
        '/xuanwu/flight_state': 'std_msgs/UInt8',
        '/xuanwu/servo/states': 'spinal/ServoStates',
        '/xuanwu/mocap/pose': 'geometry_msgs/PoseStamped',
        '/xuanwu/visual_landing/state': 'std_msgs/UInt8',
        '/xuanwu/tag_detections': 'apriltag_ros/AprilTagDetectionArray',
    }


def test_local_arrivals_pass_without_master_queries_or_command_publication(env):
    assert env.checker.check()
    assert len(env.subscriptions) == 6
    for entry in env.subscriptions.values():
        entry.sub.unregister.assert_called_once()
    bridge.rospy.get_master.assert_not_called()
    bridge.rospy.Publisher.assert_not_called()


def test_connected_publishers_without_messages_do_not_pass(env):
    env.blocked.update(topic for topic, _ in env.checker.topics)
    assert not env.checker.check()


def test_missing_topic_blocks_preflight_and_names_the_topic(env):
    topic = '/xuanwu/mocap/pose'
    env.disconnected.add(topic)
    assert not env.checker.check()
    assert any(topic in call.args for call in bridge.rospy.logerr.call_args_list)


def test_single_latched_message_is_sufficient(env, monkeypatch):
    def once(dt):
        env.advance(dt)
        env.blocked.update(env.subscriptions)

    monkeypatch.setattr(bridge.time, 'sleep', once)
    assert env.checker.check()
    assert all(count == 1 for count in env.checker.arrivals.values())


def test_empty_tag_detections_still_count_as_publication(env):
    assert env.checker.check()  # Default AprilTagDetectionArray has no detections.
    assert env.checker.arrivals['/xuanwu/tag_detections'] == 1


def test_received_message_does_not_expire(env):
    assert env.checker.check()
    env.clock.now += 1.0
    assert env.checker._status('/xuanwu/flight_state').startswith('PASS:')


def test_each_check_requires_new_message_arrivals(env):
    assert env.checker.check()
    env.blocked.update(env.subscriptions)
    assert not env.checker.check()
    assert all(count == 0 for count in env.checker.arrivals.values())


def test_disconnection_after_receipt_does_not_revoke_success(env):
    assert env.checker.check()
    topic = '/xuanwu/servo/states'
    env.disconnected.add(topic)
    assert env.checker._status(topic).startswith('PASS:')


def test_shutdown_cleans_all_subscriptions(env, monkeypatch):
    monkeypatch.setattr(bridge.rospy, 'is_shutdown', lambda: env.clock.now >= 100.05)
    assert not env.checker.check()
    assert all(entry.sub.unregister.call_count == 1 for entry in env.subscriptions.values())


def test_subscription_failure_cleans_already_created_subscriptions(env, monkeypatch):
    def fail_third(*args, **kwargs):
        if len(env.subscriptions) == 2:
            raise RuntimeError('Subscription failed')
        return env.subscribe(*args, **kwargs)

    monkeypatch.setattr(bridge.rospy, 'Subscriber', fail_third)
    with pytest.raises(RuntimeError, match='Subscription failed'):
        env.checker.check()
    assert all(entry.sub.unregister.call_count == 1 for entry in env.subscriptions.values())


@pytest.mark.parametrize('value', [0, -1, float('nan'), float('inf')])
def test_invalid_check_timeout_is_rejected(env, value):
    env.params['~bridge_check_timeout_s'] = value
    with pytest.raises(ValueError):
        bridge.BridgePreflight()

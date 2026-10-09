"""Offline dock regressions; ROS I/O and elapsed time are mocked."""

import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray

from cooperation_landing import drone_basic_function as drone_module
from cooperation_landing import manipulation_states as states
from cooperation_landing.control_utils import p_with_deadzone


@pytest.mark.parametrize('tol,gain,minimum', [(0.03, 0.2, 0.025), (0.03, -0.3, 0.025),
                                           (0.05, -0.5, 0.1)])
@pytest.mark.parametrize('sign', [-1, 1])
def test_strict_deadzone(tol, gain, minimum, sign):
    assert p_with_deadzone(sign * tol * 0.99, gain, minimum, 0.6, tol) == 0
    assert abs(p_with_deadzone(sign * tol, gain, minimum, 0.6, tol)) == minimum


@pytest.fixture
def env(monkeypatch):
    clock = SimpleNamespace(now=100.0, sitting=False, post_sent=False, post='valid')
    fake_time = SimpleNamespace(monotonic=lambda: clock.now)
    monkeypatch.setattr(states, 'time', fake_time)
    monkeypatch.setattr(drone_module, 'time', fake_time)
    monkeypatch.setattr(states.rospy.Time, 'now', lambda: states.rospy.Time(clock.now))
    monkeypatch.setattr(states.rospy, 'get_param', lambda name, default: default)
    monkeypatch.setattr(states.rospy, 'is_shutdown', lambda: False)
    for name in ('logwarn', 'logdebug', 'logwarn_throttle'):
        monkeypatch.setattr(states.rospy, name, Mock())

    drone = drone_module.DroneBasic.__new__(drone_module.DroneBasic)
    drone._tag_info_lock = threading.Lock()
    drone.tag_info = None
    drone._tag_receive_time = None
    drone._tag_message_counter = 0

    def receive(z=0.48, marker=0):
        det = AprilTagDetection()
        det.id = [marker]
        det.pose.pose.pose.position.x = 0.01
        det.pose.pose.pose.position.z = z
        det.pose.pose.pose.orientation.w = 1.0
        drone._callback_tag_info(AprilTagDetectionArray(detections=[det]))

    def advance(duration):
        clock.now += duration

    def tick():
        advance(0.1)
        marker = 0 if userdata.object_state == 0 else 2
        target = 0.48 if userdata.object_state == 0 else 0.47
        if clock.sitting:
            if clock.post != 'missing' and not clock.post_sent:
                receive(target if clock.post == 'valid' else 0.60, marker)
                clock.post_sent = True
        else:
            receive(target, marker)

    dog = Mock()
    dog.sit.side_effect = lambda: setattr(clock, 'sitting', True)
    monkeypatch.setattr(states, 'DogBasic', lambda: dog)
    monkeypatch.setattr(states, 'DroneBasic', lambda: drone)
    rate_factory = Mock(return_value=SimpleNamespace(sleep=tick))
    monkeypatch.setattr(states.rospy, 'Rate', rate_factory)
    monkeypatch.setattr(states.rospy, 'sleep', advance)
    node = states.DockApproach()
    userdata = SimpleNamespace(object_state=0, picking_marker_far=0, picking_marker_near=1,
                               placing_marker_far=2, placing_marker_near=3)
    return SimpleNamespace(node=node, drone=drone, dog=dog, clock=clock,
                           receive=receive, userdata=userdata, rate_factory=rate_factory)


def test_cached_frame_expires_without_refreshing_arrival(env):
    env.receive()
    assert env.node._detect_fresh_marker(0, 1)[0]
    arrival = env.drone.tag_snapshot()[1]
    env.clock.now += 0.5
    assert not env.node._detect_fresh_marker(0, 1)[0]
    assert env.drone.tag_snapshot()[1] == arrival


def test_direct_velocity_at_ten_hz(env):
    env.receive(z=0.88)
    assert env.node.execute(env.userdata) == 'succeeded'
    env.rate_factory.assert_called_once_with(10.0)
    assert env.dog.qilin_cmd_vel.call_args_list[0].args == pytest.approx((0.08, 0, 0, 0, 0))
    env.dog.stand.assert_not_called()


@pytest.mark.parametrize('post,expected', [('valid', 'succeeded'), ('outside', 'retry'),
                                         ('missing', 'retry')])
@pytest.mark.parametrize('mode', [0, 1])
def test_sit_requires_new_in_range_frame(env, post, expected, mode):
    env.userdata.object_state = mode
    env.receive(0.48 if mode == 0 else 0.47, 0 if mode == 0 else 2)
    env.clock.post = post
    assert env.node._alignment_attempt(env.userdata, states.rospy.Time.now()) == expected
    env.dog.sit.assert_called_once()
    assert all(call.args == (0, 0, 0, 0, 0) for call in env.dog.qilin_cmd_vel.call_args_list)
    assert env.dog.stand.call_count == (expected == 'retry')


@pytest.mark.parametrize('post', ['outside', 'missing'])
@pytest.mark.parametrize('mode', [0, 1])
def test_failed_recheck_stands_and_realigns_before_success(env, post, mode, monkeypatch):
    env.userdata.object_state = mode
    marker = 0 if mode == 0 else 2
    target = 0.48 if mode == 0 else 0.47
    env.receive(target, marker)
    env.clock.post = post

    def stand():
        env.clock.sitting = False
        env.clock.post_sent = False
        env.clock.post = 'valid'

    def sleep(duration):
        env.clock.now += duration
        if duration == 2.0 and not env.clock.sitting:
            # Fresh telemetry after standing settles forces actual correction.
            env.receive(target + 0.10, marker)

    env.dog.stand.side_effect = stand
    monkeypatch.setattr(states.rospy, 'sleep', sleep)
    assert env.node.execute(env.userdata) == 'succeeded'
    assert env.dog.sit.call_count == 2
    env.dog.stand.assert_called_once()
    assert any(call.args[0] > 0 for call in env.dog.qilin_cmd_vel.call_args_list)
    assert env.clock.now - 100.0 < env.node.timeout_s


def test_repeated_recheck_failure_uses_original_timeout(env):
    env.receive()
    env.clock.post = 'outside'
    env.node.timeout_s = 10.0

    def stand():
        env.clock.sitting = False
        env.clock.post_sent = False

    env.dog.stand.side_effect = stand
    assert env.node.execute(env.userdata) == 'failed'
    assert env.dog.sit.call_count >= 2
    assert env.dog.stand.call_count == env.dog.sit.call_count
    assert env.clock.now - 100.0 < 15.0
    assert env.dog.qilin_cmd_vel.call_args.args == (0, 0, 0, 0, 0)


def test_stale_observation_stops_without_extending_hold(env, monkeypatch):
    env.receive(z=0.88)

    def tick():
        env.clock.now += 0.1

    env.rate_factory.return_value.sleep = tick
    env.node.timeout_s = 0.8
    assert env.node.execute(env.userdata) == 'failed'
    commands = [call.args for call in env.dog.qilin_cmd_vel.call_args_list]
    assert commands[0][0] > 0
    assert commands[-2:] == [(0, 0, 0, 0, 0)] * 2
    env.dog.sit.assert_not_called()

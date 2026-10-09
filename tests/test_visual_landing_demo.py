"""Offline flight orchestration checks; no ROS master or hardware commands."""

from pathlib import Path
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml
from nav_msgs.msg import Odometry

from cooperation_landing import visual_landing_demo as demo

RealDemoDrone = demo.DemoDrone

@pytest.fixture
def env(monkeypatch):
    params = yaml.safe_load((Path(__file__).resolve().parents[1] /
                             'config/LandingPoints.yaml').read_text())
    params = {'~' + key: value for key, value in params.items()}
    clock = SimpleNamespace(now=100.0)
    ros_clock = SimpleNamespace(now=1000.0)
    rate = Mock()

    def rate_sleep():
        clock.now += 0.1
        ros_clock.now += 0.1

    rate.sleep.side_effect = rate_sleep
    monkeypatch.setattr(demo.rospy, 'Rate', Mock(return_value=rate))
    monkeypatch.setattr(demo.rospy.Time, 'now',
                        lambda: demo.rospy.Time.from_sec(ros_clock.now))
    drone = Mock(robot_ns='/xuanwu', odom_arrival=100.0, flight_arrival=100.0,
                 drone_state=demo.ARM_OFF, drone_x=0.2, drone_y=0.3, drone_z=0.5,
                 drone_yaw=0.1, takeoff_x=0.2, takeoff_y=0.3, takeoff_z=0.5)
    monkeypatch.setattr(demo.rospy, 'get_param', lambda key, default: params.get(key, default))
    monkeypatch.setattr(demo.rospy, 'is_shutdown', lambda: False)
    monkeypatch.setattr(demo.rospy, 'on_shutdown', Mock())
    monkeypatch.setattr(demo.rospy, 'Subscriber', Mock())
    monkeypatch.setattr(demo.rospy, 'Publisher', lambda *args, **kwargs:
                        Mock(get_num_connections=lambda: 1))
    monkeypatch.setattr(demo.rospy, 'sleep', Mock())
    for name in ('loginfo', 'logwarn', 'logerr'):
        monkeypatch.setattr(demo.rospy, name, Mock())
    monkeypatch.setattr(demo, 'DemoDrone', lambda: drone)
    monkeypatch.setattr(demo, 'DogBasic', Mock())
    monkeypatch.setattr(demo, 'BridgePreflight', Mock(return_value=Mock(check=lambda: True)))
    monkeypatch.setattr(demo.time, 'monotonic', lambda: clock.now)
    monkeypatch.setattr(demo.time, 'sleep', lambda dt: setattr(clock, 'now', clock.now + dt))
    node = demo.VisuallandqilinNode()
    node.visual_state = demo.IDLE
    node._hold = Mock(return_value=True)
    return SimpleNamespace(node=node, drone=drone, clock=clock, ros_clock=ros_clock,
                           rate=rate, params=params)


def test_all_ten_points_share_config(env):
    points = env.params['~landing_points']
    assert len(points) == 10
    for index, point in enumerate(points):
        xyz, quat = demo.select_point(points, index)
        assert xyz == (point['x'], point['y'], point['z'])
        assert sum(value ** 2 for value in quat) == pytest.approx(1.0)


@pytest.mark.parametrize('index', [-1, 10, 'p1', True, 1.5])
def test_bad_index_cannot_select_another_point(env, index):
    with pytest.raises(ValueError):
        demo.select_point(env.params['~landing_points'], index)


def test_route_matches_original_demo(env):
    node, drone = env.node, env.drone
    drone.drone_takeoff.side_effect = lambda: setattr(drone, 'drone_state', demo.HOVER)
    node._send_target_and_wait = Mock(return_value=True)
    assert node.demo()
    assert [call.args for call in node._send_target_and_wait.call_args_list] == [
        (node.point_xyz, node.point_quat),
        ((0.2, 0.3, 1.0), (0, 0, 0, 1)),
        ((0.2, 0.3, 0.8), (0, 0, 0, 1)),
    ]
    drone.drone_start.assert_not_called()
    drone.drone_takeoff.assert_called_once()
    drone.drone_land.assert_not_called()


def test_failed_waypoint_stops_route_before_visual_trigger(env):
    env.drone.drone_state = demo.ARM_ON
    env.drone.drone_takeoff.side_effect = lambda: setattr(env.drone, 'drone_state', demo.HOVER)
    env.node._send_target_and_wait = Mock(return_value=False)
    assert not env.node.demo()
    env.node._send_target_and_wait.assert_called_once()
    env.node.trigger.publish.assert_not_called()


def test_cached_odom_cannot_finish_target(env):
    env.drone.drone_state = demo.HOVER
    env.drone.odom_arrival = 98.0
    env.params['~demo_target_timeout'] = 0.1
    env.params['~demo_target_retries'] = 0
    assert not env.node._send_target_and_wait((0.2, 0.3, 0.5), (0, 0, 0, 1))


def test_single_trigger_yields_nav_and_waits_for_motor_shutdown(env):
    node, drone = env.node, env.drone
    drone.drone_state = demo.HOVER
    node.trigger.publish.side_effect = lambda msg: setattr(node, 'visual_state', demo.DESCENDING)
    stages = iter([demo.LAND, demo.ARM_OFF])

    def advance(dt):
        env.clock.now += dt
        drone.flight_arrival = env.clock.now
        drone.drone_state = next(stages)

    demo.time.sleep = advance
    assert node.run()
    node.trigger.publish.assert_called_once()
    node.cancel_session()
    node.cancel.publish.assert_not_called()
    drone.drone_target.assert_not_called()
    drone.drone_nav.assert_not_called()
    drone.drone_land.assert_not_called()


def test_unacknowledged_trigger_times_out_and_cancels_once(env):
    env.drone.drone_state = demo.HOVER
    env.params['~trigger_ack_timeout_s'] = 0.1
    assert not env.node.run()
    env.node.cancel_session()
    env.node.cancel_session()
    env.node.trigger.publish.assert_called_once()
    env.node.cancel.publish.assert_called_once()


@pytest.mark.parametrize('state', [demo.IDLE, demo.ALIGNED, demo.ABORTED])
def test_alignment_or_abort_is_not_landing_success(env, state):
    env.node.visual_state = state
    with pytest.raises(RuntimeError):
        env.node._landing_result()


def test_stale_arm_off_does_not_finish_landing(env):
    env.node.landing_seen = True
    env.node.visual_state = demo.BRAKING
    env.drone.flight_arrival = 98.0
    assert not env.node._landing_result()


def feedback_clock(env, monkeypatch, update=None):
    """Simulate unique live telemetry while advancing the wall clock."""
    env.drone.drone_state = demo.HOVER

    def advance(dt):
        env.clock.now += dt
        env.drone.odom_arrival = env.drone.flight_arrival = env.clock.now
        if update:
            update()

    monkeypatch.setattr(demo.time, 'sleep', advance)
    env.params.update({'~target_no_progress_timeout_s': 0.2,
                       '~demo_target_timeout': 2.0,
                       '~target_progress_distance_m': 0.01})


def test_failed_communication_never_arms(env):
    env.node.communication.check = lambda: False
    assert not env.node.demo()
    env.drone.drone_start.assert_not_called()
    env.drone.drone_takeoff.assert_not_called()


@pytest.mark.parametrize('state', [demo.ARM_OFF, demo.ARM_ON, 3])
def test_missing_hover_times_out_without_repeating_takeoff(env, state):
    env.params['~takeoff_state_timeout_s'] = 0.1
    env.node._send_target_and_wait = Mock()
    env.drone.drone_takeoff.side_effect = lambda: setattr(env.drone, 'drone_state', state)

    def advance():
        env.ros_clock.now += 0.1

    env.rate.sleep.side_effect = advance
    assert not env.node.demo()
    assert env.clock.now == 100.0
    env.rate.sleep.assert_called_once()
    env.drone.drone_start.assert_not_called()
    env.drone.drone_takeoff.assert_called_once()
    env.node._send_target_and_wait.assert_not_called()
    env.node.trigger.publish.assert_not_called()
    demo.rospy.logerr.assert_called_once_with(
        'Timed out after %.1f s waiting for flight state 5.', 0.1)


def test_takeoff_waits_for_hover_without_arm_on_feedback(env):
    states = iter([3, demo.HOVER])

    def advance():
        env.clock.now += 0.1
        env.ros_clock.now += 0.1
        env.drone.flight_arrival = env.drone.odom_arrival = env.clock.now
        env.drone.drone_state = next(states)
        env.drone.drone_takeoff.assert_called_once()
        env.node._send_target_and_wait.assert_not_called()

    env.rate.sleep.side_effect = advance
    env.node._send_target_and_wait = Mock(return_value=True)
    assert env.node.demo()
    env.drone.drone_start.assert_not_called()
    env.drone.drone_takeoff.assert_called_once()
    assert env.clock.now == pytest.approx(100.2)
    demo.rospy.Rate.assert_called_once_with(10)
    demo.rospy.loginfo.assert_any_call(
        '[Waiting] %s, timeout %.1f s.',
        '/xuanwu/flight_state = 5 (HOVER) after takeoff', 30.0)


def test_takeoff_accepts_hover_without_recent_flight_callback(env):
    def takeoff():
        env.drone.drone_state = demo.HOVER
        env.drone.flight_arrival = 98.0

    env.drone.drone_takeoff.side_effect = takeoff
    env.node._send_target_and_wait = Mock(return_value=True)
    assert env.node.demo()
    env.drone.drone_takeoff.assert_called_once()
    env.rate.sleep.assert_not_called()
    assert env.node._send_target_and_wait.call_count == 3


def test_shutdown_during_takeoff_stops_route(env, monkeypatch):
    shutdown = SimpleNamespace(value=False)
    monkeypatch.setattr(demo.rospy, 'is_shutdown', lambda: shutdown.value)
    env.rate.sleep.side_effect = lambda: setattr(shutdown, 'value', True)
    env.node._send_target_and_wait = Mock()
    assert not env.node.demo()
    env.drone.drone_takeoff.assert_called_once()
    env.node._hold.assert_not_called()
    env.node._send_target_and_wait.assert_not_called()
    env.node.trigger.publish.assert_not_called()


def test_lost_first_command_resends_identical_pose_and_trigger(env, monkeypatch):
    def update():
        if env.drone.drone_target.call_count == 2:
            env.drone.drone_x = 1.0
            env.drone.drone_yaw = 0.0

    feedback_clock(env, monkeypatch, update)
    xyz, quat = (1.0, 0.3, 0.5), (0, 0, 0, 1)
    assert env.node._send_target_and_wait(xyz, quat)
    assert [call.args for call in env.drone.drone_target.call_args_list] == [
        ('world', *xyz, *quat), ('world', *xyz, *quat)]
    env.drone.drone_nav.assert_not_called()


def test_stationary_drone_exhausts_retry_budget(env, monkeypatch):
    feedback_clock(env, monkeypatch)
    assert not env.node._send_target_and_wait((1.0, 0.3, 0.5), (0, 0, 0, 1))
    assert env.drone.drone_target.call_count == 4
    assert env.clock.now - 100.0 < 1.0


def test_continuous_progress_does_not_resend(env, monkeypatch):
    def update():
        env.drone.drone_x = min(1.0, env.drone.drone_x + 0.03)
        env.drone.drone_yaw = 0.0

    feedback_clock(env, monkeypatch, update)
    assert env.node._send_target_and_wait((1.0, 0.3, 0.5), (0, 0, 0, 1))
    env.drone.drone_target.assert_called_once()


def test_yaw_progress_alone_avoids_resending(env, monkeypatch):
    env.drone.drone_yaw = 0.5

    def update():
        env.drone.drone_yaw = max(0.0, env.drone.drone_yaw - 0.03)

    feedback_clock(env, monkeypatch, update)
    assert env.node._send_target_and_wait((0.2, 0.3, 0.5), (0, 0, 0, 1))
    env.drone.drone_target.assert_called_once()


def test_state_change_stops_waypoint_retries(env, monkeypatch):
    def update():
        env.drone.drone_state = demo.LAND

    feedback_clock(env, monkeypatch, update)
    assert not env.node._send_target_and_wait((1.0, 0.3, 0.5), (0, 0, 0, 1))
    env.drone.drone_target.assert_called_once()


def test_visual_session_cannot_send_waypoint(env):
    env.drone.drone_state = demo.HOVER
    env.node.session_started = True
    assert not env.node._send_target_and_wait((1.0, 0.3, 0.5), (0, 0, 0, 1))
    env.drone.drone_target.assert_not_called()


def test_duplicate_source_odom_cannot_refresh_feedback(env):
    drone = RealDemoDrone.__new__(RealDemoDrone)
    drone._odom_lock = threading.Lock()
    drone._odom_source_stamp = None
    drone._odom_message_counter = 0
    drone.odom_arrival = drone.odom_stamp = None
    msg = Odometry()
    msg.header.frame_id = '/world'
    msg.header.stamp = demo.rospy.Time(12)
    msg.pose.pose.orientation.w = 1.0
    RealDemoDrone._callback_drone_position(drone, msg)
    assert drone.odom_arrival == 100.0
    env.clock.now = 105.0
    RealDemoDrone._callback_drone_position(drone, msg)
    assert drone.odom_arrival == 100.0
    msg.header.stamp = demo.rospy.Time(13)
    RealDemoDrone._callback_drone_position(drone, msg)
    assert drone.odom_arrival == 105.0

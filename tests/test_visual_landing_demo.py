"""Offline flight orchestration checks; no ROS master or hardware commands."""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml

from cooperation_landing import visual_landing_demo as demo


@pytest.fixture
def env(monkeypatch):
    params = yaml.safe_load((Path(__file__).resolve().parents[1] /
                             'config/LandingPoints.yaml').read_text())
    params = {'~' + key: value for key, value in params.items()}
    clock = SimpleNamespace(now=100.0)
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
    for name in ('loginfo', 'logerr'):
        monkeypatch.setattr(demo.rospy, name, Mock())
    monkeypatch.setattr(demo, 'DemoDrone', lambda: drone)
    monkeypatch.setattr(demo, 'DogBasic', Mock())
    monkeypatch.setattr(demo.time, 'monotonic', lambda: clock.now)
    monkeypatch.setattr(demo.time, 'sleep', lambda dt: setattr(clock, 'now', clock.now + dt))
    node = demo.VisuallandqilinNode()
    node.visual_state = demo.IDLE
    return SimpleNamespace(node=node, drone=drone, clock=clock, params=params)


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
    drone.drone_start.assert_called_once()
    drone.drone_takeoff.assert_called_once()
    drone.drone_land.assert_not_called()


def test_failed_waypoint_stops_route_before_visual_trigger(env):
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

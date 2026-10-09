"""COG waypoint and flight-state regressions with ROS I/O mocked."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from nav_msgs.msg import Odometry

from cooperation_landing import drone_basic_function as drone_module
from cooperation_landing import manipulation_states as states


@pytest.fixture
def env(monkeypatch):
    clock = SimpleNamespace(now=100.0)
    params = {'~allow_payload_transfer': True, '~waypoint_timeout_s': 2.0}
    monkeypatch.setattr(drone_module, 'time', SimpleNamespace(monotonic=lambda: clock.now))
    monkeypatch.setattr(drone_module.rospy, 'get_param', lambda key, default: params.get(key, default))
    monkeypatch.setattr(drone_module.rospy, 'Subscriber', Mock())
    monkeypatch.setattr(drone_module.rospy, 'Publisher', Mock())
    monkeypatch.setattr(drone_module.rospy, 'ServiceProxy', Mock())
    monkeypatch.setattr(drone_module.rospy, 'is_shutdown', lambda: False)
    monkeypatch.setattr(drone_module.rospy, 'sleep', Mock())
    for name in ('loginfo', 'logwarn', 'logerr', 'loginfo_throttle'):
        monkeypatch.setattr(drone_module.rospy, name, Mock())
    drone = drone_module.DroneBasic()
    drone.drone_target = Mock()

    def receive(x=0.0, y=0.0, z=0.0, yaw=0.0, stamp=None, frame='/world'):
        msg = Odometry()
        msg.header.frame_id = frame
        msg.header.stamp = drone_module.rospy.Time.from_sec(clock.now if stamp is None else stamp)
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = x, y, z
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        drone._callback_drone_position(msg)
        return msg

    hooks = SimpleNamespace(tick=lambda: None)

    def sleep():
        clock.now += 0.1
        hooks.tick()

    rate = Mock()
    rate.sleep.side_effect = sleep
    monkeypatch.setattr(drone_module.rospy, 'Rate', Mock(return_value=rate))
    monkeypatch.setattr(states, 'DogBasic', Mock())
    monkeypatch.setattr(states, 'DroneBasic', lambda: drone)
    monkeypatch.setattr(states, 'GripperMoveNode', Mock())
    return SimpleNamespace(drone=drone, receive=receive, clock=clock, params=params,
                           hooks=hooks, rate=rate)


def test_uses_cog_topic(env):
    assert env.drone.cog_odom_topic == '/xuanwu/uav/cog/odom'
    assert drone_module.rospy.Subscriber.call_args_list[0].args[0] == env.drone.cog_odom_topic


def test_cannot_send_target_without_fresh_odom(env):
    assert not env.drone.move_to_target(0, 0, 1, 0, 0, 0, 1)
    env.drone.drone_target.assert_not_called()


def test_waits_for_new_feedback_after_target_trigger(env):
    env.receive(z=1.0)
    env.hooks.tick = lambda: env.receive(z=1.0)
    assert env.drone.move_to_target(0, 0, 1, 0, 0, 0, 1)
    assert env.rate.sleep.call_count == 1
    env.drone.drone_target.assert_called_once_with('world', 0, 0, 1, 0, 0, 0, 1)


def test_cached_in_range_pose_cannot_complete_waypoint(env):
    message = env.receive(z=1.0)
    env.hooks.tick = lambda: env.drone._callback_drone_position(message)
    assert not env.drone.move_to_target(0, 0, 1, 0, 0, 0, 1)
    assert env.drone.odom_snapshot()[2] == 1
    assert env.clock.now < 102.0  # Expired source feedback, before waypoint timeout.


@pytest.mark.parametrize('stamp', [0.0, 99.0, 100.0])
def test_replayed_source_stamp_does_not_refresh_arrival(env, stamp):
    env.receive()
    env.clock.now += 0.5
    env.receive(stamp=stamp)
    assert env.drone.odom_snapshot()[1:] == (100.0, 1)


@pytest.mark.parametrize('frame', ['map', 'camera'])
def test_wrong_frame_invalidates_feedback(env, frame):
    env.receive()
    env.clock.now += 0.1
    env.receive(frame=frame)
    assert not env.drone.move_to_target(0, 0, 1, 0, 0, 0, 1)
    env.drone.drone_target.assert_not_called()


def test_nonfinite_feedback_invalidates_waypoint(env):
    env.receive(x=float('nan'))
    assert not env.drone.move_to_target(0, 0, 1, 0, 0, 0, 1)


def test_zero_quaternion_is_invalid_feedback(env):
    message = env.receive()
    message.pose.pose.orientation.w = 0
    message.header.stamp = drone_module.rospy.Time(101)
    env.drone._callback_drone_position(message)
    assert env.drone.odom_snapshot()[1] is None


def test_position_arrival_still_waits_for_yaw(env):
    env.receive()
    updates = iter([0.5, 0.0])
    env.hooks.tick = lambda: env.receive(z=1.0, yaw=next(updates))
    assert env.drone.move_to_target(0, 0, 1, 0, 0, 0, 1)
    assert env.rate.sleep.call_count == 2


def test_yaw_wraps_across_pi(env):
    env.receive()
    env.hooks.tick = lambda: env.receive(z=1, yaw=-math.pi + 0.01)
    yaw = math.pi - 0.01
    assert env.drone.move_to_target(0, 0, 1, 0, 0, math.sin(yaw / 2), math.cos(yaw / 2))


def test_fresh_but_distant_feedback_times_out(env):
    env.drone.waypoint_retries = 0
    env.receive()
    env.hooks.tick = lambda: env.receive()
    assert not env.drone.move_to_target(0, 0, 1, 0, 0, 0, 1)
    assert env.clock.now >= 102.0


def test_lost_navigation_command_resends_same_pose_then_arrives(env):
    env.drone.waypoint_no_progress_timeout_s = 0.3
    env.receive()

    def tick():
        env.receive(z=1.0 if env.drone.drone_target.call_count >= 2 else 0.0)

    env.hooks.tick = tick
    assert env.drone.move_to_target(0, 0, 1, 0, 0, 0, 1)
    commands = env.drone.drone_target.call_args_list
    assert len(commands) == 2
    assert commands[0] == commands[1]
    assert env.clock.now < 102.0


def test_stationary_drone_exhausts_retry_budget(env):
    env.drone.waypoint_no_progress_timeout_s = 0.3
    env.drone.waypoint_retries = 2
    env.receive()
    env.hooks.tick = lambda: env.receive()
    assert not env.drone.move_to_target(0, 0, 1, 0, 0, 0, 1)
    commands = env.drone.drone_target.call_args_list
    assert len(commands) == 3
    assert all(command == commands[0] for command in commands)


def test_continuous_position_progress_does_not_resend(env):
    env.drone.waypoint_no_progress_timeout_s = 0.4
    env.receive()
    env.hooks.tick = lambda: env.receive(z=(env.clock.now - 100.0) * 0.6)
    assert env.drone.move_to_target(0, 0, 1, 0, 0, 0, 1)
    env.drone.drone_target.assert_called_once()


def test_yaw_progress_at_target_position_does_not_resend(env):
    env.drone.waypoint_no_progress_timeout_s = 0.4
    env.receive(z=1, yaw=0.8)
    env.hooks.tick = lambda: env.receive(z=1, yaw=max(0, 0.8 - (env.clock.now - 100.0) * 0.5))
    assert env.drone.move_to_target(0, 0, 1, 0, 0, 0, 1)
    env.drone.drone_target.assert_called_once()


def test_attempt_timeout_also_resends_current_waypoint(env):
    env.drone.waypoint_timeout_s = 0.3
    env.drone.waypoint_no_progress_timeout_s = 3.0
    env.receive()
    env.hooks.tick = lambda: env.receive(z=1 if env.drone.drone_target.call_count >= 2 else 0)
    assert env.drone.move_to_target(0, 0, 1, 0, 0, 0, 1)
    assert env.drone.drone_target.call_count == 2


@pytest.mark.parametrize('flight_state', ['target', 'back'])
def test_next_waypoint_is_sent_only_after_previous_arrival(env, flight_state):
    env.drone.waypoint_no_progress_timeout_s = 0.3
    env.receive()
    previous = SimpleNamespace(target=None, sends=0, unique=0)

    def publish(frame, x, y, z, qx, qy, qz, qw):
        target = (x, y, z)
        if previous.target is not None and target != previous.target:
            pose, _, _ = env.drone.odom_snapshot()
            assert math.dist(pose[:3], previous.target) < env.drone.waypoint_position_tol
        if previous.target != target:
            previous.unique += 1
        previous.target = target
        previous.sends += 1

    def tick():
        # Lose the initial command; recovery must repeat it before advancing.
        if previous.sends < 2:
            env.receive()
        else:
            env.receive(*previous.target)

    env.drone.drone_target.side_effect = publish
    env.hooks.tick = tick
    if flight_state == 'target':
        node = states.FlyTarget()
        userdata = SimpleNamespace(object_state=0, picking_position=[1, 2, 3, 0, 0, 0, 1])
    else:
        node = states.FlyBack()
        userdata = SimpleNamespace(takeoff_position=[1, 2, 0.5, 0])
    assert node.execute(userdata) == 'succeeded'
    assert previous.unique == 2
    assert previous.sends == 3  # First waypoint, its retry, then the next waypoint.


def test_retry_exhaustion_never_sends_next_waypoint_or_grips(env):
    env.drone.waypoint_no_progress_timeout_s = 0.3
    env.drone.waypoint_retries = 1
    env.receive()
    env.hooks.tick = lambda: env.receive()
    node = states.FlyTarget()
    userdata = SimpleNamespace(object_state=0, picking_position=[1, 2, 3, 0, 0, 0, 1])
    assert node.execute(userdata) == 'failed'
    commands = env.drone.drone_target.call_args_list
    assert len(commands) == 2
    assert commands[0] == commands[1]
    node.gripper_move.servo_target_cmd_qilin.assert_not_called()
    assert userdata.object_state == 0


@pytest.mark.parametrize('values', [(0, 0, float('nan'), 0, 0, 0, 1),
                                  (0, 0, 1, 0, 0, 0, 0)])
def test_invalid_target_is_not_sent(env, values):
    env.receive()
    assert not env.drone.move_to_target(*values)
    env.drone.drone_target.assert_not_called()


@pytest.mark.parametrize('mode', [0, 1])
@pytest.mark.parametrize('results,expected,calls', [([False], 'failed', 1),
                                                  ([True, False], 'failed', 2),
                                                  ([True, True], 'succeeded', 2)])
def test_fly_target_checks_both_waypoints_before_gripper(env, mode, results, expected, calls):
    env.drone.move_to_target = Mock(side_effect=results)
    node = states.FlyTarget()
    target = [1, 2, 3, 0, 0, 0, 1]
    userdata = SimpleNamespace(object_state=mode, picking_position=target, placing_position=target)
    assert node.execute(userdata) == expected
    assert env.drone.move_to_target.call_count == calls
    assert env.drone.move_to_target.call_args_list[0].args == (1, 2, 3.5, 0, 0, 0, 1)
    if expected == 'failed':
        node.gripper_move.servo_target_cmd_qilin.assert_not_called()
        assert userdata.object_state == mode
    else:
        assert node.gripper_move.servo_target_cmd_qilin.call_count == 2
        assert userdata.object_state == 1 - mode


@pytest.mark.parametrize('results,expected,calls', [([False], 'failed', 1),
                                                  ([True, False], 'failed', 2),
                                                  ([True, True], 'succeeded', 2)])
def test_fly_back_checks_hover_and_final_waypoints(env, results, expected, calls):
    env.drone.move_to_target = Mock(side_effect=results)
    node = states.FlyBack()
    assert 'failed' in node.get_registered_outcomes()
    assert node.execute(SimpleNamespace(takeoff_position=[1, 2, 0.5, 0])) == expected
    assert env.drone.move_to_target.call_count == calls
    assert env.drone.move_to_target.call_args_list[0].args == (1, 2, 1, 0, 0, 0, 1)
    if calls == 2:
        assert env.drone.move_to_target.call_args_list[1].args == (1, 2, 0.8, 0, 0, 0, 1)


@pytest.mark.parametrize('direction', [-1, 1])
def test_horizontal_scan_rotates_at_fixed_speed_and_stops_at_target(env, monkeypatch, direction):
    node = states.HorizontalScan()
    env.receive(yaw=0)
    monkeypatch.setattr(states.rospy.Time, 'now', lambda: states.rospy.Time.from_sec(env.clock.now))

    def tick():
        env.clock.now += 0.05
        yaw_speed = node.dog_basic.qilin_cmd_vel.call_args.args[4]
        env.receive(yaw=env.drone.drone_yaw + yaw_speed * 0.05)

    env.rate.sleep.side_effect = tick
    target = direction * node.delta_yaw
    assert node._rotate_to(target)
    commands = [call.args for call in node.dog_basic.qilin_cmd_vel.call_args_list]
    assert all(command[4] == direction * 0.25 for command in commands[:-1])
    assert commands[-1] == (0, 0, 0, 0, 0)
    assert abs(target - env.drone.drone_yaw) < node.yaw_tol
    assert 4.0 < env.clock.now - 100.0 < 6.0


def test_horizontal_scan_timeout_stops_stationary_robot(env, monkeypatch):
    node = states.HorizontalScan()
    env.receive(yaw=0)
    monkeypatch.setattr(states.rospy.Time, 'now', lambda: states.rospy.Time.from_sec(env.clock.now))
    env.rate.sleep.side_effect = lambda: setattr(env.clock, 'now', env.clock.now + 0.05)
    assert not node._rotate_to(node.delta_yaw)
    assert env.clock.now - 100.0 > 6.0
    assert node.dog_basic.qilin_cmd_vel.call_args.args == (0, 0, 0, 0, 0)

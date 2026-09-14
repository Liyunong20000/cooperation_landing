"""Offline controller regressions using real ROS messages and mocked ROS I/O.

With the ROS workspace sourced, run:
    PYTHONPATH="src:$PYTHONPATH" python3 -m pytest -q tests/test_apriltag_alignment.py
No ROS master, subscribers, publishers, services or hardware are started.
"""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import yaml
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray

from cooperation_landing import apriltag_alignment as alignment


@pytest.fixture
def env(monkeypatch):
    clock = SimpleNamespace(wall=100.0, ros=100.0)
    params = {
        '/camera_drone_matrix': np.eye(4).ravel().tolist(),
        '/drone_tags_matrix': [
            {'id': tag_id, 'matrix': np.eye(4).ravel().tolist()} for tag_id in (0, 1)
        ],
    }
    publishers = {}

    def publisher(topic, msg_type, **kwargs):
        publishers[topic.rsplit('/', 1)[-1]] = Mock(msg_type=msg_type)
        assert topic.startswith('/go1/alignment_debug/')
        return publishers[topic.rsplit('/', 1)[-1]]

    monkeypatch.setattr(alignment, 'time', SimpleNamespace(monotonic=lambda: clock.wall))
    monkeypatch.setattr(alignment.rospy.Time, 'now', lambda: alignment.rospy.Time(clock.ros))
    monkeypatch.setattr(alignment.rospy, 'get_param', lambda name, default: params.get(name, default))
    monkeypatch.setattr(alignment.rospy, 'Publisher', publisher)
    monkeypatch.setattr(alignment.rospy, 'Subscriber', Mock())
    monkeypatch.setattr(alignment, 'DogBasic', Mock())
    for name in ('logdebug', 'logwarn', 'logwarn_throttle'):
        monkeypatch.setattr(alignment.rospy, name, Mock())
    return SimpleNamespace(clock=clock, params=params, publishers=publishers)


def detection(tag_id, x=0.0, y=0.0, z=1.0, yaw=0.0):
    det = AprilTagDetection()
    det.id = [tag_id]
    pose = det.pose.pose.pose
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    q = alignment.tft.quaternion_from_euler(0, 0, yaw)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return det


def cycle(node, *detections):
    node._callback_apriltag(AprilTagDetectionArray(detections=list(detections)))
    node.align_dog_with_drone()


def command(node):
    return node.dog_basic_function.qilin_cmd_vel.call_args.args


def debug(env, name):
    return env.publishers[name].publish.call_args.args[0]


def test_defaults_and_private_parameter_precedence(env):
    node = alignment.AprilmoveqilinNode()
    assert (node.min_linear_vel, node.align_enter_distance, node.align_exit_distance,
            node.max_pair_gap) == (0.03, 0.025, 0.035, 0.06)
    assert (node.apriltag_message_timeout, node.tag_loss_timeout) == (0.5, 0.5)
    assert node.command_hold_timeout == 0.35
    assert (node.yaw_enter_threshold, node.yaw_exit_threshold) == (0.03, 0.05)
    assert (node.move_param, node.rotate_param) == (1.10, 0.3)
    assert (node.last_valid_lx, node.last_valid_ly, node.last_valid_ryaw) == (0.0, 0.0, 0.0)
    assert not node.xy_aligned
    assert not node.yaw_aligned
    assert node.last_valid_detection_time is None
    assert node.previous_drone_center_position is None
    env.params.update({'/move_parameter': 1.5, '~move_parameter': 0.2})
    assert alignment.AprilmoveqilinNode().move_param == 0.2


@pytest.mark.parametrize('params,error', [
    ({'~yaw_enter_threshold': -0.01}, 'yaw_enter_threshold'),
    ({'~yaw_enter_threshold': 0.05}, 'yaw_enter_threshold'),
    ({'~yaw_exit_threshold': 0.02}, 'yaw_enter_threshold'),
    ({'~command_hold_timeout': -0.01}, 'command_hold_timeout'),
    ({'~command_hold_timeout': 0.5}, 'command_hold_timeout'),
    ({'~tag_loss_timeout': 0.3}, 'command_hold_timeout'),
])
def test_invalid_yaw_and_dropout_parameter_ordering(env, params, error):
    env.params.update(params)
    with pytest.raises(ValueError, match=error):
        alignment.AprilmoveqilinNode()
    alignment.DogBasic.assert_not_called()


def test_new_parameters_preserve_private_over_root_precedence(env):
    env.params.update({'/command_hold_timeout': 0.2, '~command_hold_timeout': 0.3,
                       '/yaw_enter_threshold': 0.01, '~yaw_enter_threshold': 0.02,
                       '/yaw_exit_threshold': 0.04, '~yaw_exit_threshold': 0.06})
    node = alignment.AprilmoveqilinNode()
    assert (node.command_hold_timeout, node.yaw_enter_threshold, node.yaw_exit_threshold) == (
        0.3, 0.02, 0.06
    )


@pytest.mark.parametrize('enter,exit_', [(-0.01, 0.035), (0.035, 0.035), (0.04, 0.035)])
def test_invalid_hysteresis_configuration(env, enter, exit_):
    env.params.update({'~align_enter_distance': enter, '~align_exit_distance': exit_})
    with pytest.raises(ValueError, match='align_enter_distance'):
        alignment.AprilmoveqilinNode()
    alignment.DogBasic.assert_not_called()


@pytest.mark.parametrize('gain,x,y,expected', [
    (0.2, 0.03, 0.04, (0.018, 0.024)),  # Requested 0.006/0.008 raw-speed example.
    (1.0, 0.3, 0.4, (0.15, 0.20)),     # Maximum applies to the whole vector.
    (1.0, -0.04, 0.001, (-0.04, 0.001)),  # No per-axis minimum or deadzone.
    (0.0, 0.3, 0.4, (0.0, 0.0)),
])
def test_resultant_speed_and_direction(env, gain, x, y, expected):
    env.params['~move_parameter'] = gain
    node = alignment.AprilmoveqilinNode()
    cycle(node, detection(0, x, y))
    assert command(node)[:2] == pytest.approx(expected)
    assert not node.xy_aligned


def test_hysteresis_boundaries_and_immediate_reversal(env):
    env.params['~move_parameter'] = 1.0
    env.params['~rotate_parameter'] = 0.5
    node = alignment.AprilmoveqilinNode()
    for x, aligned in [(0.03, False), (0.025, True), (0.03, True),
                       (0.034999, True), (0.035, False), (-0.035, False), (0.0, True)]:
        cycle(node, detection(0, x=x, yaw=0.2))
        assert node.xy_aligned == aligned
        assert command(node)[:2] == pytest.approx((0.0, 0.0) if aligned else (x, 0.0))
        assert command(node)[4] == pytest.approx(0.1)  # XY stop does not stop yaw.
        assert debug(env, 'xy_aligned').data == aligned


@pytest.mark.parametrize('yaw,expected', [(0.0, 0.0), (0.01, 0.0), (-0.01, 0.0),
                                          (0.06, 0.05), (-0.06, -0.05),
                                          (0.4, 0.2), (1.0, 0.3), (-1.0, -0.3)])
def test_raw_yaw_limits_and_fallback_with_rotated_camera(env, yaw, expected):
    env.params['~rotate_parameter'] = 0.5
    camera = alignment.tft.euler_matrix(0, 0, np.pi / 2)
    env.params['/camera_drone_matrix'] = camera.ravel().tolist()
    node = alignment.AprilmoveqilinNode()
    cycle(node, detection(0, x=0.1, yaw=yaw), detection(1, x=0.1, yaw=-0.6))
    assert command(node)[4] == pytest.approx(expected)
    assert debug(env, 'yaw_source').data == 0
    assert debug(env, 'error').vector.z == pytest.approx(yaw)
    # Camera rotation affects XY only.
    assert command(node)[:2] == pytest.approx((0.0, 0.11), abs=1e-12)
    cycle(node, detection(1, x=0.1, yaw=yaw))
    assert command(node)[4] == pytest.approx(expected)
    assert command(node)[1] > 0.0
    assert debug(env, 'yaw_source').data == 1
    assert not debug(env, 'tag0_visible').data
    assert debug(env, 'tag1_visible').data


@pytest.mark.parametrize('sign', [1, -1])
def test_yaw_hysteresis_exact_boundaries(env, monkeypatch, sign):
    env.params['~rotate_parameter'] = 0.5
    node = alignment.AprilmoveqilinNode()
    for yaw, aligned in [(0.06, False), (0.04, False), (0.03, True),
                         (0.04, True), (0.049, True), (0.05, False)]:
        # Test exact comparator boundaries independently of quaternion round-off.
        monkeypatch.setattr(alignment.tft, 'euler_from_quaternion',
                            lambda q, value=sign * yaw: (0.0, 0.0, value))
        cycle(node, detection(0, x=0.1, yaw=sign * yaw))
        assert node.yaw_aligned == aligned
        assert command(node)[4] == (0.0 if aligned else sign * 0.05)
        assert debug(env, 'yaw_aligned').data == aligned
        assert command(node)[0] == pytest.approx(0.11)  # Yaw stop leaves XY independent.


def test_yaw_hysteresis_survives_tag_switch_and_near_zero_sign_changes(env):
    node = alignment.AprilmoveqilinNode()
    cycle(node, detection(0, x=0.1, yaw=0.02), detection(1, x=0.1, yaw=-0.2))
    assert node.yaw_aligned
    assert debug(env, 'yaw_source').data == 0
    for yaw in (0.04, -0.02, 0.001, -0.04):
        cycle(node, detection(1, x=0.1, yaw=yaw))
        assert node.yaw_aligned
        assert command(node)[4] == 0.0
        assert debug(env, 'yaw_source').data == 1
    cycle(node, detection(1, x=0.1, yaw=-0.06))
    assert not node.yaw_aligned
    assert command(node)[4] == -0.05


def test_pair_gap_boundary_and_continuity_over_camera_distance(env):
    node = alignment.AprilmoveqilinNode()
    pair = AprilTagDetectionArray(detections=[detection(0), detection(1, x=0.06)])
    fused = node.find_drone_center(pair)
    assert fused[:3, 3] == pytest.approx((0.03, 0, 1))
    fused[0, 3] = 9.0
    assert node.previous_drone_center_position[0] == pytest.approx(0.03)  # No alias.
    # Tag 1 stays near the previous estimate, despite Tag 0 being nearer the camera.
    pair.detections = [detection(0, z=0.1), detection(1, x=0.03, z=1.01)]
    cycle(node, *pair.detections)
    assert node.previous_drone_center_position == pytest.approx((0.03, 0, 1.01))
    assert debug(env, 'tag0_visible').data and debug(env, 'tag1_visible').data
    assert debug(env, 'yaw_source').data == 0  # Yaw independent of selected center.
    # Tag 0 wins ties in temporal distance, too.
    node.previous_drone_center_position = np.array([0.0, 0.0, 1.0])
    pair.detections = [detection(0, x=-0.1), detection(1, x=0.1)]
    assert node.find_drone_center(pair)[0, 3] == pytest.approx(-0.1)


def test_disagreeing_initial_pair_and_reacquisition_reset(env):
    node = alignment.AprilmoveqilinNode()
    cycle(node, detection(0, z=2.0, yaw=0.2), detection(1, z=0.1))
    assert node.previous_drone_center_position[2] == 2.0
    assert node.last_valid_ryaw != 0.0
    alignment.rospy.logwarn_throttle.assert_called()
    env.clock.wall += 0.500001  # ROS time is frozen; monotonic tracking loss still resets.
    node.align_dog_with_drone()
    assert command(node) == (0, 0, 0, 0, 0)
    assert not node.xy_aligned
    assert node.previous_drone_center_position is None
    assert node.dog_align_drone_matrix is None
    assert (node.last_valid_lx, node.last_valid_ly, node.last_valid_ryaw) == (0.0, 0.0, 0.0)
    assert debug(env, 'yaw_source').data == -1
    assert not debug(env, 'tag0_visible').data
    assert not debug(env, 'tag1_visible').data
    assert np.isnan(debug(env, 'error').vector.x)
    cycle(node, detection(0, z=0.1), detection(1, z=2.0))
    assert node.previous_drone_center_position[2] == 0.1


@pytest.mark.parametrize('dropout_type', ['empty', 'no_callback', 'irrelevant_tag', 'invalid_pose'])
@pytest.mark.parametrize('aligned,yaw_aligned', [(False, False), (True, False), (False, True)])
def test_dropout_hold_zero_then_reset_every_cycle(env, aligned, yaw_aligned, dropout_type):
    env.clock.wall = 0.0  # Represent the hold boundary exactly; ROS time remains frozen.
    node = alignment.AprilmoveqilinNode()
    cycle(node, detection(1, x=0.0 if aligned else 0.1,
                          y=0.0 if aligned else -0.04, yaw=0.02 if yaw_aligned else 0.2))
    assert node.xy_aligned == aligned
    assert node.yaw_aligned == yaw_aligned
    last_command = command(node)
    stored_command = (last_command[0], last_command[1], last_command[4])
    assert stored_command != (0.0, 0.0, 0.0)
    previous_position = node.previous_drone_center_position.copy()
    previous_transform = node.dog_align_drone_matrix.copy()
    last_tag_time = node.last_tag_time
    start = env.clock.wall
    # Includes both exact boundaries and successive 10 Hz missed detections.
    for age in (0.1, 0.2, 0.3, 0.35, 0.350001, 0.4, 0.5, 0.500001, 0.6):
        env.clock.wall = start + age
        count = node.dog_basic_function.qilin_cmd_vel.call_count
        if dropout_type == 'no_callback':
            node.align_dog_with_drone()
        elif dropout_type == 'irrelevant_tag':
            cycle(node, detection(9))
        elif dropout_type == 'invalid_pose':
            cycle(node, detection(1, x=float('nan')))
        else:
            cycle(node)
        assert node.dog_basic_function.qilin_cmd_vel.call_count == count + 1
        expected = last_command if age <= 0.35 else (0.0, 0.0, 0, 0, 0.0)
        assert command(node) == expected
        msg = debug(env, 'cmd')
        assert (msg.vector.x, msg.vector.y, 0, 0, msg.vector.z) == expected
        assert env.publishers['cmd'].publish.call_count == count + 1
        assert node.last_tag_time == last_tag_time  # Missing observations do not renew the timer.
        if age <= 0.5:
            assert node.xy_aligned == aligned
            assert node.yaw_aligned == yaw_aligned
            np.testing.assert_array_equal(node.previous_drone_center_position, previous_position)
            np.testing.assert_array_equal(node.dog_align_drone_matrix, previous_transform)
            assert (node.last_valid_lx, node.last_valid_ly, node.last_valid_ryaw) == stored_command
            assert node.last_valid_detection_time == start
            assert debug(env, 'lost_duration').data == pytest.approx(age)
        else:
            assert not node.xy_aligned
            assert not node.yaw_aligned
            assert node.previous_drone_center_position is None
            assert node.dog_align_drone_matrix is None
            assert (node.last_valid_lx, node.last_valid_ly, node.last_valid_ryaw) == (0.0, 0.0, 0.0)
            assert node.last_valid_detection_time is None
            assert np.isnan(debug(env, 'lost_duration').data)
        assert not debug(env, 'tag0_visible').data
        visible = dropout_type == 'invalid_pose' or (dropout_type == 'no_callback' and age < 0.5)
        assert debug(env, 'tag1_visible').data == visible
        assert debug(env, 'yaw_source').data == -1
        assert debug(env, 'xy_aligned').data == node.xy_aligned
        assert debug(env, 'yaw_aligned').data == node.yaw_aligned
        assert debug(env, 'command_hold_active').data == (age <= 0.35)


@pytest.mark.parametrize('dropout_age', [0.3, 0.4, 0.6])
def test_reacquisition_replaces_stored_command_and_restarts_hold(env, dropout_age):
    node = alignment.AprilmoveqilinNode()
    cycle(node, detection(0, x=0.1, y=0.04, yaw=0.2))
    old_command = command(node)
    env.clock.wall += dropout_age
    cycle(node, detection(9))
    cycle(node, detection(1, x=-0.1, y=-0.04, yaw=-0.2))
    new_command = command(node)
    assert new_command != old_command
    assert (node.last_valid_lx, node.last_valid_ly, node.last_valid_ryaw) == (
        new_command[0], new_command[1], new_command[4]
    )
    env.clock.wall += 0.1
    cycle(node)
    assert command(node) == new_command


def test_dropout_without_any_valid_command_sends_zero(env):
    node = alignment.AprilmoveqilinNode()
    env.clock.wall += 0.1
    cycle(node)
    assert command(node) == (0.0, 0.0, 0, 0, 0.0)
    assert (node.last_valid_lx, node.last_valid_ly, node.last_valid_ryaw) == (0.0, 0.0, 0.0)
    assert not debug(env, 'command_hold_active').data
    assert np.isnan(debug(env, 'lost_duration').data)


def test_valid_zero_command_can_be_held(env):
    node = alignment.AprilmoveqilinNode()
    cycle(node, detection(0, yaw=0.02))
    assert node.xy_aligned and node.yaw_aligned
    env.clock.wall += 0.1
    node.align_dog_with_drone()
    assert command(node) == (0.0, 0.0, 0, 0, 0.0)
    assert debug(env, 'command_hold_active').data
    assert node.xy_aligned and node.yaw_aligned


@pytest.mark.parametrize('age', [0.3, 0.4, 0.6])
def test_yaw_reacquisition_in_band_preserves_or_resets_alignment(env, age):
    node = alignment.AprilmoveqilinNode()
    cycle(node, detection(0, x=0.1, yaw=0.02))
    assert node.yaw_aligned
    env.clock.wall += age
    node.align_dog_with_drone()
    cycle(node, detection(1, x=0.1, yaw=0.04))
    assert node.yaw_aligned == (age <= 0.5)
    assert command(node)[4] == (0.0 if age <= 0.5 else 0.05)
    assert not debug(env, 'command_hold_active').data


def test_new_callback_with_repeated_header_is_processed_once(env):
    node = alignment.AprilmoveqilinNode()
    data = AprilTagDetectionArray(detections=[detection(0, x=0.1, yaw=0.2)])
    node._callback_apriltag(data)
    node.align_dog_with_drone()
    first_time = node.last_valid_detection_time
    env.clock.wall += 0.1
    env.clock.ros += 1000.0  # ROS time/header sequencing does not control dropout age.
    node.align_dog_with_drone()
    assert node.last_valid_detection_time == first_time
    assert debug(env, 'command_hold_active').data
    # The callback itself is the event, even with a repeated object and zero header stamp.
    node._callback_apriltag(data)
    node.align_dog_with_drone()
    assert node.apriltag_message_counter == node.last_processed_message_counter == 2
    assert node.last_valid_detection_time == env.clock.wall
    assert not debug(env, 'command_hold_active').data


def test_message_arriving_during_control_is_processed_next_cycle(env, monkeypatch):
    node = alignment.AprilmoveqilinNode()
    original_find = node.find_drone_center
    next_data = AprilTagDetectionArray(detections=[detection(1, x=-0.1, yaw=-0.2)])

    def find_and_receive(data):
        node._callback_apriltag(next_data)
        return original_find(data)

    monkeypatch.setattr(node, 'find_drone_center', find_and_receive)
    cycle(node, detection(0, x=0.1, yaw=0.2))
    assert command(node)[0] > 0.0
    assert command(node)[4] > 0.0
    assert debug(env, 'tag0_visible').data and not debug(env, 'tag1_visible').data
    assert node.apriltag_message_counter == 2
    assert node.last_processed_message_counter == 1
    monkeypatch.setattr(node, 'find_drone_center', original_find)
    node.align_dog_with_drone()
    assert command(node)[0] < 0.0
    assert command(node)[4] < 0.0
    assert debug(env, 'yaw_source').data == 1
    assert node.last_processed_message_counter == 2


def test_expired_unprocessed_message_cannot_restart_control(env):
    node = alignment.AprilmoveqilinNode()
    node._callback_apriltag(AprilTagDetectionArray(detections=[detection(0, x=0.1)]))
    env.clock.wall += 0.5
    node.align_dog_with_drone()
    assert command(node) == (0.0, 0.0, 0, 0, 0.0)
    assert node.last_valid_detection_time is None
    assert not debug(env, 'command_hold_active').data
    assert not debug(env, 'tag0_visible').data
    assert node.previous_drone_center_position is None


def test_stream_freshness_threshold_does_not_extend_or_cut_short_hold(env):
    env.params['~apriltag_message_timeout'] = 0.2
    node = alignment.AprilmoveqilinNode()
    cycle(node, detection(0, x=0.1, yaw=0.2))
    valid_command = command(node)
    start = env.clock.wall
    env.clock.wall = start + 0.3
    assert not node.has_fresh_apriltag()
    node.align_dog_with_drone()
    assert command(node) == valid_command
    assert debug(env, 'command_hold_active').data
    env.clock.wall = start + 0.4
    node.align_dog_with_drone()
    assert command(node) == (0.0, 0.0, 0, 0, 0.0)
    assert not debug(env, 'command_hold_active').data
    assert node.last_valid_detection_time == start
    assert node.previous_drone_center_position is not None


def test_configured_timeouts_use_callback_arrival_not_message_stamp(env):
    env.params.update({'~apriltag_message_timeout': 0.25, '~command_hold_timeout': 0.1,
                       '~tag_loss_timeout': 0.125})
    node = alignment.AprilmoveqilinNode()
    cycle(node, detection(0))  # The detection array's header stamp is zero.
    assert node.has_fresh_apriltag()
    env.clock.wall += 0.25
    assert not node.has_fresh_apriltag()
    node.align_dog_with_drone()
    assert command(node) == (0, 0, 0, 0, 0)
    cycle(node, detection(1))
    env.clock.wall += 0.25
    cycle(node)  # Fresh topic, but no valid center beyond the configured grace period.
    assert not node.xy_aligned
    assert node.previous_drone_center_position is None


def test_invalid_pose_falls_back_without_false_visibility(env):
    node = alignment.AprilmoveqilinNode()
    cycle(node, detection(0, x=float('nan')), detection(1, x=0.1))
    assert debug(env, 'tag0_visible').data  # Present in message, but unusable for control.
    assert debug(env, 'yaw_source').data == 1
    assert np.all(np.isfinite(command(node)))
    assert node.previous_drone_center_position == pytest.approx((0.1, 0, 1))


def test_debug_command_exactly_matches_every_sent_command(env):
    node = alignment.AprilmoveqilinNode()
    node.align_dog_with_drone()  # No message yet: stop.
    cycle(node, detection(0, x=0.1, y=-0.04, yaw=-0.1))
    assert debug(env, 'error').vector.x == pytest.approx(0.1)
    assert debug(env, 'error').vector.y == pytest.approx(-0.04)
    assert debug(env, 'error').vector.z == pytest.approx(-0.1)
    assert debug(env, 'distance').data == pytest.approx(np.hypot(0.1, -0.04))
    node._stop_alignment()
    calls = node.dog_basic_function.qilin_cmd_vel.call_args_list
    debug_calls = env.publishers['cmd'].publish.call_args_list
    assert len(calls) == len(debug_calls) == 3
    for sent, recorded in zip(calls, debug_calls):
        msg = recorded.args[0]
        assert (msg.vector.x, msg.vector.y, 0, 0, msg.vector.z) == sent.args
        assert msg.header.stamp == alignment.rospy.Time(env.clock.ros)


def test_repository_calibration_and_yaml(env):
    root = Path(__file__).resolve().parents[1]
    for filename in ('LandingInfo.yaml', 'DroneTagsMatrix.yaml', 'CameraDroneMatrix.yaml'):
        config = yaml.safe_load((root / 'config' / filename).read_text())
        env.params.update({'/' + key: value for key, value in config.items()})
    node = alignment.AprilmoveqilinNode()
    assert (node.min_linear_vel, node.max_pair_gap) == (0.03, 0.06)
    assert (node.command_hold_timeout, node.tag_loss_timeout) == (0.35, 0.5)
    assert (node.yaw_enter_threshold, node.yaw_exit_threshold) == (0.03, 0.05)
    assert (node.move_param, node.rotate_param) == (1.0, 0.3)
    # Real +/- 0.065 m tag offsets both estimate the same camera-frame center.
    cycle(node, detection(0, x=0.04, y=-0.195), detection(1, x=0.04, y=-0.065))
    assert node.previous_drone_center_position == pytest.approx((0.04, -0.13, 1.0))
    assert command(node) == pytest.approx((0.0, 0.04, 0, 0, 0), abs=1e-12)

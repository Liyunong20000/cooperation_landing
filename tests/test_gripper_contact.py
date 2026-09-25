"""UAV contact protection and remote request regressions; no ROS master/hardware.

Run: PYTHONPATH="src:$PYTHONPATH" python3 -m pytest -q tests/test_gripper_contact.py
"""

import threading
from io import BytesIO
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from spinal.msg import ServoControlCmd, ServoState, ServoStates
from std_msgs.msg import Bool, Int8, Int32, UInt8

from xuanwu import event_trigger as bridge
from cooperation_landing.gripper import gripper_move as gripper


@pytest.fixture
def env(monkeypatch):
    clock = SimpleNamespace(now=0.0, shutdown=False, tick=lambda: None)
    params = {}
    publishers, subscribers = [], []

    def publisher(topic, msg_type, **kwargs):
        pub = Mock(topic=topic, msg_type=msg_type, options=kwargs)
        publishers.append(pub)
        return pub

    def subscriber(topic, msg_type, callback, **kwargs):
        sub = SimpleNamespace(topic=topic, callback=callback)
        subscribers.append(sub)
        return sub

    monkeypatch.setattr(gripper.rospy, 'get_param', lambda name, default: params.get(name, default))
    monkeypatch.setattr(gripper.rospy, 'Publisher', publisher)
    monkeypatch.setattr(gripper.rospy, 'Subscriber', subscriber)
    monkeypatch.setattr(gripper.rospy, 'sleep', Mock())
    monkeypatch.setattr(gripper.rospy, 'is_shutdown', lambda: clock.shutdown)
    monkeypatch.setattr(gripper.rospy.Time, 'now', lambda: gripper.rospy.Time(clock.now))

    def tick():
        clock.now += 1.0 / 3
        clock.tick()

    monkeypatch.setattr(gripper.rospy, 'Rate', lambda hz: SimpleNamespace(sleep=tick))
    for name in ('logdebug', 'loginfo', 'logwarn', 'logwarn_throttle', 'logerr'):
        monkeypatch.setattr(gripper.rospy, name, Mock())
    return SimpleNamespace(clock=clock, params=params, publishers=publishers, subscribers=subscribers)


def feedback(node, angle=700, load=0, error=0, index=0):
    node._callback_servo_states(ServoStates(servos=[
        ServoState(index=index, angle=angle, load=load, error=error)
    ]))


def targets(node):
    return [call.args[0].angles[0] for call in node.pub_servo_target.publish.call_args_list]


def request(node, angle, index=0):
    node._callback_servo_target_states_info(ServoControlCmd(index=[index], angles=[angle]))
    node._callback_servo_target_states_trigger(None)


def contact(node):
    feedback(node, angle=900)
    request(node, 650)
    feedback(node, load=300)
    feedback(node, load=310)
    assert node._servo_contact_latched


def test_normal_remote_closing_without_contact(env):
    node = bridge.EventtriggerNode()
    feedback(node, angle=900)
    for angle in (850, 800, 750):
        request(node, angle)
        feedback(node, angle=angle, load=100)
    assert targets(node) == [850, 800, 750]
    assert node._servo_closing_active
    assert not node._servo_contact_latched


@pytest.mark.parametrize('sign', [-1, 1])
def test_new_local_sample_confirmation_and_immediate_release(env, sign):
    node = bridge.EventtriggerNode()
    feedback(node, angle=900)
    request(node, 650)
    feedback(node, angle=705, load=sign * 250)
    feedback(node, angle=705, load=sign * 315)
    for _ in range(5):
        request(node, 650)
    assert not node._servo_contact_latched
    assert node._servo_contact_count == 1
    gripper.rospy.sleep.reset_mock()
    feedback(node, angle=700, load=sign * 330)
    assert node._servo_contact_latched
    assert not node._servo_closing_active
    assert targets(node)[-1] == 710
    gripper.rospy.sleep.assert_not_called()
    count = len(targets(node))
    for angle in (650, 640, 600, 700) * 5:
        request(node, angle)
    assert len(targets(node)) == count
    assert node._servo_contact_latched


def test_spike_resets_counter(env):
    node = bridge.EventtriggerNode()
    feedback(node, angle=900)
    request(node, 650)
    for load in (250, 320, 260):
        feedback(node, load=load)
        assert not node._servo_contact_latched
    assert node._servo_contact_count == 0
    assert targets(node) == [650]


def test_only_explicit_return_clears_latch_and_allows_next_close(env):
    node = bridge.EventtriggerNode()
    contact(node)
    # Present position is still 700, before the physical servo reaches safe goal 710.
    assert node.servo_angle == 700
    for angle in (650, 700, 705, 750, 1000, 1400):
        request(node, angle)
        assert node._servo_contact_latched
        assert targets(node) == [650, 710]
    node._callback_servo_return_trigger(None)
    assert targets(node)[-1] == 1400
    assert not node._servo_contact_latched
    assert node._servo_contact_count == 0
    assert not node._servo_closing_active
    request(node, 600)
    feedback(node, load=300)
    assert not node._servo_contact_latched
    feedback(node, load=300)
    assert targets(node)[-1] == 710


def test_generic_open_before_contact_remains_allowed(env):
    node = bridge.EventtriggerNode()
    feedback(node, angle=900)
    request(node, 650)
    feedback(node, load=315)
    request(node, 1000)
    assert targets(node) == [650, 1000]
    assert not node._servo_closing_active
    assert node._servo_contact_count == 0
    assert not node._servo_contact_latched


@pytest.mark.parametrize('method', ['return_zero', 'return_zero_qilin'])
def test_remote_return_uses_explicit_trigger_and_resets_uav(env, method):
    node = bridge.EventtriggerNode()
    remote = gripper.GripperMoveNode()
    contact(node)
    feedback(remote, load=330)
    feedback(remote, load=330)
    remote.pub_servo_return_qilin_trigger.publish.side_effect = node._callback_servo_return_trigger
    getattr(remote, method)()
    remote.pub_servo_return_qilin_trigger.publish.assert_called_once()
    remote.pub_servo_target_qilin.publish.assert_not_called()
    remote.pub_servo_target_qilin_trigger.publish.assert_not_called()
    assert not node._servo_contact_latched
    assert targets(node)[-1] == node.servo_max_angles
    assert remote._grasp_observation_count == 0


def test_limits_and_no_object(env):
    node = bridge.EventtriggerNode()
    feedback(node, angle=900)
    request(node, -9999)
    assert targets(node)[-1] == -150
    feedback(node, angle=-150)
    assert not node._servo_closing_active
    for _ in range(5):
        request(node, -9999)
    request(node, 9999)
    assert all(-150 <= target <= 1400 for target in targets(node))
    assert targets(node)[-1] == 1400


def test_parameters_namespace_and_release_clamp(env):
    env.params.update({'~robot_ns': '/custom_uav/',
                       '/custom_uav/servo_info/min_angles': -120,
                       '/custom_uav/servo_info/max_angles': 1300,
                       '~grasp_contact_load_threshold': 200,
                       '~grasp_contact_confirm_samples': 3,
                       '~grasp_release_offset': 20})
    node = bridge.EventtriggerNode()
    assert node.pub_servo_target.topic == '/custom_uav/servo/target_states'
    assert node._servo_states_sub.topic == '/custom_uav/servo/states'
    feedback(node, angle=1295)
    request(node, -9999)
    assert targets(node) == [-120]
    for _ in range(2):
        feedback(node, angle=1295, load=200)
        assert not node._servo_contact_latched
    feedback(node, angle=1295, load=200)
    assert node._servo_contact_latched
    assert targets(node)[-1] == 1300
    feedback(node, angle=1300)
    node._callback_servo_return_trigger(None)
    assert not node._servo_contact_latched  # Explicit reset also works at equal/max position.


@pytest.mark.parametrize('error', [1, 32, 64, 128])
def test_hardware_error_cancels_goal_and_blocks_close(env, error):
    node = bridge.EventtriggerNode()
    feedback(node, angle=900)
    request(node, 650)
    feedback(node, angle=700, error=error)
    assert targets(node) == [650, 700]
    assert node._servo_error_latched
    request(node, 600)
    feedback(node, error=0)
    request(node, 600)
    assert targets(node) == [650, 700]
    for angle in (700, 705, 750, 1000, 1400):
        request(node, angle)
        assert node._servo_error_latched
        assert targets(node) == [650, 700]
    node._callback_servo_return_trigger(None)
    assert not node._servo_error_latched
    request(node, 600)
    assert targets(node)[-1] == 600


def test_initial_error_requires_explicit_return(env):
    node = bridge.EventtriggerNode()
    feedback(node, error=32)
    request(node, 600)
    assert targets(node) == [700]
    request(node, 1000)
    assert targets(node) == [700]
    node._callback_servo_return_trigger(None)
    assert targets(node)[-1] == 1400


def test_missing_feedback_and_malformed_requests(env):
    node = bridge.EventtriggerNode()
    request(node, 650)
    node._callback_servo_return_trigger(None)
    node._callback_servo_states(ServoStates())
    assert not targets(node)
    feedback(node)
    for msg in (ServoControlCmd(), ServoControlCmd(index=[0], angles=[]),
                ServoControlCmd(index=[0, 1], angles=[600])):
        node._callback_servo_target_states_info(msg)
        node._callback_servo_target_states_trigger(None)
    request(node, 600, index=1)
    assert not targets(node)


def test_command_disable_does_not_disable_active_safety(env):
    node = bridge.EventtriggerNode()
    feedback(node, angle=900)
    request(node, 650)
    node.allow_remote_commands = False
    feedback(node, load=300)
    feedback(node, load=310)
    assert targets(node) == [650, 710]
    request(node, 1000)
    node._callback_servo_return_trigger(None)
    assert targets(node) == [650, 710]
    assert node._servo_contact_latched


def test_local_protection_with_remote_feedback_and_commands_delayed(env):
    node = bridge.EventtriggerNode()
    remote = gripper.GripperMoveNode()
    feedback(remote, angle=900)
    remote.servo_target_cmd_qilin(0, 650)
    # Deliver the request, but withhold all UAV telemetry from the remote client.
    request(node, 650)  # Still no local feedback: this request must be rejected.
    assert not targets(node)
    feedback(node, angle=900)
    request(node, 650)
    feedback(node, load=315)
    feedback(node, load=330)
    assert targets(node) == [650, 710]
    assert remote.servo_angle == 900
    assert remote.servo_load == 0
    # Deliver queued requests long after the local callback already released contact.
    env.clock.now = 20
    for angle in (660, 640):
        request(node, angle)
    assert targets(node) == [650, 710]
    # Remote telemetry delivery never produces a competing correction.
    before = remote.pub_servo_target_qilin.publish.call_count
    feedback(remote, load=330)
    feedback(remote, load=330)
    assert remote.grasp(0, 50) is True
    assert remote.pub_servo_target_qilin.publish.call_count == before


@pytest.mark.parametrize('delayed_angle', [600, 705, 750])
def test_waiting_trigger_cannot_overwrite_contact_release(env, delayed_angle):
    node = bridge.EventtriggerNode()
    feedback(node, angle=900)
    request(node, 650)
    feedback(node, load=300)
    node._callback_servo_target_states_info(ServoControlCmd(index=[0], angles=[delayed_angle]))
    attempted = threading.Event()

    def trigger():
        attempted.set()
        node._callback_servo_target_states_trigger(None)

    with node._servo_lock:
        thread = threading.Thread(target=trigger, daemon=True)
        thread.start()
        assert attempted.wait(1)
        feedback(node, load=310)
        assert targets(node)[-1] == 710
    thread.join(timeout=1)
    assert not thread.is_alive()
    assert targets(node) == [650, 710]


def test_local_callback_runs_during_trigger_delay(env):
    node = bridge.EventtriggerNode()
    feedback(node, angle=900)
    request(node, 650)
    feedback(node, load=300)

    def while_sleeping(_duration):
        thread = threading.Thread(target=lambda: feedback(node, load=330), daemon=True)
        thread.start()
        thread.join(timeout=1)
        assert not thread.is_alive()

    gripper.rospy.sleep.side_effect = while_sleeping
    request(node, 600)
    assert targets(node) == [650, 710]


@pytest.mark.parametrize('method', ['servo_target_cmd', 'servo_target_cmd_qilin'])
def test_all_client_commands_use_existing_gate_topics(env, method):
    remote = gripper.GripperMoveNode()
    getattr(remote, method)(0, -9999)
    assert all(pub.topic != '/xuanwu/servo/target_states' for pub in env.publishers)
    assert remote.pub_servo_target_qilin.topic == '/xuanwu/servo/target_states/info'
    assert remote.pub_servo_target_qilin.publish.call_args.args[0].angles == [-150]
    remote.pub_servo_target_qilin_trigger.publish.assert_called_once()
    count = remote.pub_servo_target_qilin.publish.call_count
    feedback(remote, load=400)
    feedback(remote, load=400)
    assert remote.pub_servo_target_qilin.publish.call_count == count


@pytest.mark.parametrize('method', ['grasp', 'grasp_qilin'])
def test_remote_task_incremental_success_uses_no_corrective_goal(env, method):
    remote = gripper.GripperMoveNode()
    feedback(remote, angle=900)
    samples = iter([(850, 0), (800, 0), (700, -330), (700, -330)])
    env.clock.tick = lambda: feedback(remote, *next(samples))
    assert getattr(remote, method)(0, 50) is True
    requests = remote.pub_servo_target_qilin.publish.call_args_list
    assert [call.args[0].angles[0] for call in requests] == [850, 800, 750, 650]


@pytest.mark.parametrize('loads', [(330,), (330, 260), (330, 260, 330)])
def test_remote_observation_does_not_recount_cached_sample_or_spike(env, loads):
    env.params['~grasp_timeout_s'] = 1.0
    remote = gripper.GripperMoveNode()
    for load in loads:
        feedback(remote, load=load)
    # Several loop iterations with no new feedback cannot turn one spike into success.
    assert remote.grasp(0, 50) is False
    assert remote.pub_servo_target_qilin.publish.call_count >= 2


def test_remote_observation_counts_new_samples_between_task_iterations(env):
    remote = gripper.GripperMoveNode()
    feedback(remote, angle=900)

    def tick():
        feedback(remote, load=315)
        feedback(remote, load=330)

    env.clock.tick = tick
    assert remote.grasp(0, 50) is True
    remote.pub_servo_target_qilin.publish.assert_called_once()


@pytest.mark.parametrize('abort', ['timeout', 'error', 'minimum', 'shutdown', 'interrupt'])
def test_remote_task_abort_is_not_a_second_safety_controller(env, abort):
    env.params['~grasp_timeout_s'] = 0.5
    remote = gripper.GripperMoveNode()
    feedback(remote, angle=700)

    def tick():
        if abort == 'error':
            feedback(remote, error=32)
        elif abort == 'minimum':
            feedback(remote, angle=-150)
        elif abort == 'shutdown':
            env.clock.shutdown = True
        elif abort == 'interrupt':
            raise gripper.rospy.ROSInterruptException('shutdown')

    env.clock.tick = tick
    assert remote.grasp(0, 50) is False
    assert all(pub.topic != '/xuanwu/servo/target_states' for pub in env.publishers)
    requests = remote.pub_servo_target_qilin.publish.call_args_list
    assert requests
    assert all(call.args[0].angles == [650] for call in requests)


def debug_snapshot(node):
    return {name: pub.publish.call_args.args[0].data
            for name, pub in node._gripper_debug_publishers.items()}


def test_debug_topics_types_namespace_and_queue_size(env):
    env.params.update({'~robot_ns': '/custom_uav/', '~grasp_contact_load_threshold': 325,
                       '~grasp_contact_confirm_samples': 3})
    node = bridge.EventtriggerNode()
    expected = {
        'closing_active': Bool, 'contact_count': UInt8, 'contact_latched': Bool,
        'error_latched': Bool, 'load_threshold': Int32, 'confirm_samples': UInt8,
        'last_requested_target': Int32, 'last_published_target': Int32,
        'last_safe_target': Int32, 'command_direction': Int8, 'contact_condition': Bool,
    }
    assert set(node._gripper_debug_publishers) == set(expected)
    feedback(node, angle=900)
    for name, msg_type in expected.items():
        pub = node._gripper_debug_publishers[name]
        assert pub.topic == '/custom_uav/gripper_debug/' + name
        assert pub.msg_type is msg_type
        assert pub.options == {'queue_size': 1}
        message = pub.publish.call_args.args[0]
        assert isinstance(message, msg_type)
        message.serialize(BytesIO())
    assert debug_snapshot(node)['load_threshold'] == 325
    assert debug_snapshot(node)['confirm_samples'] == 3


def test_debug_exposes_high_current_while_not_closing(env):
    node = bridge.EventtriggerNode()
    for _ in range(3):
        feedback(node, angle=700, load=-470)
    sample = debug_snapshot(node)
    assert sample['contact_condition'] is True
    assert sample['closing_active'] is False
    assert sample['contact_count'] == 0
    assert sample['contact_latched'] is False
    assert sample['error_latched'] is False
    assert sample['last_safe_target'] == -999999
    assert not targets(node)
    for pub in node._gripper_debug_publishers.values():
        assert pub.publish.call_count == 3


def test_debug_contact_sequence_is_after_physical_safety_publish(env):
    node = bridge.EventtriggerNode()
    feedback(node, angle=900)
    request(node, 650)
    feedback(node, angle=700, load=-250)
    sample = debug_snapshot(node)
    assert sample['command_direction'] == -1
    assert sample['closing_active'] is True
    assert sample['last_requested_target'] == 650
    assert sample['last_published_target'] == 650
    assert sample['contact_condition'] is False
    assert sample['contact_count'] == 0
    feedback(node, angle=700, load=-315)
    assert debug_snapshot(node)['contact_count'] == 1
    order = []
    node.pub_servo_target.publish.side_effect = lambda msg: order.append(('physical', msg.angles[0]))
    node._gripper_debug_publishers['contact_latched'].publish.side_effect = (
        lambda msg: order.append(('debug', msg.data))
    )
    feedback(node, angle=700, load=-330)
    assert order == [('physical', 710), ('debug', True)]
    sample = debug_snapshot(node)
    assert sample['contact_count'] == 2
    assert sample['contact_condition'] is True
    assert sample['contact_latched'] is True
    assert sample['closing_active'] is False
    assert sample['last_safe_target'] == sample['last_published_target'] == 710
    gripper.rospy.sleep.reset_mock()
    feedback(node, angle=710, load=0)
    assert debug_snapshot(node)['last_safe_target'] == 710
    gripper.rospy.sleep.assert_not_called()


@pytest.mark.parametrize('angle,direction', [(650, -1), (700, 0), (705, 1), (750, 1)])
def test_debug_rejected_target_is_distinct_from_published_target(env, angle, direction):
    node = bridge.EventtriggerNode()
    contact(node)
    request(node, angle)
    feedback(node, angle=700, load=-400)
    sample = debug_snapshot(node)
    assert sample['last_requested_target'] == angle
    assert sample['command_direction'] == direction
    assert sample['last_published_target'] == sample['last_safe_target'] == 710
    assert sample['contact_latched'] is True
    assert targets(node) == [650, 710]


def test_debug_open_clamp_and_hold_mirror_existing_direction(env):
    node = bridge.EventtriggerNode()
    feedback(node, angle=900)
    request(node, 9999)
    feedback(node, angle=950)
    sample = debug_snapshot(node)
    assert sample['command_direction'] == 1
    assert sample['closing_active'] is False
    assert sample['last_requested_target'] == 9999
    assert sample['last_published_target'] == 1400
    feedback(node, angle=-150)
    request(node, -9999)  # The control clamp makes this equal, not closing.
    feedback(node, angle=-150)
    assert debug_snapshot(node)['command_direction'] == 0


def test_debug_error_early_return_and_explicit_reset(env):
    node = bridge.EventtriggerNode()
    feedback(node, angle=900)
    request(node, 650)
    feedback(node, angle=700, load=-400, error=32)
    sample = debug_snapshot(node)
    assert sample['error_latched'] is True
    assert sample['closing_active'] is False
    assert sample['last_published_target'] == 700
    assert sample['last_safe_target'] == -999999
    node._callback_servo_return_trigger(None)
    feedback(node, angle=800)
    sample = debug_snapshot(node)
    assert sample['error_latched'] is False
    assert sample['last_requested_target'] == sample['last_published_target'] == 1400


def test_debug_missing_feedback_and_empty_message(env):
    node = bridge.EventtriggerNode()
    assert node.servo_target_cmd(0, 650) is False
    node._callback_servo_states(ServoStates())
    sample = debug_snapshot(node)
    assert sample['last_requested_target'] == 650
    assert sample['command_direction'] == 0
    assert sample['last_published_target'] == 0
    assert sample['last_safe_target'] == -999999
    assert not targets(node)
    for pub in node._gripper_debug_publishers.values():
        pub.publish.assert_called_once()

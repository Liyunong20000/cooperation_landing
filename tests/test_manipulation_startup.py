"""Startup ordering: communication precedes execution; stall recovery is disabled."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from cooperation_landing import manipulation_motion as motion


@pytest.fixture
def env(monkeypatch):
    events = []
    params = {}
    checker = Mock()
    checker.check.side_effect = lambda: events.append('preflight') or True
    machine = Mock()
    machine.execute.side_effect = lambda: events.append('execute')
    server = Mock()
    server.start.side_effect = lambda: events.append('server_start')
    server.stop.side_effect = lambda: events.append('server_stop')
    build = Mock(side_effect=lambda: events.append('build') or machine)
    monitor = Mock(side_effect=lambda: events.append('monitor'))
    monkeypatch.setattr(motion.rospy, 'init_node', Mock())
    monkeypatch.setattr(motion.rospy, 'get_param', lambda key, default: params.get(key, default))
    monkeypatch.setattr(motion.rospy, 'loginfo', Mock())
    monkeypatch.setattr(motion.rospy, 'logwarn', Mock())
    monkeypatch.setattr(motion.rospy, 'logerr', Mock())
    monkeypatch.setattr(motion, 'BridgePreflight', Mock(return_value=checker))
    monkeypatch.setattr(motion, 'build_state_machine', build)
    monkeypatch.setattr(motion, 'DogStallMonitor', monitor, raising=False)
    monkeypatch.setattr(motion.smach_ros, 'IntrospectionServer', Mock(return_value=server))
    return SimpleNamespace(events=events, params=params, checker=checker, build=build,
                           monitor=monitor, machine=machine, server=server)


def test_missing_bridge_topic_blocks_all_task_and_monitor_startup(env):
    env.params['~enable_dog_stall_monitor'] = True
    env.checker.check.side_effect = lambda: env.events.append('preflight') or False
    motion.main()
    assert env.events == ['preflight']
    env.build.assert_not_called()
    env.monitor.assert_not_called()
    env.machine.execute.assert_not_called()
    motion.smach_ros.IntrospectionServer.assert_not_called()
    motion.rospy.logerr.assert_called_once()


@pytest.mark.parametrize('monitor_enabled', [False, True])
def test_all_bridge_topics_pass_before_state_machine_runs(env, monitor_enabled):
    env.params['~enable_dog_stall_monitor'] = monitor_enabled
    motion.main()
    expected = ['preflight', 'build', 'server_start', 'execute', 'server_stop']
    assert env.events == expected
    env.checker.check.assert_called_once()
    env.monitor.assert_not_called()
    motion.BridgePreflight.assert_called_once_with(timeout_default=5.0)
    motion.rospy.init_node.assert_called_once_with('manipulation_motion')


def test_introspection_still_stops_after_execution_error(env):
    env.machine.execute.side_effect = RuntimeError('execution failed')
    with pytest.raises(RuntimeError, match='execution failed'):
        motion.main()
    env.checker.check.assert_called_once()
    env.server.stop.assert_called_once()

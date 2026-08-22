#!/usr/bin/env python3
"""Unpause Gazebo once ros_control is ready, then verify all controllers."""

import time

import rospy
from controller_manager_msgs.srv import ListControllers
from gazebo_msgs.srv import GetPhysicsProperties, SetPhysicsProperties
from geometry_msgs.msg import Vector3
from std_srvs.srv import Empty


def main():
    rospy.init_node("wait_for_controllers_and_unpause")

    required = set(rospy.get_param("~required_controllers", []))
    timeout = float(rospy.get_param("~timeout", 45.0))
    spawn_configuration_delay = float(
        rospy.get_param("~spawn_configuration_delay", 1.5)
    )
    gravity_ramp_steps = max(1, int(rospy.get_param("~gravity_ramp_steps", 10)))
    gravity_ramp_step_duration = float(
        rospy.get_param("~gravity_ramp_step_duration", 0.1)
    )
    deadline = time.monotonic() + timeout

    rospy.loginfo("Waiting for the Gazebo controller manager")
    rospy.wait_for_service("/controller_manager/list_controllers", timeout=timeout)

    # JointTrajectoryController initialization needs Gazebo update callbacks.
    # spawn_model also waits one wall-clock second before applying its -J joint
    # positions.  Keep physics paused slightly longer so controllers capture the
    # intended crouched/tucked pose instead of a transient near-zero arm pose.
    time.sleep(spawn_configuration_delay)

    # Let ros_control finish loading without a free-fall interval.  Gravity is
    # restored only after every controller reports running, at which point a
    # configured spawn clearance becomes a deliberate drop test.
    rospy.wait_for_service("/gazebo/get_physics_properties", timeout=timeout)
    rospy.wait_for_service("/gazebo/set_physics_properties", timeout=timeout)
    get_physics = rospy.ServiceProxy(
        "/gazebo/get_physics_properties", GetPhysicsProperties
    )
    set_physics = rospy.ServiceProxy(
        "/gazebo/set_physics_properties", SetPhysicsProperties
    )
    physics = get_physics()
    if not physics.success:
        raise RuntimeError("Unable to read Gazebo physics properties")

    zero_gravity = Vector3(0.0, 0.0, 0.0)
    response = set_physics(
        physics.time_step,
        physics.max_update_rate,
        zero_gravity,
        physics.ode_config,
    )
    if not response.success:
        raise RuntimeError("Unable to disable gravity before controller startup")

    # Unpause after that configuration window; otherwise a paused world and the
    # trajectory-controller spawners can wait on each other forever.
    rospy.wait_for_service("/gazebo/unpause_physics", timeout=timeout)
    rospy.ServiceProxy("/gazebo/unpause_physics", Empty)()
    rospy.loginfo("Gazebo is unpaused; waiting for controllers: %s",
                  ", ".join(sorted(required)))

    list_controllers = rospy.ServiceProxy(
        "/controller_manager/list_controllers", ListControllers
    )

    while not rospy.is_shutdown() and time.monotonic() < deadline:
        response = list_controllers()
        running = {
            controller.name
            for controller in response.controller
            if controller.state == "running"
        }
        missing = required - running
        if not missing:
            break
        rospy.loginfo_throttle(
            2.0, "Still waiting for: %s", ", ".join(sorted(missing))
        )
        # Wall-clock sleep is intentional because Gazebo is paused and /clock
        # therefore does not advance yet.
        time.sleep(0.2)
    else:
        raise RuntimeError("Timed out waiting for the Spot+Kortex controllers")

    rospy.loginfo("All Spot+Kortex controllers are running; restoring gravity")
    for step in range(1, gravity_ramp_steps + 1):
        scale = float(step) / gravity_ramp_steps
        gravity = Vector3(
            physics.gravity.x * scale,
            physics.gravity.y * scale,
            physics.gravity.z * scale,
        )
        response = set_physics(
            physics.time_step,
            physics.max_update_rate,
            gravity,
            physics.ode_config,
        )
        if not response.success:
            raise RuntimeError("Unable to restore Gazebo gravity")
        time.sleep(gravity_ramp_step_duration)
    rospy.loginfo("Gazebo gravity restored")


if __name__ == "__main__":
    try:
        main()
    except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
        rospy.logfatal("Unable to initialize Spot+Kortex simulation: %s", error)
        raise SystemExit(1)

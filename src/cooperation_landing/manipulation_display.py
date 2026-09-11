"""Publish the current SMACH graph for inspection."""

import rospy
import smach_ros

from cooperation_landing.manipulation_motion import build_state_machine


def main():
    """Expose the task graph without executing states or recovery monitors."""
    rospy.init_node('manipulation_motion')
    state_machine = build_state_machine()
    server = smach_ros.IntrospectionServer('manipulation_smach_server', state_machine, '/SM_ROOT')
    server.start()
    try:
        rospy.spin()
    finally:
        server.stop()

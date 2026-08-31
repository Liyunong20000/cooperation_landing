#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
from dog_stall_monitor import DogStallMonitor

from manipulation_state_v2 import *

def main():
    rospy.init_node('manipulation_motion')

    # Keep one stall monitor for the complete state-machine lifetime. Because
    # it listens to the final /go1/cmd_vel topic, every DogBasic command issued
    # by every state is covered without adding recovery code to each state.
    _dog_stall_monitor = None
    if rospy.get_param('~enable_dog_stall_monitor', False):
        _dog_stall_monitor = DogStallMonitor()
    else:
        rospy.logwarn('Dog stall monitor is disabled by parameter.')

    # Load ROS parameters (far/near marker ids)
    picking_marker_far = rospy.get_param('~picking_marker_far', 0)
    picking_marker_near = rospy.get_param('~picking_marker_near', 1)
    placing_marker_far = rospy.get_param('~placing_marker_far', 2)
    placing_marker_near = rospy.get_param('~placing_marker_near', 3)

    # Keep existing state interfaces: picking/placing_marker represent FAR ids.

    object_state = rospy.get_param('object_state', 1)
    picking_position = rospy.get_param('picking_position', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    placing_position = rospy.get_param('placing_position', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    takeoff_position = rospy.get_param('takeoff_position', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    switching_threshold = rospy.get_param('switching_threshold', 0.40)
    detaching_takeoff_threshold = rospy.get_param('detaching_takeoff_threshold', 0.75)

    sm_top = smach.StateMachine(outcomes=['preempted'])

    sm_top.userdata.picking_marker_far = picking_marker_far
    sm_top.userdata.placing_marker_far = placing_marker_far
    sm_top.userdata.picking_marker_near = picking_marker_near
    sm_top.userdata.placing_marker_near = placing_marker_near
    sm_top.userdata.object_state = object_state
    sm_top.userdata.picking_position = picking_position
    sm_top.userdata.placing_position = placing_position
    sm_top.userdata.takeoff_position = takeoff_position
    sm_top.userdata.home_x = None
    sm_top.userdata.home_y = None
    sm_top.userdata.home_yaw = None

    sm_top.userdata.switching_threshold = switching_threshold
    sm_top.userdata.detaching_takeoff_threshold = detaching_takeoff_threshold

    with sm_top:
        # Sub-state machine for target search
        smach.StateMachine.add('Start', Start(),
                               transitions={'succeeded': 'TargetSearch'}, remapping={'object_state': 'object_state', 'home_x': 'home_x', 'home_y': 'home_y'})

        sm_target_search = smach.StateMachine(
            outcomes=['succeed_docking', 'succeed_detaching', 'failed'],
            input_keys=['object_state', 'picking_marker_far', 'placing_marker_far', 'switching_threshold', 'picking_position', 'placing_position'],
            output_keys=['picking_position', 'placing_position']
        )

        with sm_target_search:

            smach.StateMachine.add('HorizontalScan', HorizontalScan(),transitions={'succeeded': 'StateJudgment', 'need_vertical': 'VerticalScan', 'failed': 'failed'}, remapping={'object_state': 'object_state', 'picking_position': 'picking_position', 'placing_position': 'placing_position', 'picking_marker_far': 'picking_marker_far', 'placing_marker_far': 'placing_marker_far'})

            smach.StateMachine.add('VerticalScan', VerticalScan(), transitions={'succeeded': 'StateJudgment', 'failed': 'HorizontalScan'}, remapping={'object_state': 'object_state', 'picking_position': 'picking_position', 'placing_position': 'placing_position', 'picking_marker_far': 'picking_marker_far', 'placing_marker_far': 'placing_marker_far'})

            smach.StateMachine.add(
                'StateJudgment',
                StateJudgment(),
                transitions={'succeed_docking': 'succeed_docking', 'succeed_detaching': 'succeed_detaching'},
                remapping={
                    'object_state': 'object_state',
                    'switching_threshold': 'switching_threshold',
                    'picking_position': 'picking_position',
                    'placing_position': 'placing_position'
                }
            )

        smach.StateMachine.add('TargetSearch', sm_target_search, transitions={'succeed_docking': 'DockingManipulation','succeed_detaching': 'DetachingManipulation', 'failed': 'Finish'}, remapping={'object_state': 'object_state', 'picking_marker_far': 'picking_marker_far', 'placing_marker_far': 'placing_marker_far', 'picking_position': 'picking_position', 'placing_position': 'placing_position', 'switching_threshold': 'switching_threshold'})

        sm_docking_manipulation = smach.StateMachine(outcomes=['succeed', 'finish'], input_keys=['object_state', 'picking_marker_far', 'placing_marker_far', 'picking_marker_near', 'placing_marker_near', 'picking_position', 'placing_position', 'home_x', 'home_y', 'home_yaw'], output_keys=['object_state'])

        with sm_docking_manipulation:

            smach.StateMachine.add('DockApproach', DockApproach(),transitions={'succeeded': 'DockManipulation', 'failed': 'DockHome'}, remapping={'object_state': 'object_state', 'picking_marker_far': 'picking_marker_far', 'placing_marker_far': 'placing_marker_far', 'picking_marker_near': 'picking_marker_near', 'placing_marker_near': 'placing_marker_near'})

            smach.StateMachine.add('DockManipulation', DockManipulation(), transitions={'succeeded': 'DockHome'}, remapping={'object_state': 'object_state'})

            smach.StateMachine.add('DockHome', DockHome(), transitions={'succeed': 'succeed', 'finish': 'finish'}, remapping={'object_state': 'object_state', 'home_x': 'home_x', 'home_y': 'home_y', 'home_yaw': 'home_yaw'})

        smach.StateMachine.add('DockingManipulation', sm_docking_manipulation, transitions={'succeed': 'TargetSearch','finish': 'Finish'}, remapping={'object_state': 'object_state', 'picking_marker_far': 'picking_marker_far', 'placing_marker_far': 'placing_marker_far', 'picking_marker_near': 'picking_marker_near', 'placing_marker_near': 'placing_marker_near', 'picking_position': 'picking_position', 'placing_position': 'placing_position', 'home_x': 'home_x', 'home_y': 'home_y', 'home_yaw': 'home_yaw'})

        sm_detaching_manipulation = smach.StateMachine(outcomes=['succeed', 'failed'], input_keys=['object_state', 'picking_marker_far', 'placing_marker_far', 'picking_marker_near', 'placing_marker_near', 'picking_position', 'placing_position', 'detaching_takeoff_threshold', 'takeoff_position'], output_keys=['object_state', 'takeoff_position', 'picking_position', 'placing_position'])

        with sm_detaching_manipulation:

            smach.StateMachine.add('DetachApproach', DetachApproach(),transitions={'succeeded': 'DetachManipulation', 'failed': 'failed'}, remapping={'object_state': 'object_state', 'picking_marker_far': 'picking_marker_far', 'placing_marker_far': 'placing_marker_far', 'picking_marker_near': 'picking_marker_near', 'placing_marker_near': 'placing_marker_near', 'detaching_takeoff_threshold': 'detaching_takeoff_threshold', 'picking_position': 'picking_position', 'placing_position': 'placing_position'})
            
            sm_detach_manipulation = smach.StateMachine(
                outcomes=['succeed', 'failed'],
                input_keys=['object_state', 'picking_position', 'placing_position', 'takeoff_position'],
                output_keys=['object_state', 'takeoff_position']
            )

            with sm_detach_manipulation:
                smach.StateMachine.add('Takeoff', Takeoff(),
                                       transitions={'succeeded': 'FlyTarget', 'failed': 'failed'},
                                       remapping={'takeoff_position': 'takeoff_position'})
                smach.StateMachine.add('FlyTarget', FlyTarget(),
                                       transitions={'succeeded': 'succeed', 'failed': 'failed'},
                                       remapping={'object_state': 'object_state', 'picking_position': 'picking_position',
                                                  'placing_position': 'placing_position'})

            smach.StateMachine.add('DetachManipulation', sm_detach_manipulation, transitions={'succeed': 'succeed', 'failed': 'failed'}, remapping={'object_state': 'object_state', 'takeoff_position': 'takeoff_position', 'picking_position': 'picking_position', 'placing_position': 'placing_position'})

        smach.StateMachine.add('DetachingManipulation', sm_detaching_manipulation, transitions={'succeed': 'PreciseLanding', 'failed': 'Finish'}, remapping={'object_state': 'object_state', 'picking_marker_far': 'picking_marker_far', 'placing_marker_far': 'placing_marker_far', 'picking_marker_near': 'picking_marker_near', 'placing_marker_near': 'placing_marker_near', 'picking_position': 'picking_position', 'placing_position': 'placing_position', 'takeoff_position': 'takeoff_position', 'detaching_takeoff_threshold': 'detaching_takeoff_threshold'})

        sm_precise_landing = smach.StateMachine(outcomes=['succeed', 'finish'], input_keys=['object_state', 'takeoff_position'])

        with sm_precise_landing:

            smach.StateMachine.add('FlyBack', FlyBack(), transitions={'succeeded': 'AlignAndLand'}, remapping={'takeoff_position': 'takeoff_position'})

            smach.StateMachine.add('AlignAndLand', AlignAndLand(), transitions={'succeeded': 'succeed', 'failed': 'finish'})

        smach.StateMachine.add('PreciseLanding', sm_precise_landing, transitions={'succeed': 'TargetSearch', 'finish': 'Finish'}, remapping={'object_state': 'object_state', 'picking_marker_far': 'picking_marker_far', 'placing_marker_far': 'placing_marker_far', 'picking_position': 'picking_position', 'placing_position': 'placing_position', 'takeoff_position': 'takeoff_position'})

        smach.StateMachine.add('Finish', Finish(), transitions={})

        sis = smach_ros.IntrospectionServer('manipulation_smach_server', sm_top, '/SM_ROOT')
        sis.start()

    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
from manipulation_state_v2 import *

def main():
    rospy.init_node('manipulation_motion')

    # Load ROS parameters
    picking_marker = rospy.get_param('picking_marker', 11)  # Default to 11
    placing_marker = rospy.get_param('placing_marker', 12)  # Default to 12

    sm_top = smach.StateMachine(outcomes=['preempted'])
    sm_top.userdata.rm = 1  # real_machine = 1
    sm_top.userdata.picking_marker = picking_marker
    sm_top.userdata.placing_marker = placing_marker

    with sm_top:
        # Sub-state machine for target search
        sm_target_search = smach.StateMachine(outcomes=['to_docking', 'to_detaching', 'finished'], input_keys=['rm', 'picking_marker', 'placing_marker'], output_keys=['scan_results'])

        with sm_target_search:
            smach.StateMachine.add('HorizontalScan', HorizontalScan(),
                                   transitions={'scanned': 'VerticalScan', 'finished': 'finished'},
                                   remapping={'rm': 'rm', 'scan_results': 'scan_results'})

            smach.StateMachine.add('VerticalScan', VerticalScan(),
                                   transitions={'scanned': 'StateJudgment'},
                                   remapping={'rm': 'rm', 'scan_results': 'scan_results'})

            smach.StateMachine.add('StateJudgment', StateJudgment(),
                                   transitions={'to_docking': 'to_docking',
                                                'to_detaching': 'to_detaching',
                                                'finished': 'finished'},
                                   remapping={'rm': 'rm', 'scan_results': 'scan_results', 'picking_marker': 'picking_marker', 'placing_marker': 'placing_marker'})

        smach.StateMachine.add('TargetSearch', sm_target_search,
                               transitions={'to_docking': 'DockingManipulation',
                                            'to_detaching': 'DetachingManipulation',
                                            'finished': 'Finish'},
                               remapping={'rm': 'rm', 'picking_marker': 'picking_marker', 'placing_marker': 'placing_marker'})

        smach.StateMachine.add('DockingManipulation', DockingManipulation(),
                               transitions={'back_to_search': 'TargetSearch',
                                            'finished': 'Finish'},
                               remapping={'rm': 'rm'})

        smach.StateMachine.add('DetachingManipulation', DetachingManipulation(),
                               transitions={'to_precise_landing': 'PreciseLanding'},
                               remapping={'rm': 'rm'})

        smach.StateMachine.add('PreciseLanding', PreciseLanding(),
                               transitions={'back_to_search': 'TargetSearch',
                                            'finished': 'Finish'},
                               remapping={'rm': 'rm'})

        smach.StateMachine.add('Finish', Finish(), transitions={})

        sis = smach_ros.IntrospectionServer('manipulation_smach_server', sm_top, '/SM_ROOT')
        sis.start()

    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
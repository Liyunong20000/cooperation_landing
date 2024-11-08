#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
from gitlab import VISIBILITY_PRIVATE
from inspection_state import *
from pyasn1_modules.rfc3281 import Target


def main():
    rospy.init_node('inspection_motion')
    # creat a smach state machine

    sm_top = smach.StateMachine(outcomes=['preempted'])
    sm_top.userdata.takeoff_position = [(0.0, 0.0, 0.0, 0.0)]
    sm_top.userdata.target_position = [(0.0, 0.0, 0.0)]

    sm_top.userdata.rm = 0  # if real_machine,= 1
    sm_top.userdata.camera_offset = 0.5

    with sm_top:
        smach.StateMachine.add('Start', Start(),
                               transitions={'succeeded': 'TargetSearch'}, remapping={'rm':'rm'})

        sm_target_search = smach.StateMachine(outcomes=['finished'], input_keys=['rm'], output_keys=['takeoff_position','target_position'])

        with sm_target_search:
            smach.StateMachine.add('TargetSearch', MarkerSearch(),transitions={'succeeded': 'TargetCalculation'}, remapping={'takeoff_position': 'takeoff_position'})

            smach.StateMachine.add('TargetCalculation', TargetCalculation(),transitions={'succeeded': 'finished'},remapping={'takeoff_position': 'takeoff_position', 'target_position': 'target_position'})

        smach.StateMachine.add('TargetSearch', sm_target_search,transitions={'finished': 'UavInspection'})

        sm_uav_inspection = smach.StateMachine(outcomes=['succeeded', 'failed'], input_keys=['takeoff_position','target_position', 'rm'] )

        with sm_uav_inspection:
            smach.StateMachine.add('Takeoff', UavTakeoff(),transitions={'succeeded': 'FlyTarget', 'failed': 'failed'})

            smach.StateMachine.add('FlyTarget', FlyTarget(),transitions={'succeeded': 'Inspection'},remapping={'target_position': 'target_position', 'rm': 'rm'})

            smach.StateMachine.add('Inspection', Inspection(),transitions={'succeeded': 'FlyBack'}, remapping={ 'target_position': 'target_position', 'rm': 'rm'})

            smach.StateMachine.add('FlyBack', FlyBack(),transitions={'succeeded': 'succeeded'}, remapping={'takeoff_position': 'takeoff_position', 'target_position':'target_position', 'rm': 'rm'})

        smach.StateMachine.add('UavInspection', sm_uav_inspection,transitions={'succeeded': 'PreciseLanding', 'failed': 'preempted'}, remapping={'takeoff_position': 'takeoff_position', 'target_position': 'target_position'})


        sm_precise_landing = smach.StateMachine(outcomes=['succeeded'], input_keys=['takeoff_position', 'rm'])
        with sm_precise_landing:
            smach.StateMachine.add('VisibilityAdjustment', VisibilityAdjustment(),transitions={'succeeded': 'AlignAndLand'}, remapping={'takeoff_position': 'takeoff_position', 'rm': 'rm'})

            smach.StateMachine.add('AlignAndLand', AlignAndLand(),transitions={'succeeded': 'succeeded','failed': 'VisibilityAdjustment'})
        smach.StateMachine.add('PreciseLanding', sm_precise_landing,transitions={'succeeded': 'Finish'})

        # smach.StateMachine.add('Idle', Idle(),transitions={'succeeded': MarkerSearch})

        smach.StateMachine.add('Finish', Finish(),transitions={})

        sis = smach_ros.IntrospectionServer('inspection_smach_server', sm_top, '/SM_ROOT')
        sis.start()

    outcome = sm_top.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
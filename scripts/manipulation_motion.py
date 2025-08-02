#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
from gitlab import VISIBILITY_PRIVATE
from manipulation_state import *
from pyasn1_modules.rfc3281 import Target


def main():
    rospy.init_node('manipulation_motion')
    # creat a smach state machine

    sm_top = smach.StateMachine(outcomes=['preempted'])
    sm_top.userdata.takeoff_position = [(0.0, 0.0, 0.0, 0.0)]
    sm_top.userdata.pick_position = [(2.5, 2.5, 0.0)]
    sm_top.userdata.put_position = [(4.0, 0.0, 0.0)]

    sm_top.userdata.rm = 0  # if real_machine,= 1
    sm_top.userdata.camera_offset = 0.5

    with sm_top:
        smach.StateMachine.add('Start', Start(),
                               transitions={'succeeded': 'TargetPick'}, remapping={'rm':'rm'})

        sm_target_pick = smach.StateMachine(outcomes=['finished'], input_keys=['rm', 'pick_position'])

        with sm_target_pick:

            smach.StateMachine.add('TargetSearch', TargetSearch(),transitions={'succeeded': 'pick'}, remapping={'rm': 'rm'})

            smach.StateMachine.add('pick', Pick(), transitions={'succeeded': 'finished'}, remapping={'rm': 'rm'})

        smach.StateMachine.add('TargetPick', sm_target_pick,transitions={'finished': 'TargetPut'})

        sm_target_put = smach.StateMachine(outcomes=['succeeded', 'failed'], input_keys=['rm'], output_keys=['put_position', 'takeoff_position'] )

        with sm_target_put:
            smach.StateMachine.add('MoveDestination', MoveDestination(),transitions={'succeeded': 'Takeoff', 'failed': 'failed'}, remapping={'rm': 'rm', 'put_position': 'put_position'})

            smach.StateMachine.add('Takeoff', Takeoff(),transitions={'succeeded': 'FlyTarget'},remapping={'rm': 'rm'})

            smach.StateMachine.add('FlyTarget', FlyTarget(),transitions={'succeeded': 'Correction'},remapping={'rm': 'rm', 'put_position': 'put_position', 'takeoff_position': 'takeoff_position'})

            smach.StateMachine.add('Correction', Correction(),transitions={'succeeded': 'succeeded'},remapping={'rm': 'rm', 'put_position': 'put_position'})

        smach.StateMachine.add('TargetPut', sm_target_put,transitions={'succeeded': 'Finish', 'failed': 'preempted'}, remapping={'takeoff_position': 'takeoff_position', 'target_position': 'target_position'})


        # sm_precise_landing = smach.StateMachine(outcomes=['succeeded'], input_keys=['takeoff_position', 'rm'])
        # with sm_precise_landing:
        #     smach.StateMachine.add('VisibilityAdjustment', VisibilityAdjustment(),transitions={'succeeded': 'AlignAndLand'}, remapping={'takeoff_position': 'takeoff_position', 'rm': 'rm'})
        #
        #     smach.StateMachine.add('AlignAndLand', AlignAndLand(),transitions={'succeeded': 'succeeded','failed': 'VisibilityAdjustment'}, remapping={ 'rm': 'rm'})
        # smach.StateMachine.add('PreciseLanding', sm_precise_landing,transitions={'succeeded': 'Finish'})

        # smach.StateMachine.add('Idle', Idle(),transitions={'succeeded': MarkerSearch})

        smach.StateMachine.add('Finish', Finish(),transitions={})

        sis = smach_ros.IntrospectionServer('manipulation_smach_server', sm_top, '/SM_ROOT')
        sis.start()

    outcome = sm_top.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
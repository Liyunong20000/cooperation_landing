#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import numpy as np
import time
import tf.transformations as tft
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray
from gazebo_msgs.msg import ModelState

from basic_function import *
from dog_basic_function import *
from drone_basic_function import *
from gripper.gripper_move import *
from AprillandQilin import *

class TargetSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_docking', 'to_detaching', 'finished'], input_keys=['rm'])
        self.dog_basic = DogBasic()
        self.drone_basic = DroneBasic()
        self.rm = 0
        # Add necessary attributes for target search logic
        self.msg_apriltag = None
        self.find_drone_tag = 0
        self.april_drone_z = 0.0
        self.land_offset = 0.4  # Example offset
        self.timeout_s = 60.0  # Timeout for scanning

        # Subscribers
        if self.rm == 1:
            rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)
        else:
            rospy.Subscriber('/xuanwu/tag_detections', AprilTagDetectionArray, self._callback_simulation_apriltag)

        # Publishers for simulation or real
        if self.rm == 0:
            self.pub_sim_pose = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        else:
            self.pub_drone_target = rospy.Publisher('/quadrotor/target_pose', PoseStamped, queue_size=10)

    def _callback_apriltag(self, data):
        if data.detections:
            for det in data.detections:
                if 0 in det.id:  # Assuming drone tag id is 0
                    self.april_drone_z = det.pose.pose.pose.position.z
                    self.find_drone_tag = 1
                    break
        else:
            self.find_drone_tag = 0

    def _callback_simulation_apriltag(self, msg):
        self.msg_apriltag = msg
        # Similar logic for simulation

    def scanning(self, x, y, z, c, rm):
        # Horizontal scanning with vertical options
        # c is offset for scanning points
        if rm == 1:
            # Real machine: send drone target poses
            self.drone_target_pose(x + c, y, z, 0, 0, 0, 1)
            time.sleep(3.5)
            if self.find_drone_tag == 1:
                return True
            self.drone_target_pose(x, y + c, z, 0, 0, 0, 1)
            time.sleep(3.5)
            if self.find_drone_tag == 1:
                return True
            self.drone_target_pose(x - c, y, z, 0, 0, 0, 1)
            time.sleep(3.5)
            if self.find_drone_tag == 1:
                return True
            self.drone_target_pose(x, y - c, z, 0, 0, 0, 1)
            time.sleep(3.5)
            if self.find_drone_tag == 1:
                return True
        else:
            # Simulation: set model state
            self.sim_pose(x + c, y, 0, 0, 0, 1)
            time.sleep(1.5)
            if self.find_drone_tag == 1:
                return True
            # Add vertical scanning if needed, e.g., different z
            # For simplicity, horizontal only
        return False

    def drone_target_pose(self, x, y, z, ox, oy, oz, ow):
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = ox
        pose.pose.orientation.y = oy
        pose.pose.orientation.z = oz
        pose.pose.orientation.w = ow
        self.pub_drone_target.publish(pose)
        time.sleep(0.5)

    def sim_pose(self, px, py, ox, oy, oz, ow):
        pose = ModelState()
        pose.model_name = 'unitree'  # Adjust model name
        pose.pose.position.x = px
        pose.pose.position.y = py
        pose.pose.orientation.x = ox
        pose.pose.orientation.y = oy
        pose.pose.orientation.z = oz
        pose.pose.orientation.w = ow
        pose.reference_frame = 'world'
        self.pub_sim_pose.publish(pose)

    def execute(self, userdata):
        self.rm = userdata.rm
        # Get current position
        if self.rm == 1:
            takeoff_x = self.drone_basic.drone_x
            takeoff_y = self.drone_basic.drone_y
            takeoff_z = self.drone_basic.drone_z + self.land_offset
        else:
            # Simulation position
            takeoff_x, takeoff_y, takeoff_z = 0.0, 0.0, 1.0  # Placeholder

        start_t = rospy.Time.now()
        rate = rospy.Rate(1.0)

        while not rospy.is_shutdown():
            # Timeout
            if (rospy.Time.now() - start_t).to_sec() > self.timeout_s:
                return 'finished'

            # Perform scanning
            if self.scanning(takeoff_x, takeoff_y, takeoff_z, 0.2, self.rm):
                # Marker found, judge height
                if self.april_drone_z < 1.0:  # Example threshold for docking (low height)
                    return 'to_docking'
                else:  # High height for detaching
                    return 'to_detaching'

            # If not found, continue or adjust scanning
            rate.sleep()

        return 'finished'

class DockingManipulation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['back_to_search', 'finished'], input_keys=['rm'])
        self.gripper_move = GripperMoveNode()
        self.dog_basic = DogBasic()
        self.drone_basic = DroneBasic()
        self.rm = 0

    def execute(self, userdata):
        self.rm = userdata.rm
        # Implement docking manipulation logic (e.g., pick and place)
        # If need to search again, return 'back_to_search'
        # Else, 'finished'
        return 'finished'

class DetachingManipulation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_precise_landing'], input_keys=['rm'])
        self.drone_basic = DroneBasic()
        self.gripper_move = GripperMoveNode()
        self.rm = 0

    def execute(self, userdata):
        self.rm = userdata.rm
        # Implement detaching manipulation logic
        # Must go to precise landing
        return 'to_precise_landing'

class PreciseLanding(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['back_to_search', 'finished'], input_keys=['rm'])
        self.drone_basic = DroneBasic()
        self.dog_basic = DogBasic()
        self.alm = AprillandqilinNode()
        self.rm = 0

    def execute(self, userdata):
        self.rm = userdata.rm
        # Implement precise landing logic
        # Then decide to go back to search or finish
        return 'finished'

class Finish(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted'])

    def execute(self, userdata):
        return 'preempted'

class HorizontalScan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['scanned'], input_keys=['rm'], output_keys=['scan_results'])
        self.dog_basic = DogBasic()
        self.rm = 0
        self.delta_psi = 30  # Example, calculate based on FOV
        self.N_h = 12  # 360 / 30
        self.Psi = [k * self.delta_psi for k in range(self.N_h)]

    def execute(self, userdata):
        self.rm = userdata.rm
        # Perform horizontal scanning: rotate to each psi in Psi
        for psi in self.Psi:
            if self.rm == 1:
                self.dog_basic.qilin_cmd_vel(0, 0, 0, 0, psi)  # Rotate yaw
                time.sleep(1)  # Wait for rotation
            # Placeholder for vertical scan here if needed
        userdata.scan_results = {'horizontal_done': True}
        return 'scanned'

class VerticalScan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['scanned'], input_keys=['rm', 'scan_results'], output_keys=['scan_results'])
        self.dog_basic = DogBasic()
        self.rm = 0
        self.Theta = [-45, -30, 0, 30, 45]  # Degrees

    def execute(self, userdata):
        self.rm = userdata.rm
        # Perform vertical scanning: adjust pitch
        for theta in self.Theta:
            if self.rm == 1:
                # Adjust pitch, assuming method exists
                self.dog_basic.adjust_pitch(theta)
                time.sleep(1)
                # Detect here or in judgment
        userdata.scan_results['vertical_done'] = True
        return 'scanned'

class StateJudgment(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_docking', 'to_detaching', 'continue_scan', 'finished'], input_keys=['rm', 'scan_results', 'picking_marker', 'placing_marker'])
        self.find_marker = 0
        self.marker_id = -1
        # Add subscriber if needed

    def execute(self, userdata):
        self.rm = userdata.rm
        picking_id = userdata.picking_marker
        placing_id = userdata.placing_marker
        # Check if marker detected (assume from callback or shared data)
        if self.find_marker == 1:
            if self.marker_id == picking_id:
                return 'to_docking'
            elif self.marker_id == placing_id:
                return 'to_detaching'
        else:
            # If not all scanned, continue
            if not userdata.scan_results.get('all_done', False):
                return 'continue_scan'
            else:
                return 'finished'
#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import numpy as np
import time, math, threading
from aerial_robot_msgs.msg import FlightNav
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Empty, UInt8
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState

# It is for  the Coopration for Mini_Quadrotor and Qilin



class CooperationNode:

    def __init__(self):  # This part will work when this node is used.
        print(f'Hi, I am Cloud Cube')
        rospy.init_node('Cooperation', anonymous=True)

        self.event = threading.Event()
        self.lx, self.ly, self.lz = 0, 0, 0
        self.qx, self.qy, self.qz, self.qw = 0, 0, 0, 0
        self.april_x, self.april_y, self.april_z = 0.0, 0.0, 0.0
        self.april_qx,self.april_qy, self.april_qz, self.april_qw = 0.0, 0.0, 0.0, 0.0

        self.camera2base_x = 0.27
        self.camera2base_y = 0
        self.camera2base_z = 0.137
        self.valve2tag_x, self.valve2tag_y = 1.0, 1.0
        self.valve_x, self.valve_y = 0, 0
        self.april_valve_x, self.april_valve_y, self.april_valve_z = 0, 0, 0

        self.dog_x, self.dog_y, self.dog_z = (2.8, 2.8, 0.0)
        self.drone_x, self.drone_y, self.drone_z = 0.0, 0.0, 0.0
        self.takeoff_x, self.takeoff_y, self.takeoff_z = 0.0, 0.0, 0.0

        self.data_array = 0
        self.find_valve_tag = 0
        self.D = 0
        self.time_rece = rospy.Time()
        self._seq = 0
        self.state = 0
        self.beginland = 0
        self.beginfollow = 0
        self.flag = 0

        # Subscribe and publish.
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)
        rospy.Subscriber('/quadrotor/uav/cog/odom', Odometry, self._callback_position)
        rospy.Subscriber('/quadrotor/flight_state', UInt8, self._callback_state)
        # rospy.Subscriber('/uavandgr/event', UInt8, self._callback_event)

        self.pub_drone_nav = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)
        self.pub_takeoff = rospy.Publisher('/quadrotor/teleop_command/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('/quadrotor/teleop_command/land', Empty, queue_size=10)

        self.pub_event = rospy.Publisher('/uavandgr/event', UInt8, queue_size=10)
        # simulation: unitree position

        self.pub_sim_pose = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

        self.pub_qilin_vel = rospy.Publisher('/go1/cmd_vel', Twist, queue_size=10)
        self.pub_qilin_pose = rospy.Publisher('/go1/body_pose', Pose, queue_size=10)
        # rospy.wait_for_service('/go1/sit')
        # rospy.wait_for_service('/go1/stand')
        # self.service_client_sit = rospy.ServiceProxy('/go1/sit', Trigger)
        # self.service_client_stand = rospy.ServiceProxy('/go1/stand', Trigger)

        rospy.set_param('/converge_interval', 0.05)
        self.converge_interval = rospy.get_param("/converge_interval")
        rospy.set_param('/above_z', 0.3)
        self.above_z = rospy.get_param("/above_z")

        # rospy.set_param('/move_parameter', 2)
        # self.move_parameter = rospy.get_param("/move_parameter")
        # rospy.set_param('/pose_parameter', 0.05)
        # self.pose_parameter = rospy.get_param("/pose_parameter")


    def _callback_apriltag(self, data):
        current_time = rospy.Time.now()
        # print(f'apriltag:{current_time.to_sec()}')

        # get the apriltag`s position information compare with camera coordination
        if data.detections:
            self.find_valve_tag = self.find_target_tag(data.detections, 1)

            #rospy.loginfo("latest arigtarg timestamp: {}".format(data.header.stamp.to_sec()))
            a = data.detections[0]
            self.april_uav_x = a.pose.pose.pose.position.x
            self.april_uav_y = a.pose.pose.pose.position.y
            self.april_uav_qx = a.pose.pose.pose.orientation.x
            self.april_uav_qy = a.pose.pose.pose.orientation.y
            self.april_uav_qz = a.pose.pose.pose.orientation.z
            self.april_uav_qw = a.pose.pose.pose.orientation.w

            # b = data.detections[1]
            # self.april_valve_x = b.pose.pose.pose.position.x
            # self.april_valve_y = b.pose.pose.pose.position.y
            # if len(data.detections) > 1 :
                # print(f'222222222222')
            # else:
            #     print(f'111111111111')


            if self.beginfollow == 1:
                self.lx = - 2 * self.april_y
                self.ly = 2 * self.april_x
                self.april_z = 0.05 * self.quaternion_to_euler_angle(self.april_qx, self.april_qy,
                                                                                    self.april_qz, self.april_qw)
                # self.qx = self.pose_parameter * self.april_qx
                # self.qy = self.pose_parameter * self.april_qy
                # self.qz = self.pose_parameter * self.april_qz
                # self.qw = self.pose_parameter * self.april_qw
                apriltag_time = rospy.Time.now()
                print("euler_z (degree):", self.lx, self.ly)
                # print(f'apriltag_time:{apriltag_time.to_sec()}')
                if abs(self.lx) < 5 and abs(self.ly) < 5:
                    navigation_time = rospy.Time.now()
                    print("enter")
                    # print(f'navigation_time:{navigation_time.to_sec()}')
                    self.qilin_cmd_vel(self.lx, self.ly, 0, 0, self.april_z)
                    # self.qilin_body_pose(self.qx, self.qy, self.qz, self.qw)
                    # self.qilin_body_pose(self.april_qx, self.april_qy, self.april_qz, self.april_qw)
        else:
            if self.beginfollow == 1:
                self.qilin_cmd_vel(0, 0, 0, 0, 0)

            # self.qilin_body_pose(0, 0, 0, 1)

    def _callback_position(self, odom_msg):
        self.drone_x = odom_msg.pose.pose.position.x
        self.drone_y = odom_msg.pose.pose.position.y
        self.drone_z = odom_msg.pose.pose.position.z

    def _callback_state(self, msg):
        self.state = msg.data

    def find_target_tag(self,data,target_id):
        y = len(data)
        x = 0

        while x < y:
            a = target_id in data[x].id
            print(f'{a}')
            if a:
                self.april_valve_x = -data[x].pose.pose.pose.position.y
                self.april_valve_y = data[x].pose.pose.pose.position.x
                self.april_valve_z = data[x].pose.pose.pose.position.z
                self.valve_x = self.april_valve_x + self.valve2tag_x
                self.valve_y = self.april_valve_y + self.valve2tag_y
                print(f'{self.april_valve_x},{self.april_valve_y}')
                return 1
            else:
                return 0


    # drone takeoff
    def takeoff(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing takeoff command...")
        empty_msg = Empty()
        self.pub_takeoff.publish(empty_msg)

    # drone land
    def land(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing land command...")
        empty_msg = Empty()
        self.pub_land.publish(empty_msg)

    def event(self, x):
        event_msgs = UInt8()
        event_msgs.data = x
        self.pub_event.publish(event_msgs)

    def drone_landing_detection(self, i):
        r = rospy.Rate(i)
        number = i
        while not rospy.is_shutdown():
            number = number - 1
            if math.sqrt(self.april_x ** 2 + self.april_y ** 2) < 0.03 and abs(self.april_z) < 10:
                i = i - 1
            if number == 0:
                break
            r.sleep()
        self.flag = i

    def drone_landing_condition(self):
        while not rospy.is_shutdown():
            i = 1
            plus = 0
            self.flag = 0

            while i > 0:
                i = i - 1
                self.drone_landing_detection(10)
                plus = plus + self.flag
                print(f'plus = {plus}')
            if plus == 0:
                self.beginfollow = 0
                self.event(3)
                self.qilin_cmd_vel(0, 0, 0, 0, 0)
                print(f'landon')

                break

    def sit(self):
        try:
            response = self.service_client_sit()
            if response.success:
                rospy.loginfo('Sit command executed successfully')
            else:
                rospy.logwarn('Stand command failed: %s', response.message)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)

    def stand(self):
        try:
            response = self.service_client_stand()
            if response.success:
                rospy.loginfo('Stand command executed successfully')
            else:
                rospy.logwarn('Stand command failed: %s', response.message)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)

    def sim_pose(self, px, py, ox, oy, oz, ow):

        sim_pose = ModelState()
        sim_pose.model_name = 'unitree'
        sim_pose.pose.position.x = px
        sim_pose.pose.position.y = py
        sim_pose.pose.orientation.x = ox
        sim_pose.pose.orientation.y = oy
        sim_pose.pose.orientation.z = oz
        sim_pose.pose.orientation.w = ow
        sim_pose.reference_frame = 'world'
        self.pub_sim_pose.publish(sim_pose)


    def drone_nav_info(self, x, y, z):
        flight_nav_msg = FlightNav()
        flight_nav_msg.header.seq = self._seq
        self._seq += 1
        flight_nav_msg.header.stamp = rospy.Time.now()
        flight_nav_msg.header.frame_id = 'world'
        flight_nav_msg.pos_xy_nav_mode = 2
        flight_nav_msg.target_pos_x = x
        flight_nav_msg.target_pos_y = y
        flight_nav_msg.pos_z_nav_mode = 2
        flight_nav_msg.target_pos_z = z
        self.pub_drone_nav.publish(flight_nav_msg)
    def qilin_cmd_vel(self, lx, ly, ax, ay, az):
        qilin_cmd_vel = Twist()
        qilin_cmd_vel.linear.x = lx
        qilin_cmd_vel.linear.y = ly
        qilin_cmd_vel.angular.x = ax
        qilin_cmd_vel.angular.y = ay
        qilin_cmd_vel.angular.z = az

        self.pub_qilin_vel.publish(qilin_cmd_vel)

    def qilin_body_pose(self, qx, qy, qz, qw):
        qilin_body_pose = Pose()
        qilin_body_pose.orientation.x = qx
        qilin_body_pose.orientation.y = qy
        qilin_body_pose.orientation.z = qz
        qilin_body_pose.orientation.w = qw

        self.pub_qilin_pose.publish(qilin_body_pose)

    def quaternion_to_euler_angle(self, x, y, z, w):
        R = np.array([[1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y],
                      [2 * x * y + 2 * w * z, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * w * x],
                      [2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x ** 2 - 2 * y ** 2]])
        # theta_x = math.degrees(np.arctan2(R[2, 1], R[2, 2]))
        # theta_y = math.degrees(np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2)))
        theta_z = math.degrees(np.arctan2(R[1, 0], R[0, 0]))
        return theta_z

    def come_back(self):

        while not rospy.is_shutdown():
            if self.state == 5:
                break
            time.sleep(0.1)
        self.event(2)
        print(f'Move to above takeoff_Z')
        # self.converge(self.takeoff_x, self.takeoff_y, tz)
        time.sleep(7)
        self.beginfollow = 1
        print(f'begin follow')

    def tag_detection(self, step, scope):
        while self.dog_x < scope:
            while self.dog_y < scope:
                self.dog_y += step
                self.sim_pose(self.dog_x, self.dog_y, 0, 0, 0, 1)
                self.tag_detection_trigger()
                time.sleep(1)
            self.dog_x += step
            time.sleep(0.5)
            self.sim_pose(self.dog_x, self.dog_y, 0, 0, 0, 1)
            self.tag_detection_trigger()
            while self.dog_y > 0:
                self.dog_y -= step
                self.sim_pose(self.dog_x, self.dog_y, 0, 0, 0, 1)
                self.tag_detection_trigger()
                time.sleep(1)
            self.dog_x += step
            self.sim_pose(self.dog_x, self.dog_y, 0, 0, 0, 1)
            self.tag_detection_trigger()
        while self.dog_y < scope:
            self.dog_y += step
            self.sim_pose(self.dog_x, self.dog_y, 0, 0, 0, 1)
            self.tag_detection_trigger()
            time.sleep(1)
        self.dog_x = 0
        self.dog_y = 0
        self.sim_pose(self.dog_x, self.dog_y, 0, 0, 0, 1)
        self.tag_detection_trigger()

    def tag_detection_trigger(self):
        if self.find_valve_tag == 1:
            print(f'find it, gank!')
            self.event.set()
            return

    def tag_detection_gank(self):
        # self.event.wait()
        if self.find_valve_tag == 1:
            time.sleep(2)
            print(f'{self.april_valve_x},{self.april_valve_y}')
            while abs(self.april_valve_x) > 0.1 and abs(self.april_valve_y) > 0.1:
                print(f'11111111111111111111111111111')
                print(f'{self.dog_x + self.april_valve_x + self.camera2base_x}')
                self.sim_pose(self.dog_x + self.april_valve_x + self.camera2base_x,
                              self.dog_y + self.april_valve_y + self.camera2base_y, 0, 0, 0, 1)
                self.dog_x=self.dog_x + self.april_valve_x + self.camera2base_x
                self.dog_y=self.dog_y + self.april_valve_y + self.camera2base_y
                print(f'finish')
            print(
                f'{self.dog_x + self.april_valve_x + self.camera2base_x + self.valve2tag_x},{self.dog_y + self.april_valve_y + self.camera2base_y + self.valve2tag_y},{self.april_valve_z}')
            self.drone_nav_info(self.dog_x + self.april_valve_x + self.camera2base_x + self.valve2tag_x, self.dog_y +
                                self.april_valve_y + self.camera2base_y + self.valve2tag_y, self.april_valve_z)


    def sim(self):
        self.takeoff()
        while not rospy.is_shutdown():
            if self.state == 5:
                break
            time.sleep(0.1)
        self.sim_pose(self.dog_x, self.dog_y, 0, 0, 0, 1)
        self.tag_detection_gank()







if __name__ == '__main__':
    node = CooperationNode()
    # node.stand()
    time.sleep(1)
    node.sim()

    while not rospy.is_shutdown():
        rospy.spin()

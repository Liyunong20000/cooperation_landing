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

        # self.event = threading.Event()
        self.lx, self.ly, self.lz = 0, 0, 0
        self.qx, self.qy, self.qz, self.qw = 0, 0, 0, 0
        self.tag_target_x, self.tag_target_y, self.tag_target_z, self.tag_target_yaw = 0, 0, 0, 0
        self.april_x, self.april_y, self.april_z = 0.0, 0.0, 0.0
        self.april_qx,self.april_qy, self.april_qz, self.april_qw = 0.0, 0.0, 0.0, 0.0

        self.camera2base_x = 0.27
        self.camera2base_y = 0
        self.camera2base_z = 0.137
        self.valve2tag_x, self.valve2tag_y = 1.0, 1.0
        self.valve_x, self.valve_y = 0, 0
        self.april_valve_x, self.april_valve_y, self.april_valve_z, self.april_valve_yaw = 0, 0, 0, 0
        self.april_drone_x, self.april_drone_y, self.april_drone_z, self.april_drone_yaw = 0, 0, 0, 0

        self.dog_x, self.dog_y, self.dog_z = (2.8, 2.8, 0.0)
        self.drone_x, self.drone_y, self.drone_z, self.drone_roll, self.drone_pitch, self.drone_yaw = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        self.drone_ori_x, self.drone_ori_y, self.drone_ori_z, self.drone_ori_w = 0.0, 0.0, 0.0, 0.0
        self.takeoff_x, self.takeoff_y, self.takeoff_z, self.takeoff_yaw = 0.0, 0.0, 0.0, 0.0

        self.miss_first_time, self.miss_second_time = rospy.Time.now(), rospy.Time.now()
        self.data_array = 0
        self.find_valve_tag = 0
        self.find_drone_tag = 0
        self.D = 0
        self.time_rece = rospy.Time()
        self._seq = 0
        self.state = 0
        self.beginland = 0
        self.beginfollow = 0
        self.flag = 0

        self.user_input = None

        self.qilin_odom_x, self.qilin_odom_y, self.qilin_odom_th = 0, 0, 0
        self.last_time = None
        # Subscribe and publish.
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._callback_apriltag)
        rospy.Subscriber('/quadrotor/uav/cog/odom', Odometry, self._callback_position)
        rospy.Subscriber('/quadrotor/flight_state', UInt8, self._callback_state)
        rospy.Subscriber('/odom', Odometry, self._callback_qilin_odom)

        self.pub_drone_nav = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)
        self.pub_takeoff = rospy.Publisher('/quadrotor/teleop_command/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('/quadrotor/teleop_command/land', Empty, queue_size=10)
        self.pub_drone_nav_info = rospy.Publisher('/quadrotor/uav/nav/info', FlightNav, queue_size=10)
        self.pub_drone_nav_trigger = rospy.Publisher('/quadrotor/uav/nav/trigger', Empty, queue_size=10)
        # self.pub_event = rospy.Publisher('/uavandgr/event', UInt8, queue_size=10)
        # self.pub_UavAndGr_Uav_Nav = rospy.Publisher('/uavandgr/uav_nav_info', Pose, queue_size=10)
        # simulation: unitree position

        # self.pub_sim_pose = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

        self.pub_qilin_vel = rospy.Publisher('/go1/cmd_vel', Twist, queue_size=10)
        self.pub_qilin_pose = rospy.Publisher('/go1/body_pose', Pose, queue_size=10)
        rospy.wait_for_service('/go1/sit')
        rospy.wait_for_service('/go1/stand')
        self.service_client_sit = rospy.ServiceProxy('/go1/sit', Trigger)
        self.service_client_stand = rospy.ServiceProxy('/go1/stand', Trigger)

        rospy.set_param('/converge_interval', 0.05)
        self.converge_interval = rospy.get_param("/converge_interval")
        rospy.set_param('/above_z', 0.3)
        self.above_z = rospy.get_param("/above_z")

        rospy.set_param('/move_parameter', 2)
        self.move_parameter = rospy.get_param("/move_parameter")
        rospy.set_param('/pose_parameter', 0.05)
        self.pose_parameter = rospy.get_param("/pose_parameter")


    def _callback_apriltag(self, data):
        current_time = rospy.Time.now()
        # print(f'apriltag:{current_time.to_sec()}')

        # get the apriltag`s position information compare with camera coordination
        if data.detections:
            self.find_valve_tag = self.find_target_tag(data.detections,1)
            self.find_drone_tag = self.find_target_tag(data.detections, 0)
            if self.beginfollow == 1:
                if self.find_drone_tag == 1:
                    self.align_dog_with_drone()
                else:
                    self.qilin_cmd_vel(0, 0, 0, 0, 0)

        else:
            if self.beginfollow == 1:
                self.miss_second_time = rospy.Time.now()
                duration = self.miss_second_time - self.miss_first_time
                # print(f'{duration.to_sec()}')
                if duration.to_sec() > 0.2:
                    self.qilin_cmd_vel(0, 0, 0, 0, 0)
                    self.miss_first_time = self.miss_second_time
                else:
                    self.miss_first_time = self.miss_second_time
        #     if self.beginfollow == 1:
        #         self.lx = - 2 * self.april_y
        #         self.ly = 2 * self.april_x
        #         self.april_z = 0.05 * self.quaternion_to_euler_angle(self.april_qx, self.april_qy,
        #                                                                             self.april_qz, self.april_qw)
        #         # self.qx = self.pose_parameter * self.april_qx
        #         # self.qy = self.pose_parameter * self.april_qy
        #         # self.qz = self.pose_parameter * self.april_qz
        #         # self.qw = self.pose_parameter * self.april_qw
        #         apriltag_time = rospy.Time.now()
        #         print("euler_z (degree):", self.lx, self.ly)
        #         # print(f'apriltag_time:{apriltag_time.to_sec()}')
        #         if abs(self.lx) < 5 and abs(self.ly) < 5:
        #             navigation_time = rospy.Time.now()
        #             print("enter")
        #             # print(f'navigation_time:{navigation_time.to_sec()}')
        #             self.qilin_cmd_vel(self.lx, self.ly, 0, 0, self.april_z)
        #             # self.qilin_body_pose(self.qx, self.qy, self.qz, self.qw)
        #             # self.qilin_body_pose(self.april_qx, self.april_qy, self.april_qz, self.april_qw)
        # else:
        #     if self.beginfollow == 1:
        #         self.qilin_cmd_vel(0, 0, 0, 0, 0)
        #
        #     # self.qilin_body_pose(0, 0, 0, 1)

    def _callback_position(self, odom_msg):
        self.drone_x = odom_msg.pose.pose.position.x
        self.drone_y = odom_msg.pose.pose.position.y
        self.drone_z = odom_msg.pose.pose.position.z
        self.drone_ori_x = odom_msg.pose.pose.orientation.x
        self.drone_ori_y = odom_msg.pose.pose.orientation.y
        self.drone_ori_z = odom_msg.pose.pose.orientation.z
        self.drone_ori_w = odom_msg.pose.pose.orientation.w
        self.drone_roll = self.quaternion_to_euler_angle(self.drone_ori_x, self.drone_ori_y, self.drone_ori_z, self.drone_ori_w)[0]
        self.drone_pitch = self.quaternion_to_euler_angle(self.drone_ori_x, self.drone_ori_y, self.drone_ori_z, self.drone_ori_w)[1]
        self.drone_yaw = self.quaternion_to_euler_angle(self.drone_ori_x, self.drone_ori_y, self.drone_ori_z, self.drone_ori_w)[2]

        # print(f'drone_yaw:{self.drone_roll}, {self.drone_pitch},{self.drone_yaw}')
        if self.drone_z <-0.5 or self.drone_z > 3 or abs(self.drone_roll) > 45 or abs(self.drone_pitch) > 45:
            rospy.loginfo("Wrong state! land!")
            self.land()
    def _callback_qilin_odom(self, msg):
        self.qilin_odom_x = msg.pose.pose.position.x
        self.qilin_odom_y = msg.pose.pose.position.y
        self.qilin_odom_z = msg.pose.pose.position.z

    def _callback_state(self, msg):
        self.state = msg.data

    def find_target_tag(self,data,target_id):
        b = len(data)
        a = 0

        while a < b:
            c = target_id in data[a].id
            # print(f'{a}')
            if c:
                if target_id == 0:
                    self.april_drone_x = -data[a].pose.pose.pose.position.y
                    self.april_drone_y = data[a].pose.pose.pose.position.x
                    self.april_drone_z = data[a].pose.pose.pose.position.z
                    self.april_drone_yaw = self.quaternion_to_euler_angle(data[a].pose.pose.pose.orientation.x,
                                                                          data[a].pose.pose.pose.orientation.y,
                                                                          data[a].pose.pose.pose.orientation.z,
                                                                          data[a].pose.pose.pose.orientation.w)[2]

                if target_id == 1:
                    self.april_valve_x = -data[a].pose.pose.pose.position.y
                    self.april_valve_y = data[a].pose.pose.pose.position.x
                    self.april_valve_z = data[a].pose.pose.pose.position.z
                    self.april_valve_yaw = self.quaternion_to_euler_angle(data[a].pose.pose.pose.orientation.x,
                                                         data[a].pose.pose.pose.orientation.y,
                                                         data[a].pose.pose.pose.orientation.z,
                                                         data[a].pose.pose.pose.orientation.w)[2]

                return 1
            else:
                return 0


    # drone takeoff
    def takeoff(self):
        time.sleep(0.5)
        if (-10 < self.drone_x <10) and (-10 < self.drone_y <10) and (-0.5 < self.drone_z <3):
            rospy.loginfo("Publishing takeoff command...")
            empty_msg = Empty()
            self.pub_takeoff.publish(empty_msg)
        else:
            rospy.loginfo("Don`t takeoff, state is wrong!!!")


    # drone land
    def land(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing land command...")
        empty_msg = Empty()
        self.pub_land.publish(empty_msg)

    def drone_nav_trigger(self):
        time.sleep(0.5)
        rospy.loginfo("Publishing nav trigger command...")
        empty_msg = Empty()
        self.pub_drone_nav_trigger.publish(empty_msg)

    def drone_landing_detection(self, i):
        r = rospy.Rate(i)
        number = i
        while not rospy.is_shutdown():
            number = number - 1
            if math.sqrt(self.april_drone_x ** 2 + self.april_drone_y ** 2) < 0.03 and abs(self.april_drone_yaw) < 10:
                i = i - 1
            if number == 0:
                break
            r.sleep()
        self.flag = i

    def drone_landing_condition(self):
        while not rospy.is_shutdown():
            if self.april_drone_x != 0:
                break
            time.sleep(0.1)
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
                self.qilin_cmd_vel(0, 0, 0, 0, 0)
                self.land()
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


    def drone_nav_info(self, x, y, z, yaw_mode, omega_z, yaw):
        flight_nav_msg = FlightNav()
        flight_nav_msg.header.seq = self._seq
        self._seq += 1
        flight_nav_msg.header.stamp = rospy.Time.now()
        flight_nav_msg.header.frame_id = 'world'

        flight_nav_msg.control_frame = 0
        flight_nav_msg.target = 0
        flight_nav_msg.pos_xy_nav_mode = 2
        flight_nav_msg.target_pos_x = x
        flight_nav_msg.target_vel_x = 0.0
        flight_nav_msg.target_acc_x = 0.0
        flight_nav_msg.target_pos_y = y
        flight_nav_msg.target_vel_y = 0.0
        flight_nav_msg.target_acc_y = 0.0
        flight_nav_msg.yaw_nav_mode = yaw_mode
        flight_nav_msg.target_omega_z = omega_z
        flight_nav_msg.target_yaw = yaw
        flight_nav_msg.pos_z_nav_mode = 2
        flight_nav_msg.target_pos_z = z
        flight_nav_msg.target_vel_z = 0.0
        flight_nav_msg.target_pos_diff_z = 0.0

        self.pub_drone_nav_info.publish(flight_nav_msg)
        time.sleep(1)
        self.drone_nav_trigger()

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
        theta_x = math.degrees(np.arctan2(R[2, 1], R[2, 2]))
        theta_y = math.degrees(np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2)))
        theta_z = math.degrees(np.arctan2(R[1, 0], R[0, 0]))
        theta = np.array([theta_x, theta_y, theta_z])
        return theta

    def record_takeoff_position(self):
        self.takeoff_x = self.drone_x
        self.takeoff_y = self.drone_y
        self.takeoff_z = self.drone_z
        self.takeoff_yaw = self.drone_yaw
        print(f'takeoff position:{self.takeoff_x}, {self.takeoff_y},{self.takeoff_z}, {self.takeoff_yaw}')

    def tag_detection_gank(self):
        # self.event.wait()
        time.sleep(2)
        # print(f'gankgankgankgank')
        print(f'{self.april_valve_x},{self.april_valve_y},{self.april_valve_z}')
        while ((abs(self.april_valve_x) > 0.1 or abs(self.april_valve_y) > 0.1 or abs(self.april_valve_yaw) > 1) and
               (self.find_valve_tag == 1)):
            print(f'11111111111111111111111111111')
            self.qilin_cmd_vel(0.7 * self.april_valve_x, 0.7 * self.april_valve_y, 0, 0, 0.03 * self.april_valve_yaw)
            time.sleep(1)
        self.qilin_cmd_vel(0, 0, 0, 0, 0)

    def manipulation(self,x,y,z,h,omega):
        time.sleep(2)
        self.drone_nav_info(x, y, z, 0, 0)
        time.sleep(5)
        self.drone_nav_info(x, y, z+h, 0, 0)
        time.sleep(5)
        self.drone_nav_info(x, y, z+h, omega, 1.57)
        time.sleep(3)
        self.drone_nav_info(x, y, z+h, omega, 3.14)
        time.sleep(3)
        self.drone_nav_info(x, y, z+h, omega, 4.71)
        time.sleep(3)
        self.drone_nav_info(x, y, z+h, omega, 6.28)
        time.sleep(2)
        self.drone_nav_info(x, y, z + h, omega, 1.57)
        time.sleep(3)
        self.drone_nav_info(x, y, z + h, omega, 3.14)
        time.sleep(3)
        self.drone_nav_info(x, y, z, 0, 3.14)
    def inspection(self,x,y,drift,z,omega,yaw):
        self.drone_nav_info(x, y+drift, z, 0,0, 0)
        time.sleep(2)
        self.drone_nav_info(x, y+drift, z, 4,omega, 90)
        time.sleep(2)
        self.drone_nav_info(x, y + drift, z, 4, omega, 180)
        time.sleep(2)
        self.drone_nav_info(x, y + drift, z, 4, omega, -90)
        time.sleep(2)
        self.drone_nav_info(x, y + drift, z, 4, omega, 0)
        time.sleep(2)
        self.drone_nav_info(x, y + drift, z, 4, omega, 90)
        time.sleep(2)
        self.drone_nav_info(x, y + drift, z, 4, omega,yaw+180)
        time.sleep(3)

    def align_dog_with_drone(self):
        self.lx = 1.2 * self.april_drone_x
        self.ly = 1.2 * self.april_drone_y
        self.april_z = 0.03 *self.april_drone_yaw
        if self.april_z > 0.3:
            self.april_z = 0.3
        if self.april_z < -0.3:
            self.april_z = -0.3
        # print("align speed lx, ly, :", self.lx, self.ly, self.april_z)
        # print(f'apriltag_time:{apriltag_time.to_sec()}')
        if abs(self.lx) < 2 and abs(self.ly) < 2:
            navigation_time = rospy.Time.now()
            # print(f'navigation_time:{navigation_time.to_sec()}')
            self.qilin_cmd_vel(self.lx, self.ly, 0, 0, self.april_z)



    def demo(self):
        while not self.find_valve_tag:
            self.qilin_cmd_vel(0.2, 0, 0, 0, 0)
        time.sleep(2)
        self.qilin_cmd_vel(0, 0, 0, 0, 0)
        self.record_takeoff_position()
        time.sleep(1)
        print(f'takeoff?')
        self.user_input = input()
        if self.user_input == 'y':
            print(f'yes')
            self.takeoff()

        while not rospy.is_shutdown():
            if self.state == 5:
                break
            time.sleep(0.1)

        self.inspection(self.takeoff_x+self.april_valve_x, self.takeoff_y-self.april_valve_y, -1.5, self.april_valve_z - 1,0.2,self.takeoff_yaw)
        self.drone_nav_info(self.takeoff_x,self.takeoff_y,self.april_valve_z - 1,0,0,0)
        time.sleep(3)
        self.drone_nav_info(self.takeoff_x, self.takeoff_y, self.takeoff_z + 0.5, 0, 0,0)
        self.beginfollow = 1
        self.drone_landing_condition()
    def demo2(self):
        self.beginfollow = 1

if __name__ == '__main__':
    node = CooperationNode()
    time.sleep(1)
    node.stand()
    time.sleep(2)
    # node.work()
    print(f"11111111111")
    node.demo()
    while not rospy.is_shutdown():
        rospy.spin()

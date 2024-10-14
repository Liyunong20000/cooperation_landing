#!/usr/bin/env python

from __future__ import print_function # for print function in python2
import sys, select, termios, tty

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import rosgraph
from std_srvs.srv import Trigger



msg = """
Instruction:

---------------------------

     q           w           e      
(turn left)  (forward)  (turn right)

     a           s           d          j       k       l
(move left)  (backward) (move right)  (stop) (stand)  (sit)


Please don't have caps lock on.
CTRL+c to quit
---------------------------
"""

def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def printMsg(msg, msg_len = 50):
        print(msg.ljust(msg_len) + "\r", end="")
def service_client_stand():
    try:
        service_client_stand = rospy.ServiceProxy('/go1/stand', Trigger)
        response = service_client_stand()
        return response
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
def service_client_sit():
    try:
        service_client_sit = rospy.ServiceProxy('/go1/sit', Trigger)
        response = service_client_sit()
        return response
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")




if __name__=="__main__":
        settings = termios.tcgetattr(sys.stdin)
        rospy.init_node("keyboard_control")

        # robot_ns = rospy.get_param("~robot_ns", "");
        print(msg)

        # if not robot_ns:
        #         master = rosgraph.Master('/rostopic')
        #         try:
        #                 _, subs, _ = master.getSystemState()
        #
        #         except socket.error:
        #                 raise ROSTopicIOException("Unable to communicate with master!")
        #
        #         teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]
        #         if len(teleop_topics) == 1:
        #                 robot_ns = teleop_topics[0].split('/teleop')[0]

        # ns = robot_ns + "/teleop_command"
        # land_pub = rospy.Publisher(ns + '/land', Empty, queue_size=1)
        # halt_pub = rospy.Publisher(ns + '/halt', Empty, queue_size=1)
        # start_pub = rospy.Publisher(ns + '/start', Empty, queue_size=1)
        # takeoff_pub = rospy.Publisher(ns + '/takeoff', Empty, queue_size=1)
        # force_landing_pub = rospy.Publisher(ns + '/force_landing', Empty, queue_size=1)
        nav_pub = rospy.Publisher('/go1/cmd_vel', Twist, queue_size=1)

        xy_vel   = rospy.get_param("xy_vel", 0.2)
        yaw_vel  = rospy.get_param("yaw_vel", 0.4)

        # motion_start_pub = rospy.Publisher('task_start', Empty, queue_size=1)

        try:
                while(True):
                        nav_msg = Twist()
                        # nav_msg.control_frame = FlightNav.WORLD_FRAME
                        # nav_msg.target = FlightNav.COG

                        key = getKey()

                        msg = ""

                        if key == 'w':
                                nav_msg.linear.x = xy_vel
                                nav_pub.publish(nav_msg)
                                msg = "send +x vel command"
                        if key == 's':
                                nav_msg.linear.x = -xy_vel
                                nav_pub.publish(nav_msg)
                                msg = "send -x vel command"
                        if key == 'a':
                                nav_msg.linear.y = xy_vel
                                nav_pub.publish(nav_msg)
                                msg = "send +y vel command"
                        if key == 'd':
                                nav_msg.linear.y = -xy_vel
                                nav_pub.publish(nav_msg)
                                msg = "send -y vel command"
                        if key == 'q':
                                nav_msg.angular.z = yaw_vel
                                nav_pub.publish(nav_msg)
                                msg = "send +yaw vel command"
                        if key == 'e':
                                nav_msg.angular.z = -yaw_vel
                                msg = "send -yaw vel command"
                                nav_pub.publish(nav_msg)
                        if key == 'j':
                                nav_msg.linear.x = 0
                                nav_msg.linear.y = 0
                                nav_msg.angular.z = 0
                                msg = "stop!"
                                nav_pub.publish(nav_msg)
                        if key == 'k':
                                response = service_client_stand()
                                if response.success:
                                        rospy.loginfo("Stand command sent successfully!")
                                else:
                                        rospy.logwarn("Failed to send stand command.")
                                msg = "stand!"
                        if key == 'l':
                                response = service_client_sit()
                                if response.success:
                                        rospy.loginfo("Sit command sent successfully!")
                                else:
                                        rospy.logwarn("Failed to send sit command.")
                                msg = "sit!"

                        if key == '\x03':
                                break

                        printMsg(msg)
                        rospy.sleep(0.001)

        except Exception as e:
                print(repr(e))
        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)



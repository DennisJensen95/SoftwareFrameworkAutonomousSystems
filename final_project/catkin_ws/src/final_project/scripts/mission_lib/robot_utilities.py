#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import threading


class BurgerUtility():
    """
        Robot Utility box specialized for final project in Software Autonomous systems     
    """

    def __init__(self):
        """
            Initialize Robot Class
        """
        self.robot_pos = None

        rospy.init_node('robot_position', anonymous=True)
        rospy.Subscriber("/odom",
                         Odometry, self.callback_robot_pose)

        self.main_thread = threading.Thread(target=self.main).start()

    def callback_robot_pose(self, payload):
        self.robot_pos = payload

    def main(self):
        rospy.spin()

    def get_robot_x_y_position(self):
        if self.robot_pos != None:
            return (self.robot_pos.pose.pose.position.x, self.robot_pos.pose.pose.position.y, self.robot_pos.pose.pose.position.z)
        else:
            return None

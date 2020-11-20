#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import PoseStamped


class MissionPlanning():

    def __init__(self, Burger, QrCodeUtil, Transform):
        """[summary]
        Initialize all the class variables
        """
        # Logging
        self.log_tag = "[MissionPlanning]: "

        # Utility classes
        self.burger = Burger
        self.qr_code_util = QrCodeUtil
        self.transform = Transform

    def log(self, msg):
        """[summary]
        Generic log message
        Args:
            msg ([str]): [Log message]
        """
        print(self.log_tag + str(msg))

    def drive_to_qr_code(self, qr_code_pos, dist_from=1.3):
        qr_code_pos = self.transform.transform_qr_code_to_desired_pos(
            qr_code_pos, self.burger.robot_imu_pos, dist_from=dist_from)

        if isinstance(qr_code_pos, bool):
            return False

        qr_code_pos.pose.orientation = self.burger.saved_robot_pos.pose.pose.orientation

        goal_pose = self.burger.get_goal_pose(qr_code_pos)
        if self.burger.move_to_pose(goal_pose, rospy.Duration()):
            self.log("Succesfull moved to target position")
            if self.qr_code_util.qr_code_detected:
                self.qr_code_util.save_code_message(qr_code_pos.pose.position)
                return True

        self.log("Did not move to goal position")
        return False

    def check_if_qr_code(self, new):
        """[summary]

        Args:
            new ([bool]): [Get a new QR code]

        Returns:
            [bool]: [If found new QR code state]
        """
        if self.qr_code_util.is_qr_code_detected() and self.qr_code_util.qr_code_position != None:
            if new and not self.qr_code_util.is_new_qr_code():
                return False
            self.burger.stop_moving()
            self.burger.save_robot_pose()
            return True

        return False

    def find_qr_code(self):
        """[summary]
        Finding a QR code
        Args:
            new (bool, optional): [description]. Defaults to False.

        Returns:
            [type]: [description]
        """
        found_qr_code = False
        while not rospy.is_shutdown():
            found_qr_code = self.burger.drive_patrol()
            if found_qr_code:
                break

        self.log("Found QR Code orient robot again")
        self.burger.orient_to_saved_robot_pos()

        qr_code_pos = self.burger.read_qr_code(duration=0.5)

        self.drive_to_qr_code(qr_code_pos)

        return found_qr_code

    def find_qr_code_turn_around(self, new=True):
        self.burger.find_qr_code_turn_around(new)

    def get_qr_code_position_in_odom_frame(self):
        qr_code_pos = self.burger.read_qr_code(duration=1)

        # Transform the QR code position from /map to /odometry and in global /odemtry frame
        qr_code_pos_odom = self.transform.transform_qr_code_to_desired_pos(
            qr_code_pos, self.burger.robot_imu_pos, dist_from=0)

        # Save the code message
        if isinstance(qr_code_pos_odom, bool):
            return False

        return qr_code_pos_odom.pose.position

    def save_qr_code_message(self, qr_code_position):
        self.qr_code_util.save_code_message(qr_code_position)

    def drive_to_next_qr_code(self, next_x_y):
        """[summary]
        Drive to the next QR code from the last 
        """
        check_pos = PoseStamped()
        check_pos.header.frame_id = "hidden_frame"
        check_pos.pose.position.x = next_x_y[0]
        check_pos.pose.position.y = next_x_y[1]
        check_pos.pose.orientation = self.burger.robot_imu_pos.orientation

        desired_pose = self.transform.transform_pose_in_frames(
            check_pos, "/odom", "/hidden_frame").pose
        self.log("Going to next QR code hidden_frame: " + str(next_x_y) +
                 " Odom frame: " + str(desired_pose.position))

        if self.burger.move_to_pose_looking_for_qr_code(desired_pose, next_x_y):
            qr_code_pos = self.burger.read_qr_code(duration=0.2)
            self.drive_to_qr_code(qr_code_pos)

        return True

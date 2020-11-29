#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
import time


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
        """[summary]
        Driving to QR code for reading it.

        Args:
            qr_code_pos ([type]): [description]
            dist_from (float, optional): [description]. Defaults to 1.3.

        Returns:
            [type]: [description]
        """
        self.log("Trying to drive to QR code")
        if not self.qr_code_util.is_qr_code_detected():
            self.log("Did not see a QR code")
            return False

        self.log("There is a QR code drive to it")

        (qr_code_pos, odemetry_desired_pos) = self.transform.transform_qr_code_to_desired_pos(
            qr_code_pos, self.burger.robot_imu_pos, dist_from=dist_from)

        if isinstance(qr_code_pos, bool):
            return False

        # Turn to original orientation when detected qr code
        qr_code_pos.pose.orientation = self.burger.saved_robot_pos.pose.pose.orientation

        # Will be overwritten when moving closer to QR

        self.save_qr_code_message(qr_code_pos.pose.position)
        goal_pose = self.burger.get_goal_pose(odemetry_desired_pos)
        if self.burger.move_to_pose(goal_pose):
            self.log("Succesfull moved to target position")
            if self.qr_code_util.is_qr_code_detected():
                self.save_qr_code_message(qr_code_pos.pose.position)
                self.log("Driving to QR code returned True")
                return True
        self.log("Driving to QR code return False")
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
        self.log("Trying to find QR code")
        found_qr_code = False
        while not rospy.is_shutdown():
            found_qr_code = self.burger.drive_patrol()
            if found_qr_code:
                self.burger.stop_moving()
                break
            self.burger.increment_patrol_target()

        qr_code_pos = self.burger.read_qr_code(duration=1.5)

        if qr_code_pos == None:
            return False

        if not self.drive_to_qr_code(qr_code_pos):
            return False

        return found_qr_code

    def find_qr_code_turn_around(self, new=True):
        self.log("Trying to find QR code turning 360")
        self.burger.find_qr_code_turn_around(new)

    def get_qr_code_position_in_odom_frame(self):
        self.log("Get QR code in odometry position")
        qr_code_pos = self.burger.read_qr_code(duration=1)

        # Transform the QR code position from /map to /odometry and in global /odemtry frame
        (qr_code_pos_odom, _) = self.transform.transform_qr_code_to_desired_pos(
            qr_code_pos, self.burger.robot_imu_pos, dist_from=0)

        # Save the code message
        if isinstance(qr_code_pos_odom, bool) or qr_code_pos_odom == None:
            self.log("Failed getting QR code in odemtry position")
            return False

        return qr_code_pos_odom.pose.position

    def save_qr_code_message(self, qr_code_position):
        self.log("Trying to save QR code message with qr code position")
        self.qr_code_util.save_code_message(qr_code_position)

    def get_heading_quadrant(self, angle):
        # First quadrant
        if angle >= 0 and angle < math.pi/2:
            return (-1, -1)
        # Second quadrant
        elif angle >= math.pi/2 and angle < math.pi:
            return (1, -1)
        # Fourth quadrant
        elif angle < 0 and angle < -math.pi/2:
            return (1, -1)
        # Third quadrant
        elif angle >= -math.pi/2 and angle < -math.pi:
            return (1, 1)

        # Random choice
        return (np.random.choice([1, -1]), np.random.choice([1, -1]))

    def drive_around_qr_code(self, qr_code_position, next_x_y):
        self.log("Look for QR code hidden_frame: " + str(next_x_y) + " Odometry frame: (" + str(
            qr_code_position.position.x) + "," + str(qr_code_position.position.y) + ")")
        (x_robot, y_robot, _) = self.burger.get_robot_x_y_position()
        x_qr = qr_code_position.position.x
        y_qr = qr_code_position.position.y

        angle = math.atan2(y_qr - y_robot, x_qr - x_robot)

        placement_x_y = self.get_heading_quadrant(angle)

        dist_to_qr = np.random.uniform(1.25, 2.75)

        points_diff = [[dist_to_qr, 0], [0, dist_to_qr]]

        point_x_4 = qr_code_position.position.x + \
            placement_x_y[0] * points_diff[0][0]
        point_y_4 = qr_code_position.position.y + \
            placement_x_y[1] * points_diff[0][1]

        point_x_2 = qr_code_position.position.x + \
            placement_x_y[0] * points_diff[0][0]
        point_y_2 = qr_code_position.position.y + \
            placement_x_y[1] * points_diff[0][0] * 1/2

        point_x_3 = qr_code_position.position.x + \
            placement_x_y[0] * points_diff[1][1] * 1/2
        point_y_3 = qr_code_position.position.y + \
            placement_x_y[1] * points_diff[1][1]

        point_x_1 = qr_code_position.position.x + \
            placement_x_y[0] * points_diff[1][0]
        point_y_1 = qr_code_position.position.y + \
            placement_x_y[1] * points_diff[1][1]

        points = [[point_x_1, point_y_1], [point_x_2, point_y_2],
                  [point_x_3, point_y_3], [point_x_4, point_y_4]]

        go_to_pose = Pose()
        go_to_pose.position = qr_code_position.position
        go_to_pose.orientation = qr_code_position.orientation

        for i in range(len(points)):
            go_to_pose.position.x = points[i][0]
            go_to_pose.position.y = points[i][1]

            if self.burger.move_to_pose_looking_for_qr_code(go_to_pose, next_x_y=next_x_y):
                self.burger.save_robot_pose()
                self.burger.stop_moving()
                return True

            if self.burger.find_qr_code_turn_around(new=False, next_x_y=next_x_y):
                self.burger.save_robot_pose()
                self.burger.stop_moving()
                return True

        return False

    def drive_to_next_qr_code(self, next_x_y):
        """[summary]
        Drive to the next QR code from the last 
        """
        self.log("Drive to next qr code")
        check_pos = PoseStamped()
        check_pos.header.frame_id = "hidden_frame"
        check_pos.pose.position.x = next_x_y[0]
        check_pos.pose.position.y = next_x_y[1]
        check_pos.pose.orientation = self.burger.robot_imu_pos.orientation

        desired_pose = self.transform.transform_pose_in_frames(
            check_pos, "/odom", "/hidden_frame").pose
        self.log("Going to next QR code hidden_frame: " + str(next_x_y) +
                 " Odom frame: " + str(desired_pose.position))

        if self.drive_around_qr_code(desired_pose, next_x_y):
            self.burger.orient_to_saved_robot_pos()
            qr_code_pos = self.burger.read_qr_code(duration=1)
            if not self.drive_to_qr_code(qr_code_pos):
                return False
            return True

        return False

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
        """[summary]
        Driving to QR code for reading it.

        Args:
            qr_code_pos ([type]): [description]
            dist_from (float, optional): [description]. Defaults to 1.3.

        Returns:
            [type]: [description]
        """
        qr_code_pos = self.transform.transform_qr_code_to_desired_pos(
            qr_code_pos, self.burger.robot_imu_pos, dist_from=dist_from)

        if isinstance(qr_code_pos, bool):
            return False

        # Turn to original orientation when detected qr code
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
                self.burger.stop_moving()
                break
            self.burger.increment_patrol_target()

        qr_code_pos = self.burger.read_qr_code(duration=1.5)

        if qr_code_pos == None:
            return False

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
        if isinstance(qr_code_pos_odom, bool) or qr_code_pos_odom == None:
            return False

        return qr_code_pos_odom.pose.position

    def save_qr_code_message(self, qr_code_position):
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

        return (0, 0)

    def drive_around_qr_code(self, desired_pose_qr, next_x_y):
        self.log("Drive around qr code")
        (x_robot, y_robot, _) = self.burger.get_robot_x_y_position()
        x_qr = desired_pose_qr.position.x
        y_qr = desired_pose_qr.position.y
        angle = math.atan2(y_qr - y_robot, x_qr - x_robot)
        placement_x_y = self.get_heading_quadrant(angle)
        points_diff = [[1.5, 0], [0, 1.5]]

        for i in range(len(points_diff)):
            self.log(i)
            go_to_pose = desired_pose_qr
            go_to_pose.position.x = go_to_pose.position.x + \
                placement_x_y[0] * points_diff[i][0]
            go_to_pose.position.y = go_to_pose.position.y + \
                placement_x_y[1] * points_diff[i][1]

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
        self.log("Drive to next qr code")
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

        if self.drive_around_qr_code(desired_pose, next_x_y):
            self.burger.orient_to_saved_robot_pos()
            qr_code_pos = self.burger.read_qr_code(duration=1)
            self.drive_to_qr_code(qr_code_pos)
            return True

        return False

#!/usr/bin/env python
from math import pi
from time import sleep
import rospy
from nav_msgs.msg import Odometry
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
import math
import time
import numpy as np


class BurgerUtility():
    """[summary]
    Class for utilities in moving the robot 
    """

    def __init__(self, QrCodeUtil, Transform):
        """
            Initialize Robot Class
        """
        self.log_tag = "[Robot Utilities]: "

        # QR code utilities
        self.qr_code_util = QrCodeUtil
        self.transform = Transform

        # Robot position
        self.robot_pos = None
        self.saved_robot_pos = None
        self.robot_imu_pos = None
        self.saved_robot_imu_pos = None
        rospy.Subscriber("/odom",
                         Odometry, self.callback_robot_pose)
        rospy.Subscriber("/imu", Imu, self.callback_robot_imu)

        # Laser scanner
        self.g_range_ahead = 1
        rospy.Subscriber('scan', LaserScan, self.callback_scan)

        # Robot movement #
        # Desired position movement
        self.margin_error_pos = 0.1  # Meters
        self.burger = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.burger.wait_for_server()
        self.next_target = 0

        # Random position movement
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.state_change_time = rospy.Time.now() + rospy.Duration(1)
        self.driving_forward = True

    def log(self, msg):
        """[summary]
        Default logging
        Args:
            msg ([str]): [Message wanted to be logged]
        """
        print(self.log_tag + str(msg))

    def callback_robot_pose(self, payload):
        """[summary]
        Callback on robot position in odometry. Save to class variable
        Args:
            payload ([Odemtry]): [Odemtry object]
        """
        self.robot_pos = payload

    def callback_scan(self, msg):
        """[summary]
        Callback on message received on topic /scan to save results
        Args:
            msg ([LaserScan]): [LaserScan data object]
        """
        tmp = [msg.ranges[0]]
        for i in range(1, 21):
            tmp.append(msg.ranges[i])
        for i in range(len(msg.ranges)-21, len(msg.ranges)):
            tmp.append(msg.ranges[i])
            self.g_range_ahead = min(tmp)

    def callback_robot_imu(self, payload):
        self.robot_imu_pos = payload

    def get_robot_x_y_position(self):
        """[summary]
        Get current x and y position of robot in Odometry frame
        Returns:
            [tuple]: [Position]
        """
        if self.robot_pos != None:
            return (self.robot_pos.pose.pose.position.x, self.robot_pos.pose.pose.position.y, self.robot_pos.pose.pose.position.z)
        else:
            return None

    def get_robot_pose(self):
        """[summary]
        Return the current known positon of the robot in odometry coordinates
        Returns:
            [Pose]: [Odemetry coordinates]
        """
        return self.robot_pos.pose.pose

    def goal_pose(self, pose, frame="odom"):
        """[summary]
        Transform pose to a Object MovebaseGoal
        Args:
            pose ([Pose]): [goal position]

        Returns:
            [MovebaseGoal]: [MoveBaseGoal object in desired position]
        """
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = frame
        goal_pose.target_pose.pose.position.x = pose.position.x
        goal_pose.target_pose.pose.position.y = pose.position.y
        goal_pose.target_pose.pose.position.z = pose.position.z
        goal_pose.target_pose.pose.orientation.x = pose.orientation.x
        goal_pose.target_pose.pose.orientation.y = pose.orientation.y
        goal_pose.target_pose.pose.orientation.z = pose.orientation.z
        goal_pose.target_pose.pose.orientation.w = pose.orientation.w

        return goal_pose

    def move_to_pose(self, pose, duration=40, frame="odom"):
        """[summary]
        Will move the robot to a desired pose
        Args:
            pose ([Pose]): [Position of desired pose]
            duration ([rospy.duration()], optional): [How long it should try to reach goal position]. Defaults to Infinity.

        Returns:
            [Bool]: [Success]
        """
        self.log("Move to pose")
        goal = self.goal_pose(pose, frame=frame)
        self.burger.send_goal(goal)
        start = time.time()
        while True:
            state = self.burger.get_state()
            if self.move_to_pose_fail(state):
                return False

            elif self.move_to_pose_succes(state):
                break

            elif (time.time() - start >= duration):
                self.log("Move to pose timed out")
                return False

        x_desired = pose.position.x
        y_desired = pose.position.y
        (x_reported, y_reported, _) = self.get_robot_x_y_position()
        x_pos_state = (x_desired + self.margin_error_pos >=
                       x_reported and x_desired - self.margin_error_pos <= x_reported)
        y_pos_state = (y_desired + self.margin_error_pos >=
                       y_reported and y_desired - self.margin_error_pos <= y_reported)

        self.log("Got to position")
        # Check if the desired position was aquired
        if x_pos_state and y_pos_state:
            return True
        else:
            return False

    def move_to_pose_fail(self, state):

        if state == actionlib.GoalStatus.ABORTED:
            self.log("Goal aborted")
            return True

        elif state == actionlib.GoalStatus.LOST:
            self.log("Burger lost goal")
            return True

        return False

    def move_to_pose_succes(self, state):
        if state == actionlib.GoalStatus.SUCCEEDED:
            self.log("Burger found destination")
            return True
        else:
            return False

    def move_to_pose_looking_for_qr_code(self, pose, next_x_y=None, stop_before_any=False):
        self.log("Move to pose looking for QR code")
        goal = self.goal_pose(pose)
        self.burger.send_goal(goal)

        timeout = 40
        start = time.time()
        while True:
            state = self.burger.get_state()
            if self.move_to_pose_fail(state):
                return False

            elif self.move_to_pose_succes(state):
                self.log("Did not find QR code")
                return False

            elif stop_before_any and self.check_if_qr_code(new=True, stop=True):
                self.log("Found an abritary QR code")
                return True

            elif next_x_y != None and self.check_if_qr_code(new=False, stop=False):
                if self.qr_code_util.qr_code_message_coordinates_matches(next_x_y):
                    self.stop_moving()
                    self.save_robot_imu_pose()
                    self.log("Found the correct new QR code")
                    return True

            elif (time.time() - start >= timeout):
                self.log("Move to pose looking for qr code timeout")
                return False

    def read_qr_code(self, duration):
        """[summary]
        Will read the QR code position for a specific duration accumulating and averaging results
        Args:
            duration ([int]): [How long to analyze QR code]

        Returns:
            [PoseStamped]: [Position of QR code and coordinate frame]
        """
        self.log("Read QR code")
        rospy.sleep(1)
        start = time.time()
        position_x = []
        position_y = []
        position_z = []
        orientation_x = []
        orientation_y = []
        orientation_z = []
        orientation_w = []
        while (time.time() - start <= duration):
            pose_now = self.qr_code_util.get_qr_code_pose()
            position_x.append(pose_now.position.x)
            position_y.append(pose_now.position.y)
            position_z.append(pose_now.position.z)
            orientation_x.append(pose_now.orientation.x)
            orientation_y.append(pose_now.orientation.y)
            orientation_z.append(pose_now.orientation.z)
            orientation_w.append(pose_now.orientation.w)

        qr_code = self.qr_code_util.qr_code_position
        qr_code.pose.position.x = np.average(np.array(position_x))
        qr_code.pose.position.y = np.average(np.array(position_y))
        qr_code.pose.position.z = np.average(np.array(position_z))
        qr_code.pose.orientation.x = np.average(np.array(orientation_x))
        qr_code.pose.orientation.y = np.average(np.array(orientation_y))
        qr_code.pose.orientation.z = np.average(np.array(orientation_z))
        qr_code.pose.orientation.w = np.average(np.array(orientation_w))
        self.save_robot_pose()

        return qr_code

    def turn_around(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.3

        self.cmd_vel_pub.publish(twist)

    def stop_moving(self):
        self.burger.cancel_all_goals()
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)

    def calculating_angle_diff(self, first_angle, second_angle):
        diff = second_angle - first_angle
        if (diff < -math.pi):
            diff += 2*math.pi
        if (diff > math.pi):
            diff -= 2*math.pi

        return abs(diff)

    def orient_to_saved_robot_pos(self):
        self.move_to_pose(self.saved_robot_pos.pose.pose)

    def get_goal_pose(self, desired_pose):
        goal_pose = Pose()
        goal_pose.position.x = desired_pose.pose.position.x
        goal_pose.position.y = desired_pose.pose.position.y
        goal_pose.position.z = 0
        goal_pose.orientation = desired_pose.pose.orientation

        return goal_pose

    def save_robot_pose(self):
        """[summary]
        Saving the robots current pose
        """
        self.saved_robot_pos = self.robot_pos

    def save_robot_imu_pose(self):
        self.saved_robot_imu_pos = self.robot_imu_pos

    def patrol_point(self, desired_pose):
        pose = Pose()
        pose.position.x = desired_pose[0]
        pose.position.y = desired_pose[1]
        pose.position.z = 0
        pose.orientation = self.robot_pos.pose.pose.orientation
        return pose

    def check_if_qr_code(self, new, stop):
        """[summary]
        Args:
            new ([bool]): [Get a new QR code]
        Returns:
            [bool]: [If found new QR code state]
        """
        if self.qr_code_util.is_qr_code_detected() and self.qr_code_util.qr_code_position != None:
            if new and not self.qr_code_util.is_new_qr_code():
                return False

            if stop:
                self.stop_moving()
                self.save_robot_pose()
                self.save_robot_imu_pose()

            return True

    def find_qr_code_turn_around(self, new=True, next_x_y=None):
        self.log("Searching for QR code while 360 degrees turn")
        (_, _, start_angle) = self.transform.convert_orientation_to_euler(
            self.robot_imu_pos.orientation)
        accumulated_turn = 0

        while not rospy.is_shutdown():

            self.turn_around()
            if next_x_y == None:
                if self.check_if_qr_code(new, stop=True):
                    return True
            else:
                if self.check_if_qr_code(new, stop=False):
                    if self.qr_code_util.qr_code_message_coordinates_matches(next_x_y):
                        self.stop_moving()
                        return True

            (_, _, current_angle) = self.transform.convert_orientation_to_euler(
                self.robot_imu_pos.orientation)

            accumulated_turn += self.calculating_angle_diff(
                current_angle, start_angle)

            if (accumulated_turn >= 2*math.pi):
                self.stop_moving()
                return False

            start_angle = current_angle

    def increment_patrol_target(self):
        self.log("Increment patrol target")
        self.next_target += 1

    def drive_patrol(self):
        patrol_points = [[-4.6, 1.1], [-6.0, 0.28],
                         [-4.8, -2.2], [0.9, 0.6], [4.7, 1.1], [5.6, -2.0]]

        self.log("Drive to predefined point: " +
                 str(patrol_points[self.next_target]))

        patrol_point_pose = self.patrol_point(patrol_points[self.next_target])

        if self.move_to_pose_looking_for_qr_code(patrol_point_pose, stop_before_any=True):
            self.stop_moving()
            return True

        if self.find_qr_code_turn_around(new=True):
            self.stop_moving()

        return False

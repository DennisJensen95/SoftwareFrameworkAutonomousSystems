#!/usr/bin/env python
from time import sleep
import rospy
from nav_msgs.msg import Odometry
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Pose, PoseStamped
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import time
import numpy as np


class BurgerUtility():
    """[summary]
    Class for utilities in moving the robot 
    """

    def __init__(self, QrCodeUtil):
        """
            Initialize Robot Class
        """
        self.log_tag = "[Robot Utilities]:"

        # QR code utilities
        self.qr_code_util = QrCodeUtil

        # Robot position
        self.robot_pos = None
        self.saved_robot_pos = None
        rospy.Subscriber("/odom",
                         Odometry, self.callback_robot_pose)

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

    def log(self, msg):
        """[summary]
        Default logging
        Args:
            msg ([str]): [Message wanted to be logged]
        """
        print(self.log_tag + msg)

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

    def goal_pose(self, pose):
        """[summary]
        Transform pose to a Object MovebaseGoal
        Args:
            pose ([Pose]): [goal position]

        Returns:
            [MovebaseGoal]: [MoveBaseGoal object in desired position]
        """
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = "odom"
        goal_pose.target_pose.pose.position.x = pose.position.x
        goal_pose.target_pose.pose.position.y = pose.position.y
        goal_pose.target_pose.pose.position.z = pose.position.z
        goal_pose.target_pose.pose.orientation.x = pose.orientation.x
        goal_pose.target_pose.pose.orientation.y = pose.orientation.y
        goal_pose.target_pose.pose.orientation.z = pose.orientation.z
        goal_pose.target_pose.pose.orientation.w = pose.orientation.w

        return goal_pose

    def move_to_pose(self, pose, duration=rospy.Duration(40)):
        """[summary]
        Will move the robot to a desired pose
        Args:
            pose ([Pose]): [Position of desired pose]
            duration ([rospy.duration()], optional): [How long it should try to reach goal position]. Defaults to Infinity.

        Returns:
            [Bool]: [Success]
        """
        goal = self.goal_pose(pose)
        self.burger.send_goal(goal)
        self.burger.wait_for_result(duration)

        x_desired = pose.position.x
        y_desired = pose.position.y
        (x_reported, y_reported, _) = self.get_robot_x_y_position()
        x_pos_state = (x_desired + self.margin_error_pos >=
                       x_reported and x_desired - self.margin_error_pos <= x_reported)
        y_pos_state = (y_desired + self.margin_error_pos >=
                       y_reported and y_desired - self.margin_error_pos <= y_reported)

        # Check if the desired position was aquired
        if x_pos_state and y_pos_state:
            return True
        else:
            return False

    def move_to_pose_looking_for_qr_code(self, pose, next_x_y=None, stop_before_any=False):
        goal = self.goal_pose(pose)
        self.burger.send_goal(goal)

        while True:
            # If the robot is in the desired position
            if self.burger.wait_for_result(rospy.Duration(0.3)):
                return False
                
            elif stop_before_any and self.check_if_qr_code(new=True):
                self.log("Found an abritary QR code")
                return True

            elif next_x_y != None and self.check_if_qr_code(new=True):
                if self.qr_code_util.qr_code_message_coordinates_matches(next_x_y):
                    self.log("Found the correct new QR code")
                    return True

    def aquire_frame_transformations(self):
        """[summary]
        If transform buffer is not filled, try waiting and getting it again.
        Returns:
            [Bool]: [State]
        """
        self.log("Sleeping to get transform frames")
        rospy.sleep(1)
        return True

    def read_qr_code(self, duration):
        """[summary]
        Will read the QR code position for a specific duration accumulating and averaging results
        Args:
            duration ([int]): [How long to analyze QR code]

        Returns:
            [PoseStamped]: [Position of QR code and coordinate frame]
        """
        sleep(1)
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

        return qr_code

    def get_desired_position_on_qr_code(self, orig_pose, diff_pose, dist_from=0):
        """[summary]
        Get position of QR code relative to the robots current pose
        Args:
            orig_pose ([Pose]): [Robots position]
            diff_pose ([Pose]): [Relative translation position]
            dist_from (int, optional): [Distance from the relative position change you]. Defaults to 0.

        Returns:
            [Pose]: [Desired pose object]
        """
        quaternion_robot = (orig_pose.pose.orientation.x, orig_pose.pose.orientation.y,
                            orig_pose.pose.orientation.z, orig_pose.pose.orientation.w)
        quaternion_qr = (diff_pose.pose.orientation.x, diff_pose.pose.orientation.y,
                         diff_pose.pose.orientation.z, diff_pose.pose.orientation.w)
        (roll_robot, pitch_robot, heading_robot) = euler_from_quaternion(
            quaternion_robot)
        (roll_qr, pitch_qr, heading_qr) = euler_from_quaternion(quaternion_qr)

        # angle_between_qr_and_robot = heading_robot + (math.pi - roll_qr)

        quaternion_desired = quaternion_from_euler(
            roll_robot, pitch_robot, heading_robot)

        # self.log("QR roll:" + str(roll_qr*180/math.pi))
        # self.log("QR pitch: " + str(pitch_qr*180/math.pi))
        # self.log("QR Heading: " + str(heading_qr*180/math.pi))
        # self.log("Heading is:" + str(heading_robot*180/math.pi))
        desired_pose = orig_pose
        desired_pose.pose.position.x = orig_pose.pose.position.x + \
            diff_pose.pose.position.x + \
            math.cos(heading_robot) * (diff_pose.pose.position.z - dist_from)
        desired_pose.pose.position.y = orig_pose.pose.position.y + \
            diff_pose.pose.position.y + \
            math.sin(heading_robot) * (diff_pose.pose.position.z - dist_from)

        # QR Code
        self.log(str(diff_pose.pose.orientation.z))

        desired_pose.pose.orientation.x = quaternion_desired[0]
        desired_pose.pose.orientation.y = quaternion_desired[1]
        desired_pose.pose.orientation.z = quaternion_desired[2]
        desired_pose.pose.orientation.w = quaternion_desired[3]

        return desired_pose

    def drive_random(self):
        """[summary]
        Drive randomly and avoid obstacle collision
        """

        if self.g_range_ahead < 0.8:
            # TURN
            driving_forward = False

        else:  # we're not driving_forward
            driving_forward = True  # we're done spinning, time to go forward!

        twist = Twist()
        if driving_forward:
            twist.linear.x = 0.4
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.4

        self.cmd_vel_pub.publish(twist)

    def turn_around(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.4

        self.cmd_vel_pub.publish(twist)

    def find_qr_code_turn_around(self, new=True):
        self.log("Searching for QR code")
        start = time.time()
        duration = 10
        while not rospy.is_shutdown():

            self.turn_around()
            if self.check_if_qr_code(new):
                return True

            if (time.time() - start >= duration):
                return False

    def transform_qr_code_to_desired_pos(self, qr_code_pos, dist_from, tries=2):
        """[summary]
        Transform QR code position in /MAP frame to /ODOM frame

        Args:
            qr_code_pos ([PoseStamp]): [QR code position in /map frame]
            dist_from ([float]): [Distance to get coordinate from object]
            tries (int, optional): [Tries to transform coordinates]. Defaults to 2.

        Returns:
            [bool]: [Success]
        """
        for i in range(tries):
            try:
                qr_code_pos_odom_frame = self.qr_code_util.transform_pose_in_frames(
                    qr_code_pos, '/odom', '/map')

                # Internal class component updating robot position
                robot_pose = self.robot_pos.pose

                desired_pose = self.get_desired_position_on_qr_code(
                    robot_pose, qr_code_pos_odom_frame, dist_from=dist_from)
                return desired_pose
            except Exception:
                print("Did not find transformation")
                self.aquire_frame_transformations()
                qr_code_pos = self.read_qr_code(duration=1)

        return False

    def drive_to_qr_code(self, qr_code_pos, dist_from=0.8):
        qr_code_pos = self.transform_qr_code_to_desired_pos(
            qr_code_pos, dist_from=dist_from)

        if isinstance(qr_code_pos, bool):
            return False

        goal_pose = self.get_goal_pose(qr_code_pos)
        self.log("Move to goal_pose:" + str(goal_pose))
        if self.move_to_pose(goal_pose, rospy.Duration()):
            self.log("Succesfull moved to target position")
            self.qr_code_util.save_code_message(qr_code_pos.pose.position)
            return True

        return False

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

            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
            self.cmd_vel_pub.publish(twist)

            self.save_robot_pose()
            self.qr_code_util.save_qr_code_position()
            return True

        return False
    
    def drive_towards_qr_code(self):
        # Orient correct
        self.move_to_pose(self.saved_robot_pos.pose.pose)

    def patrol_point(self, desired_pose):
        pose = Pose()
        pose.position.x = desired_pose[0]
        pose.position.y = desired_pose[1]
        pose.position.z = 0
        pose.orientation = self.robot_pos.pose.pose.orientation
        return pose

    def drive_patrol(self):
        patrol_points = [[-4.6, 1.1], [-6.0, 0.28], [-4.8, -2.2], [0.9, 0.6], [4.7, 1.1], [5.6, -2.0]]

        patrol_point_pose = self.patrol_point(patrol_points[self.next_target])

        # update next_target
        self.next_target += 1
        self.next_target = self.next_target % len(patrol_points)

        if self.move_to_pose_looking_for_qr_code(patrol_point_pose, stop_before_any=True):
            return True
        
        return self.find_qr_code_turn_around(new=True)

    def find_qr_code(self, new=False):
        """
        Description of function here
        """
        self.log("Searching for QR code")
        
        while not rospy.is_shutdown():    
            if self.drive_patrol():
                break

        self.log("Found QR Code orient robot again")
        self.drive_towards_qr_code()

    def drive_to_next_qr_code(self, next_x_y):
        """[summary]
        Drive to the next QR code from the last 
        """
        check_pos = PoseStamped()
        check_pos.header.frame_id = "hidden_frame"
        check_pos.pose.position.x = next_x_y[0]
        check_pos.pose.position.y = next_x_y[1]
        check_pos.pose.orientation.x = 0
        check_pos.pose.orientation.y = 0
        check_pos.pose.orientation.z = 0
        check_pos.pose.orientation.w = 1
        self.log("Going to new QR code")
        desired_pose = self.qr_code_util.transform_pose_in_frames(
            check_pos, "odom", "hidden_frame").pose

        self.move_to_pose_looking_for_qr_code(desired_pose, next_x_y)

        return True

    # Deprecated
    def drive_to_qr_code_step(self, qr_code_pos):

        while True:
            previous_pose = self.get_robot_pose()
            qr_code_pos = self.transform_qr_code_to_desired_pos(
                qr_code_pos, dist_from=0.8)

            if isinstance(qr_code_pos, bool):
                return False

            goal_pose = self.get_goal_pose(qr_code_pos)
            self.log("Move to goal_pose:" + str(goal_pose))
            if self.move_to_pose(goal_pose, rospy.Duration()):
                self.log("Succesfull moved to target position")
                self.qr_code_util.save_code_message()
                return True

            self.burger.cancel_all_goals()
            if not self.qr_code_util.is_qr_code_detected():
                if not self.move_to_pose(previous_pose, rospy.Duration()):
                    self.log("Failed to drive to QR code")
                    return False

            qr_code_pos = self.read_qr_code(duration=0.5)

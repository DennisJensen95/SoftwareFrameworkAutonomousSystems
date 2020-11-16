#!/usr/bin/env python
from numpy.lib.function_base import diff
import rospy
from nav_msgs.msg import Odometry
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import time
import numpy as np


class BurgerUtility():
    """
        Robot Utility box specialized for final project in Software Autonomous systems
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

        # Random position movement
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.state_change_time = rospy.Time.now() + rospy.Duration(1)
        self.driving_forward = True

    def callback_robot_pose(self, payload):
        self.robot_pos = payload

    def callback_scan(self, msg):
        tmp = [msg.ranges[0]]
        for i in range(1, 21):
            tmp.append(msg.ranges[i])
        for i in range(len(msg.ranges)-21, len(msg.ranges)):
            tmp.append(msg.ranges[i])
            self.g_range_ahead = min(tmp)

    def log(self, msg):
        print(self.log_tag + msg)

    def get_robot_x_y_position(self):
        if self.robot_pos != None:
            return (self.robot_pos.pose.pose.position.x, self.robot_pos.pose.pose.position.y, self.robot_pos.pose.pose.position.z)
        else:
            return None

    def get_robot_pose(self):
        return self.robot_pos.pose.pose

    def goal_pose(self, pose):
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

    def move_to_pose(self, pose, duration=rospy.Duration()):
        goal = self.goal_pose(pose)
        self.burger.send_goal(goal)
        self.log(str(self.burger.wait_for_result(duration)))

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

    def aquire_frame_transformations(self):
        self.log("Sleeping to get transform frames")
        rospy.sleep(1)
        return True

    def read_qr_code(self, duration):
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

    def transform_qr_code_to_desired_pos(self, qr_code_pos, dist_from):
        for i in range(2):
            try:
                qr_code_pos_odom_frame = self.qr_code_util.transform_pose_in_frames(
                    qr_code_pos, '/odom', '/map')

                robot_pose = self.robot_pos.pose

                desired_pose = self.get_desired_position_on_qr_code(
                    robot_pose, qr_code_pos_odom_frame, dist_from=dist_from)
                return desired_pose
            except Exception:
                print("Did not find transformation")
                self.aquire_frame_transformations()
                qr_code_pos = self.read_qr_code(duration=1)

        return False

    def drive_to_qr_code(self, qr_code_pos):

        while True:
            previous_pose = self.get_robot_pose()
            qr_code_pos = self.transform_qr_code_to_desired_pos(
                qr_code_pos, dist_from=1.2)

            if isinstance(qr_code_pos, bool):
                return False

            goal_pose = self.get_goal_pose(qr_code_pos)
            self.log("Move to goal_pose:" + str(goal_pose))
            if self.move_to_pose(goal_pose, rospy.Duration(5)):
                self.log("Succesfull moved to target position")
                self.qr_code_util.save_code_message()
                return True

            self.burger.cancel_all_goals()
            if not self.qr_code_util.is_qr_code_detected():
                if not self.move_to_pose(previous_pose, rospy.Duration()):
                    self.log("Failed to drive to QR code")
                    return False

            qr_code_pos = self.read_qr_code(duration=0.5)

    def get_goal_pose(self, desired_pose):
        goal_pose = Pose()
        goal_pose.position.x = desired_pose.pose.position.x
        goal_pose.position.y = desired_pose.pose.position.y
        goal_pose.position.z = 0
        goal_pose.orientation = desired_pose.pose.orientation

        return goal_pose

    def find_qr_code(self):
        """
        Description of function here
        """

        self.log("Searching for QR code")
        while not rospy.is_shutdown():

            self.drive_random()

            if self.qr_code_util.is_qr_code_detected() and self.qr_code_util.qr_code_position != None:
                twist = Twist()
                twist.linear.x = 0
                twist.angular.z = 0
                self.cmd_vel_pub.publish(twist)
                break

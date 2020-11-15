#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import math


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

    def move_to_pose(self, pose):
        goal = self.goal_pose(pose)
        self.burger.send_goal(goal)
        self.log(str(self.burger.wait_for_result()))

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

    def get_desired_position_on_qr_code(self, orig_pose, diff_pose, dist_from=0):
        quaternion = (orig_pose.pose.orientation.x, orig_pose.pose.orientation.y,
                      orig_pose.pose.orientation.z, orig_pose.pose.orientation.w)
        (_, _, heading) = euler_from_quaternion(quaternion)
        self.log("Heading is:" + str(heading))
        desired_pose = orig_pose
        desired_pose.pose.position.x = orig_pose.pose.position.x + \
            diff_pose.pose.position.x + \
            math.cos(heading) * (diff_pose.pose.position.z - dist_from)
        desired_pose.pose.position.y = orig_pose.pose.position.y + \
            diff_pose.pose.position.y + \
            math.sin(heading) * (diff_pose.pose.position.z - dist_from)

        return desired_pose

    def find_qr_code(self):
        """
        Description of function here
        """

        self.log("Searching for QR code")
        while not rospy.is_shutdown():
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

            if self.qr_code_util.is_qr_code_detected() and self.qr_code_util.qr_code_position != None:
                twist.linear.x = 0
                twist.angular.z = 0
                self.cmd_vel_pub.publish(twist)
                break

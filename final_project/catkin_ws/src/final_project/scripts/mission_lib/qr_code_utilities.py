#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
import tf
import threading
import re


class QrCodeUtility():
    """
        QrCode Utility box specialized for final project in Software Autonomous systems
    """

    def __init__(self):
        """
            Initialize QrCodeUtility Class
        """

        self.log_tag = "[QR Code Utility]:"

        # QR code message
        self.saved_code_message = ""
        self.qr_code_message = ""
        self.qr_code_detected = False

        # QR code position
        self.qr_code_position = None

        # Initializing tf broadcaster
        self.tf_listener = tf.TransformListener()
        self.kill_broadcast_tf = False
        self.pose_diff_hidden_frame_to_odom = None
        self.hidden_frame_tf_thread = threading.Thread(
            target=self.broadcast_transform_hidden_to_odom)
        self.hidden_frame_tf_thread.start()

        # QR code topics subscribers
        rospy.Subscriber("/visp_auto_tracker/code_message",
                         String, self.callback_qr_code_message)
        rospy.Subscriber("/visp_auto_tracker/object_position",
                         PoseStamped, self.callback_qr_code_position)

    def callback_qr_code_message(self, payload):

        if payload.data != "":
            self.qr_code_detected = True
            self.qr_code_message = payload.data
        else:
            self.qr_code_detected = False

    def save_code_message(self):
        self.saved_code_message = self.qr_code_message

    def get_qr_code_pose(self):
        return self.qr_code_position.pose

    def get_current_qr_code_x_y(self):
        if self.qr_code_message != "":
            coordinates = re.findall(
                "X=(.*)\\r\\nY=(.*)\\r", self.saved_code_message)[0]
            x = float(coordinates[0])
            y = float(coordinates[1])
            return (x, y)
        return (0, 0)

    def get_next_qr_code_x_y(self):
        if self.qr_code_message != "":
            coordinates = re.findall(
                "X_next=(.*)\\r\\nY_next=(.*)\\r", self.saved_code_message)[0]

            x = float(coordinates[0])
            y = float(coordinates[1])
            return (x, y)
        return (0, 0)

    def log(self, msg):
        print(self.log_tag + msg)

    def teardown(self):
        self.hidden_frame_tf_thread.join()

    def is_qr_code_detected(self):
        return self.qr_code_detected

    def callback_qr_code_position(self, payload):
        if self.qr_code_detected:
            self.qr_code_position = payload

    def transform_pose_in_frames(self, pose, target_frame, current_frame):
        now = rospy.Time.now()
        self.tf_listener.waitForTransform(
            target_frame, current_frame, now, rospy.Duration(4.0))
        pose_odo_frame = self.tf_listener.transformPose(target_frame, pose)

        return pose_odo_frame

    def broadcast_transform_hidden_to_odom(self,):
        br = tf.TransformBroadcaster()
        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            if not (self.pose_diff_hidden_frame_to_odom == None):
                rotation = self.pose_diff_hidden_frame_to_odom.orientation
                rot_tuple = (rotation.x, rotation.y, rotation.z, rotation.w)
                translation = self.pose_diff_hidden_frame_to_odom.position
                trans_tuple = (translation.x, translation.y, translation.z)
                # self.log("Translation is: " + str(trans_tuple))
                # self.log("Rotation is: " + str(rot_tuple))
                try:

                    br.sendTransform(trans_tuple, rot_tuple,
                                     rospy.Time.now(), "hidden_frame", "odom")
                    rate.sleep()
                except rospy.exceptions.ROSInterruptException:
                    pass
            if self.kill_broadcast_tf:
                break

    def create_transform_from_odom_to_hidden_frame(self, qr_code_position_odo):
        qr_coordinates = self.get_current_qr_code_x_y()
        self.log(str(qr_coordinates))
        self.log(str(qr_code_position_odo))
        self.pose_diff_hidden_frame_to_odom = Pose()
        self.pose_diff_hidden_frame_to_odom.position.x = qr_code_position_odo.pose.position.x + \
            qr_coordinates[0]
        self.pose_diff_hidden_frame_to_odom.position.y = qr_code_position_odo.pose.position.y + \
            qr_coordinates[1]

        self.pose_diff_hidden_frame_to_odom.orientation.x = 0
        self.pose_diff_hidden_frame_to_odom.orientation.y = 0
        self.pose_diff_hidden_frame_to_odom.orientation.z = 0
        self.pose_diff_hidden_frame_to_odom.orientation.w = 1

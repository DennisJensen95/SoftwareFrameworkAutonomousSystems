#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
import tf
import threading


class QrCodeUtility():
    """
        QrCode Utility box specialized for final project in Software Autonomous systems
    """

    def __init__(self):
        """
            Initialize QrCodeUtility Class
        """

        # QR code message
        self.qr_code_message = ""
        self.qr_code_detected = False

        # QR code position
        self.qr_code_position = None

        # Initializing tf broadcaster
        self.tf_listener = tf.TransformListener()
        self.kill_broadcast_tf = False
        self.pose_diff_hidden_frame_to_odom = None
        self.hidden_frame_tf_thread = threading.Thread(
            target=self.broadcast_transform_hidden_to_odom).start()

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

    def is_qr_code_detected(self):
        return self.qr_code_detected

    def callback_qr_code_position(self, payload):
        if self.qr_code_detected:
            self.qr_code_position = payload

    def transform_pose_from_map_to_odom(self, pose):
        now = rospy.Time.now()
        self.tf_listener.waitForTransform(
            '/odom', '/map', now, rospy.Duration(4.0))
        pose_odo_frame = self.tf_listener.transformPose("/odom", pose)

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
                print("Translation is: %s", str(trans_tuple))
                print("Rotation is: %s", str(rot_tuple))
                br.sendTransform(trans_tuple, rot_tuple,
                                 rospy.Time.now(), "hidden_frame", "odom")
                rate.sleep()
            if self.kill_broadcast_tf:
                break

    def create_transform_from_odom_to_hidden_frame(self, pose_robot, pose_hidden_frame):
        self.pose_diff_hidden_frame_to_odom = Pose()
        translational_diff = pose_hidden_frame.position - pose_robot.position
        rotational_diff = pose_hidden_frame.orientation - pose_robot.orientation

        self.pose_diff_hidden_frame_to_odom.position = translational_diff
        self.pose_diff_hidden_frame_to_odom.orientation = rotational_diff

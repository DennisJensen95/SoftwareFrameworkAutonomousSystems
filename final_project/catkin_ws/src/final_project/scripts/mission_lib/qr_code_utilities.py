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
        
        rospy.init_node('qr_code_tools', anonymous=True)

        # QR code message
        self.qr_code_message = ""
        self.qr_code_detected = False

        # QR code position
        self.qr_code_position = None
        
        # Initializing tf broadcaster
        self.tf_listener = tf.TransformListener()
        self.kill_broadcast_tf = False
        self.pose_diff_hidden_frame_to_odom = None
        self.hidden_frame_tf_thread = threading.Thread(target=self.broadcast_transform_hidden_to_odom).start()

        # QR code topics subscribers
        rospy.Subscriber("/visp_auto_tracker/code_message",
                         String, self.callback_qr_code_message)
        rospy.Subscriber("/visp_auto_tracker/object_position", PoseStamped, self.callback_qr_code_position)                         

        self.main_thread = threading(target=self.main).start()

    def main(self):
        rospy.spin()

    def callback_qr_code_message(self, payload):
        self.qr_code_message = payload.data

        if payload.data != "":
            self.qr_code_detected = True
        else:
            self.qr_code_detected = False
    
    def callback_qr_code_position(self, payload):
        self.qr_code_position = payload

    def transform_pose_from_map_to_odom(self, pose):
        (trans,rot) = self.tf_listener.lookupTransform('/odom', '/map', rospy.Time(0))

        pose.position = pose.position + trans
        pose.orientation = pose.orientation + rot

        return pose

    def broadcast_transform_hidden_to_odom(self,):
        br = tf.TransformBroadcaster()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if not (self.pose_diff_hidden_frame_to_odom==None):
                br.sendTransform(self.pose_diff_hidden_frame_to_odom.position, self.pose_diff_hidden_frame_to_odom.orientation, 
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

       
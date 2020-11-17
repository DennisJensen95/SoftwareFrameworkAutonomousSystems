#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
import tf
import threading
import re
import numpy as np
from kabsch import rigid_transform_3D
from tf.transformations import quaternion_from_matrix, rotation_matrix


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

        # QR code resemblance X Y coordinate
        self.margin_error_resemblance = 0.15

        # QR Messages position
        self.qr_messages_position = {}

        # QR code position
        self.qr_code_position = None
        self.saved_qr_code_position = None

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
        """[summary]
        Check if code message of QR code is present - if so QR code detected and
        class state variable is True else False
        Args:
            payload ([str]): [QR code message]
        """
        if payload.data != "":
            self.qr_code_detected = True
            self.qr_code_message = payload.data
        else:
            self.qr_code_detected = False

    def callback_qr_code_position(self, payload):
        """[summary]
        Save QR code position if QR code is detected
        Args:
            payload ([PoseStamped]): [description]
        """
        if self.qr_code_detected:
            self.qr_code_position = payload

    def save_code_message(self, qr_code_pos_odom):
        """[summary]
        Save a QR code message at a specific time and dont change before next save
        """
        if self.qr_code_message != "":
            self.saved_code_message = self.qr_code_message
            next_qr = self.get_saved_next_qr_code_x_y()
            current_qr = self.get_saved_qr_code_x_y()
            N = re.findall("N=(\d+)", self.saved_code_message)[0]
            L = re.findall("L=(\w+)", self.saved_code_message)[0]
            data_object = self.qr_codes_data_object(
                N, L, current_qr, next_qr, qr_code_pos_odom)
            self.qr_messages_position.update(data_object)

            return True
        
        return False

    def get_distance_to_saved_qr_code(self):
        """[summary]
        Return the distance to QR Code
        """
        return self.saved_qr_code_position.pose.position.z

    def qr_codes_data_object(self, N, L, pos_qr, next_pos_qr, odom_pos):
        return {N: {"L": L, "pos": pos_qr, "next_pos": next_pos_qr, "odom_pos": odom_pos}}

    def qr_code_message_coordinates_matches(self, pose_x_y):
        cur_x_y = self.get_saved_next_qr_code_x_y()
        x_pos_state = (pose_x_y[0] + self.margin_error_resemblance >=
                       cur_x_y[0] and pose_x_y[0] - self.margin_error_resemblance <= cur_x_y[0])
        y_pos_state = (pose_x_y[1] + self.margin_error_resemblance >=
                       cur_x_y[1] and pose_x_y[1] - self.margin_error_resemblance <= cur_x_y[1])
        
        # Check if the qr code read is the same as desired
        if x_pos_state and y_pos_state:
            return True
        else:
            return False

    def is_new_qr_code(self):
        found_qr_codes = self.return_found_qr_codes()
        N = self.get_current_N_code()
        state = not (N in found_qr_codes)
        return state

    def get_current_N_code(self):
        N = re.findall("N=(\d+)", self.qr_code_message)[0]
        return N

    def get_number_of_qr_codes(self):
        return len(self.return_found_qr_codes())

    def return_found_qr_codes(self):
        N_found = []
        for key in self.qr_messages_position.keys():
            N_found.append(key)

        return N_found

    def get_qr_code_pose(self):
        """[summary]
        Get position fo QR code detected
        Returns:
            [Pose]: [Position of QR code in camera]
        """
        return self.qr_code_position.pose

    def get_saved_qr_code_x_y(self):
        """[summary]
        Current x and y position of code message saved
        Returns:
            [tuple]: [Position in QR code message]
        """
        if self.qr_code_message != "":
            coordinates = re.findall(
                "X=(.*)\\r\\nY=(.*)\\r", self.saved_code_message)[0]
            x = float(coordinates[0])
            y = float(coordinates[1])
            return (x, y)
        return (0, 0)

    def get_saved_next_qr_code_x_y(self):
        """[summary]
        Saved QR code message containing next QR code position.
        Returns:
            [tuple]: [Next QR code position]
        """
        if self.qr_code_message != "":
            coordinates = re.findall(
                "X_next=(.*)\\r\\nY_next=(.*)\\r", self.saved_code_message)[0]

            x = float(coordinates[0])
            y = float(coordinates[1])
            return (x, y)
        return (0, 0)

    def log(self, msg):
        """[summary]
        Generic log message
        Args:
            msg ([str]): [Log message]
        """
        print(self.log_tag + msg)

    def teardown(self):
        """[summary]
        Teardown threading
        """
        self.hidden_frame_tf_thread.join()

    def is_qr_code_detected(self):
        """[summary]
        Check if QR code is detected
        Returns:
            [bool]: [State detection]
        """
        return self.qr_code_detected
    
    def save_qr_code_position(self):
        """[summary]
        Save the current estimated QR code position
        """
        self.saved_qr_code_position = self.qr_code_position

    def print_saved_qr_codes(self):
        
        for i in range(5):
            key = str(i+1)
            if key in self.qr_messages_position:
                self.log(str(self.qr_messages_position[key]))

    def transform_pose_in_frames(self, pose, target_frame, current_frame):
        """[summary]
        Transform a Pose from one frame to another
        Args:
            pose ([PoseStamp]): [PoseStamp that needs transformation]
            target_frame ([str]): [String of target frame]
            current_frame ([str]): [String of current frame]

        Returns:
            [PoseStamp]: [PoseStamp of the pose in target frame]
        """
        now = rospy.Time.now()
        self.tf_listener.waitForTransform(
            target_frame, current_frame, now, rospy.Duration(4.0))
        pose_odo_frame = self.tf_listener.transformPose(target_frame, pose)

        return pose_odo_frame

    def broadcast_transform_hidden_to_odom(self,):
        """[summary]
        Broadcasting transform from hidden to odometry frame when found
        """
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

    def create_transform_from_odom_to_hidden_frame(self):
        """[summary]
        Create a transform for transforming from odometry frame to hidden frame
        Args:
            qr_code_position_odo ([Pose]): [QR code position in odemetry coordinates]
        """

        A = []
        B = []
        for key in self.qr_messages_position.keys():
            data = self.qr_messages_position[key]
            A.append([data["pos"][0], data["pos"][1], 0])
            B.append([data["odom_pos"].x, data["odom_pos"].y, 0])

        A = np.transpose(np.array(A))
        B = np.transpose(np.array(B))
        (R, t) = rigid_transform_3D(A, B)
        R = np.array(R)
        R = np.append(R, np.array([[0, 0, 0]]), axis=0)
        R = np.append(R, np.array([[0], [0], [0], [1]]), axis=1)
        t = t[:, 0]

        quaternion_change = quaternion_from_matrix(R)
        self.pose_diff_hidden_frame_to_odom = Pose()
        self.pose_diff_hidden_frame_to_odom.position.x = t[0]
        self.pose_diff_hidden_frame_to_odom.position.y = t[1]
        self.pose_diff_hidden_frame_to_odom.position.z = t[2]
        self.pose_diff_hidden_frame_to_odom.orientation.x = quaternion_change[0]
        self.pose_diff_hidden_frame_to_odom.orientation.y = quaternion_change[1]
        self.pose_diff_hidden_frame_to_odom.orientation.z = quaternion_change[2]
        self.pose_diff_hidden_frame_to_odom.orientation.w = quaternion_change[3]

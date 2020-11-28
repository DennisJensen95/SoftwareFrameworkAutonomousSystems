#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import re
import numpy as np
import time


class QrCodeUtility():
    """
        QrCode Utility box specialized for final project in Software Autonomous systems
    """

    def __init__(self):
        """
            Initialize QrCodeUtility Class
        """

        self.log_tag = "[QR Code Utility]: "

        # QR code message
        self.saved_code_message = ""
        self.qr_code_message = ""
        self.qr_code_detected = False
        self.updated_qr_codes = False
        self.qr_cov_margin = 0.01

        # QR code resemblance X Y coordinate
        self.margin_error_resemblance = 0.15

        # QR Messages position
        self.qr_messages_position = {}

        # QR code position
        self.qr_code_position = None
        self.saved_qr_code_position = None

        # QR code topics subscribers
        rospy.Subscriber("/visp_auto_tracker/code_message",
                         String, self.callback_qr_code_message)
        rospy.Subscriber("/visp_auto_tracker/object_position",
                         PoseStamped, self.callback_qr_code_position)
        rospy.Subscriber("/visp_auto_tracker/object_position_covariance",
                         PoseWithCovarianceStamped, self.callback_qr_code_covariance)

    def log(self, msg):
        """[summary]
        Generic log message
        Args:
            msg ([str]): [Log message]
        """
        print(self.log_tag + str(msg))

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

    def callback_qr_code_covariance(self, payload):
        self.qr_covariance = np.sum(np.array(payload.pose.covariance))

    def save_code_message(self, qr_code_pos_odom):
        """[summary]
        Save a QR code message at a specific time and dont change before next save
        """
        if self.qr_code_message != "" and self.qr_covariance < self.qr_cov_margin and qr_code_pos_odom != False:
            self.updated_qr_codes = True
            self.saved_code_message = self.qr_code_message
            next_qr = self.get_saved_next_qr_code_x_y()
            current_qr = self.get_saved_qr_code_x_y()
            N = re.findall("N=(\d+)", self.saved_code_message)[0]
            L = re.findall("L=(\w+)", self.saved_code_message)[0]
            data_object = self.qr_codes_data_object(
                N, L, current_qr, next_qr, qr_code_pos_odom)
            self.qr_messages_position.update(data_object)
            self.log("Saved QR code position")

            return True
        else:
            self.log(
                "Did not save code message because covariance of QR code was too high")

        return False

    def qr_codes_data_object(self, N, L, pos_qr, next_pos_qr, odom_pos):
        return {N: {"L": L, "pos": pos_qr, "next_pos": next_pos_qr, "odom_pos": odom_pos}}

    def check_if_qr_code_position_is_unknown(self, next_x_y):
        for key in self.qr_messages_position.keys():
            if self.match_two_points(next_x_y, self.qr_messages_position[key]["pos"]):
                return False

        return True

    def match_two_points(self, point1, point2):
        x_pos_state = (point1[0] + self.margin_error_resemblance >=
                       point2[0] and point1[0] - self.margin_error_resemblance <= point2[0])
        y_pos_state = (point1[1] + self.margin_error_resemblance >=
                       point2[1] and point1[1] - self.margin_error_resemblance <= point2[1])

        if x_pos_state and y_pos_state:
            return True
        else:
            return False

    def qr_code_message_coordinates_matches(self, pose_x_y):
        cur_x_y = self.get_current_qr_code_x_y()
        return self.match_two_points(pose_x_y, cur_x_y)

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

    def return_qr_code_data(self):
        return self.qr_messages_position

    def get_qr_code_pose(self):
        """[summary]
        Get position fo QR code detected
        Returns:
            [Pose]: [Position of QR code in camera]
        """
        return self.qr_code_position.pose

    def get_saved_qr_code_x_y(self):
        """[summary]
        Saved x and y position of code message saved
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

    def get_current_qr_code_x_y(self):
        """[summary]
        Current x and y position of code message saved
        Returns:
            [tuple]: [Position in QR code message]
        """
        if self.qr_code_message != "":
            coordinates = re.findall(
                "X=(.*)\\r\\nY=(.*)\\r", self.qr_code_message)[0]
            x = float(coordinates[0])
            y = float(coordinates[1])
            return (x, y)
        return (0, 0)

    def is_qr_code_detected(self):
        """[summary]
        Check if QR code is detected
        Returns:
            [bool]: [State detection]
        """
        timeout = 2
        start = time.time()
        while True:
            if self.qr_code_detected:
                return True

            if (time.time() - start >= timeout):
                return False

    def save_qr_code_position(self):
        """[summary]
        Save the current estimated QR code position
        """
        if self.qr_covariance < self.qr_cov_margin:
            self.saved_qr_code_position = self.qr_code_position
        else:
            self.log("Did not save QR code because covariance was too high")

    def print_saved_qr_codes(self):

        for i in range(5):
            key = str(i+1)
            if key in self.qr_messages_position:
                self.log(self.qr_messages_position[key])

    def reset_qr_codes(self):
        self.log("Resetting QR codes")
        self.qr_messages_position = {}

    def print_saved_word(self):
        word = ""
        for i in range(5):
            key = str(i+1)
            if key in self.qr_messages_position:
                word += str(self.qr_messages_position[key]["L"])

        print("Winning word is: " + word)

    def is_qr_codes_data_updated(self):
        return self.updated_qr_codes

    def set_updated_qr_codes(self, bool):
        self.updated_qr_codes = bool

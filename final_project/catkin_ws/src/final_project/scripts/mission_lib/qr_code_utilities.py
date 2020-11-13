#!/usr/bin/env python
import rospy
from std_msgs.msg import String


class QrCodeUtility():
    """
        QrCode Utility box specialized for final project in Software Autonomous systems     
    """

    def __init__(self):
        """
            Initialize QrCodeUtility Class
        """
        self.qr_code_message = ""
        self.qr_code_detected = False

        rospy.init_node('visp_code_message', anonymous=True)
        rospy.Subscriber("/visp_auto_tracker/code_message",
                         String, self.callback_qr_code_message)

        rospy.spin()

    def callback_qr_code_message(self, payload):
        self.qr_code_message = payload.data

        if payload.data != "":
            self.qr_code_detected = True
        else:
            self.qr_code_detected = False

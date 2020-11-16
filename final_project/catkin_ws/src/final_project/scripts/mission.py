#!/usr/bin/env python
import rospy
from mission_lib.robot_utilities import BurgerUtility
from mission_lib.qr_code_utilities import QrCodeUtility


def main():
    rospy.init_node('burger_robot', anonymous=True)

    # Initialize utilities
    qr_code_util = QrCodeUtility()
    burger = BurgerUtility(qr_code_util)

    for i in range(1):
        # Find QR Code
        if i == 0:
            burger.find_qr_code()
        else:
            burger.find_qr_code(new=True)

        # QR Code is found now estimate position
        qr_code_pos = burger.read_qr_code(duration=2)

        # Drive to the QR code with a distance of 80 centimeters from QR code
        burger.drive_to_qr_code(qr_code_pos, dist_from=1.2)

        # Read the QR code again to get the best estimation of the position
        qr_code_pos = burger.read_qr_code(duration=2)

        # Transform the QR code position from /map to /odometry and in global /odemtry frame
        qr_code_pos_odom = burger.transform_qr_code_to_desired_pos(
            qr_code_pos, dist_from=0)

        # Save the code message
        qr_code_util.save_code_message(qr_code_pos_odom.pose.position)

    # Create the transform from /odom frame to /hidden_frame
    qr_code_util.create_transform_from_odom_to_hidden_frame()

    # From last saved QR code message drive to it from the new transformation frame
    # burger.drive_to_next_qr_code()

    rospy.signal_shutdown("Mission done")

    qr_code_util.teardown()


if __name__ == "__main__":
    main()

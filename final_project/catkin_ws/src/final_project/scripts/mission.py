#!/usr/bin/env python
import rospy
from mission_lib.robot_utilities import BurgerUtility
from mission_lib.qr_code_utilities import QrCodeUtility


def read_and_save_qr_code(qr_code_util, burger):
    # Read the QR code again to get the best estimation of the position
    qr_code_pos = burger.read_qr_code(duration=1)

    # Transform the QR code position from /map to /odometry and in global /odemtry frame
    qr_code_pos_odom = burger.transform_qr_code_to_desired_pos(
        qr_code_pos, dist_from=0)

    # Save the code message
    if isinstance(qr_code_pos_odom, bool):
        return False

    qr_code_util.save_code_message(qr_code_pos_odom.pose.position)

    return True


def main():
    rospy.init_node('burger_robot', anonymous=True)

    # Initialize utilities
    qr_code_util = QrCodeUtility()
    burger = BurgerUtility(qr_code_util)

    while True:
        # Find QR Code
        burger.find_qr_code(new=True)

        # QR Code is found now estimate position
        qr_code_pos = burger.read_qr_code(duration=1)

        # Drive to the QR code with a distance of 80 centimeters from QR code
        burger.drive_to_qr_code(qr_code_pos, dist_from=1.2)

        read_and_save_qr_code(qr_code_util, burger)
        qr_code_util.print_saved_qr_codes()
        if qr_code_util.get_number_of_qr_codes() >= 2:
            break

    # Create the transform from /odom frame to /hidden_frame
    qr_code_util.create_transform_from_odom_to_hidden_frame()

    while qr_code_util.get_number_of_qr_codes() < 5:
        known_qr_codes = qr_code_util.qr_messages_position

        for qr_code in known_qr_codes.keys():
            next_x_y = known_qr_codes[qr_code]["next_pos"]
            burger.drive_to_next_qr_code(next_x_y)
            burger.find_qr_code_turn_around(new=True)
            read_and_save_qr_code(qr_code_util, burger)
            qr_code_util.print_saved_qr_codes()

    rospy.signal_shutdown("Mission done")

    qr_code_util.teardown()


if __name__ == "__main__":
    main()

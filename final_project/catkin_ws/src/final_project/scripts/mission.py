#!/usr/bin/env python
import rospy
from mission_lib.robot_utilities import BurgerUtility
from mission_lib.qr_code_utilities import QrCodeUtility
from mission_lib.frame_utilities import FrameUtilities
from mission_lib.mission_planning import MissionPlanning


def log(msg):
    print("[Mission]: " + str(msg))


def read_and_save_qr_code(mission_planning):
    # Read the QR code again to get the best estimation of the position and save it
    qr_code_pos_odom = mission_planning.get_qr_code_position_in_odom_frame()
    mission_planning.save_qr_code_message(qr_code_pos_odom)
    return True


def mission_part_one(mission_planning, burger, qr_code_util):
    """[summary]
    Find and locate the two first QR codes
    """
    log("\n\nStarting Mission 2 \n\n")
    found_qr_code = False
    while True:
        # Find QR Code
        found_qr_code = mission_planning.find_qr_code()

        if found_qr_code:
            read_and_save_qr_code(mission_planning)

        # Logging saved QR codes
        qr_code_util.print_saved_qr_codes()

        # Make sure hidden frame has time for computation
        rospy.sleep(10)
        if qr_code_util.get_number_of_qr_codes() >= 2:
            break

        if found_qr_code:
            mission_planning.find_qr_code_turn_around(new=True)


def mission_part_two(mission_planning, burger, qr_code_util):
    log("\n\nStarting Mission 2 \n\n")
    while qr_code_util.get_number_of_qr_codes() < 5:
        known_qr_codes = qr_code_util.qr_messages_position

        for qr_code in known_qr_codes.keys():
            next_x_y = known_qr_codes[qr_code]["next_pos"]

            if not qr_code_util.check_if_qr_code_position_is_unknown(next_x_y):
                log("Already known position hidden_frame" + str(next_x_y))
                continue

            # Drive towards next QR code
            found_qr_code = mission_planning.drive_to_next_qr_code(next_x_y)

            if found_qr_code:
                read_and_save_qr_code(mission_planning)

            # Logging saved QR codes
            qr_code_util.print_saved_qr_codes()
            if qr_code_util.get_number_of_qr_codes() == 5:
                log("Found five QR codes mission is done!")
                break


def main():
    rospy.init_node('burger_robot', anonymous=True)

    log("\n\nInitializing Burger robot mission \n\n")

    # Initialize utilities
    qr_code_util = QrCodeUtility()
    frame_util = FrameUtilities(qr_code_util)
    burger = BurgerUtility(qr_code_util, frame_util)
    mission_planning = MissionPlanning(burger, qr_code_util, frame_util)

    mission_part_one(mission_planning, burger, qr_code_util)

    mission_part_two(mission_planning, burger, qr_code_util)

    log("\n\nFound all QR codes \n\n")
    qr_code_util.print_saved_word()
    rospy.signal_shutdown("Mission done")

    frame_util.teardown()


if __name__ == "__main__":
    main()

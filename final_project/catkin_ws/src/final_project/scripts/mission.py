#!/usr/bin/env python
import rospy
from mission_lib.robot_utilities import BurgerUtility
from mission_lib.qr_code_utilities import QrCodeUtility
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node('burger_robot', anonymous=True)

    # Initialize utilities
    qr_code_util = QrCodeUtility()
    burger = BurgerUtility(qr_code_util)

    burger.find_qr_code()
    qr_code_pos = burger.read_qr_code(duration=2)
    burger.drive_to_qr_code(qr_code_pos)
    qr_code_pos = burger.read_qr_code(duration=2)
    # print(qr_code_pos)
    qr_code_pos = burger.transform_qr_code_to_desired_pos(
        qr_code_pos, dist_from=0)
    # print(qr_code_pos)
    qr_code_util.create_transform_from_odom_to_hidden_frame(qr_code_pos)

    print("Checking trans")
    check_pos = PoseStamped()
    check_pos.header.frame_id = "hidden_frame"
    check_pos.pose.position.x = qr_code_util.get_next_qr_code_x_y()[0]
    check_pos.pose.position.y = qr_code_util.get_next_qr_code_x_y()[1]
    check_pos.pose.orientation.x = 0
    check_pos.pose.orientation.y = 0
    check_pos.pose.orientation.z = 0
    check_pos.pose.orientation.w = 1
    print("Going to new QR code")
    desired_pose = qr_code_util.transform_pose_in_frames(
        check_pos, "odom", "hidden_frame").pose

    # print(desired_pose)
    burger.move_to_pose(desired_pose)

    rospy.signal_shutdown("Mission done")

    qr_code_util.teardown()


if __name__ == "__main__":
    main()

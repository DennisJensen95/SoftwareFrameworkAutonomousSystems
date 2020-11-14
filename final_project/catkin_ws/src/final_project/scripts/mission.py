#!/usr/bin/env python
import rospy
from mission_lib.robot_utilities import BurgerUtility
from mission_lib.qr_code_utilities import QrCodeUtility
import time
from geometry_msgs.msg import Pose


def main():
    rospy.init_node('burger_robot', anonymous=True)

    # Initialize utilities
    qr_code_util = QrCodeUtility()
    burger = BurgerUtility(qr_code_util)

    burger.find_qr_code()
    qr_code_pos = qr_code_util.qr_code_position
    print(qr_code_pos)
    qr_code_pos_odom_frame = qr_code_util.transform_pose_from_map_to_odom(
        qr_code_pos)
    print(qr_code_pos_odom_frame)
    # print(qr_code_pos_odom_frame)
    # qr_code_pos.pose.position.z = 0
    # print(burger.move_to_pose(qr_code_pos.pose))
    # qr_code_util.create_transform_from_odom_to_hidden_frame(
    # burger.robot_pos.pose.pose, qr_code_util.qr_code_position.pose)
    desired_pose = Pose()
    desired_pose.position.x = qr_code_pos_odom_frame.pose.position.x
    desired_pose.position.y = qr_code_pos_odom_frame.pose.position.y
    desired_pose.position.z = 0
    desired_pose.orientation.x = 0
    desired_pose.orientation.y = 0
    desired_pose.orientation.z = 0
    desired_pose.orientation.w = 1
    print(burger.move_to_pose(desired_pose))

    rospy.signal_shutdown("Mission done")


if __name__ == "__main__":
    # try:
    main()
    # except Exception as e:
    #     print(e)
    #     print("Mission failed")
    #     rospy.signal_shutdown("Mission failed")

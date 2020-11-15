#!/usr/bin/env python
import rospy
from mission_lib.robot_utilities import BurgerUtility
from mission_lib.qr_code_utilities import QrCodeUtility
import time
from geometry_msgs.msg import Pose
from tf import Exception

import math


def main():
    rospy.init_node('burger_robot', anonymous=True)

    # Initialize utilities
    qr_code_util = QrCodeUtility()
    burger = BurgerUtility(qr_code_util)

    find_transformation_tries = 2
    found_transformation = False
    while True:
        burger.find_qr_code()
        qr_code_pos = qr_code_util.qr_code_position
        print("Found QR Code")
        for i in range(find_transformation_tries):
            try:
                qr_code_pos_odom_frame = qr_code_util.transform_pose_in_frames(
                    qr_code_pos, '/odom', '/map')

                robot_pose = burger.robot_pos.pose

                desired_pose = burger.get_desired_position_on_qr_code(
                    robot_pose, qr_code_pos_odom_frame)

                found_transformation = True
            except Exception:
                print("Did not find transformation")
                burger.aquire_frame_transformations()
                qr_code_pos = qr_code_util.qr_code_position
                found_transformation = False

        if found_transformation:
            break

    print(desired_pose)
    goal_pose = Pose()
    goal_pose.position.x = desired_pose.pose.position.x
    goal_pose.position.y = desired_pose.pose.position.y
    goal_pose.position.z = 0
    goal_pose.orientation = desired_pose.pose.orientation
    # goal_pose.orientation.x = 0
    # goal_pose.orientation.y = 0
    # goal_pose.orientation.z = 0
    # goal_pose.orientation.w = 1
    # print(qr_code_pos_odom_frame)
    # qr_code_pos.pose.position.z = 0
    # print(burger.move_to_pose(qr_code_pos.pose))
    # qr_code_util.create_transform_from_odom_to_hidden_frame(
    # burger.robot_pos.pose.pose, qr_code_util.qr_code_position.pose)
    print(burger.move_to_pose(goal_pose))

    rospy.signal_shutdown("Mission done")


if __name__ == "__main__":
    # try:
    main()
    # except Exception as e:
    #     print(e)
    #     print("Mission failed")
    #     rospy.signal_shutdown("Mission failed")

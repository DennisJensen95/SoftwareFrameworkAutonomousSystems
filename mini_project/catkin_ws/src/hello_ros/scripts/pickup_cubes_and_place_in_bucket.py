#!/usr/bin/env python
from inspect import getmembers
from lib_mini_project.gripper_lib import move_gripper_to_pose, open_close_gripper
from gazebo_msgs.msg import ModelStates
# import moveit_msgs.msg
# import moveit_commander
import math
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf_conversions
# import copy
# import sys
# import roslib

# Data object keeping track of cubes
positions_cubes = {}
# Static variables for the specific scene
altitude_of_cubes = 0.7
# Stop updating state variable
stop_updating_positions = False

# Bucket pose
bucket_pose = geometry_msgs.msg.Point(x=0.53, y=-0.23, z=0.78)

# Orientations of end effector
downward_orientation = geometry_msgs.msg.Quaternion(
    *tf_conversions.transformations.quaternion_from_euler(0, -math.pi/2, 0.))


def save_cubes_position(model_states):
    if stop_updating_positions:
        return None
    for i, model_name in enumerate(model_states.name):
        if "cube" in model_name:
            positions_cubes.update(
                {model_name: model_states.pose[i].position})


def main_pickup():
    rospy.init_node("pickup_node")
    rospy.Subscriber('/gazebo/model_states', ModelStates,
                     save_cubes_position, queue_size=1000)

    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("Arm")
    print(group.get_joints())

    group.set_goal_orientation_tolerance(0.001)
    group.set_goal_tolerance(0.001)
    group.set_goal_joint_tolerance(0.001)
    group.set_num_planning_attempts(100)

    rospy.sleep(2)

    print(positions_cubes["cube1"])

    move_gripper_to_pose(positions_cubes["cube1"], group, downward_orientation)
    # open_close_gripper(open=True)
    # move_gripper_to_pose(bucket_pose, group, downward_orientation)
    # open_close_gripper(open=False)


if __name__ == "__main__":
    main_pickup()

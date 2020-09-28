#!/usr/bin/env python
import math
from numpy import zeros, array, linspace
from sensor_msgs.msg import JointState
import shape_msgs.msg as shape_msgs
import geometry_msgs.msg
import moveit_msgs.msg
import moveit_commander
import tf_conversions
import rospy
import copy
import sys
import roslib
roslib.load_manifest('hello_ros')

currentJointState = JointState()


def jointStatesCallback(msg):
    global currentJointState
    currentJointState = msg


def open_close_gripper(open=True):
    rospy.init_node('gripper_publisher')
    # Setup subscriber

    # rospy.Subscriber("/joint_states", JointState, jointStatesCallback)

    pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)

    currentJointState = rospy.wait_for_message("/joint_states", JointState)
    print 'Received!'
    currentJointState.header.stamp = rospy.get_rostime()
    if open:
        # Open gripper
        tmp = 0.005
    else:
        # Closed
        tmp = 0.7

    # tmp_tuple=tuple([tmp] + list(currentJointState.position[1:]))
    currentJointState.position = tuple(
        list(currentJointState.position[:6]) + [tmp] + [tmp] + [tmp])
    rate = rospy.Rate(10)  # 10hz
    for i in range(3):
        pub.publish(currentJointState)
        print 'Published!'
        rate.sleep()

    print 'Closed the gripper'


def move_gripper_to_pose(pose, group, orientation):
    pose_goal = group.get_current_pose().pose

    pose_goal.position.x = pose.x
    pose_goal.position.y = pose.y
    pose_goal.position.z = pose.z - 0.10

    # pose_goal.orientation = orientation

    (plan, _) = group.compute_cartesian_path(
        [pose_goal],   # Only one waypoint
        0.01,        # eef_step
        0.0, True)

    group.execute(plan, wait=True)

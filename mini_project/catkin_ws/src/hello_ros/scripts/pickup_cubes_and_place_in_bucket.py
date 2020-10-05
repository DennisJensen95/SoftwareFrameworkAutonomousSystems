#!/usr/bin/env python
from inspect import getmembers
from lib_mini_project.gripper_lib import move_gripper_to_pose, open_close_gripper
from gazebo_msgs.msg import ModelStates
# import moveit_msgs.msg
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

dist_from_origo_bucket_to_middle_bucket = 0.05*2.5

# Bucket pose
bucket_pose = geometry_msgs.msg.Point(x=0, y=0, z=0)

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

        elif "bucket" in model_name:
            bucket_pose.x = model_states.pose[i].position.x
            bucket_pose.y = model_states.pose[i].position.y
            bucket_pose.z = model_states.pose[i].position.z + \
                dist_from_origo_bucket_to_middle_bucket


def add_cubes_to_planning_scene(scene):
    p = geometry_msgs.msg.PoseStamped()
    for key in positions_cubes:
        pose = positions_cubes[key]

        p.pose.position.x = pose.x
        p.pose.position.y = pose.y
        p.pose.position.z = pose.z
        # Making a fire hydrant object
        scene.add_box(key, p, (0.05, 0.05, 0.05))

    return scene


def add_bucket_to_planning_scene(scene):
    p = geometry_msgs.msg.PoseStamped()
    p.pose.position.x = bucket_pose.x
    p.pose.position.y = bucket_pose.y
    p.pose.position.z = bucket_pose.z + dist_from_origo_bucket_to_middle_bucket + 0.1

    scene.add_box("bucket", p, (5*0.05, 5*0.05, 5*0.05))

    return scene


def cube_in_bucket_state(key):
    pose = positions_cubes[key]
    diff = 2*0.05
    x_pos_state = (pose.x > bucket_pose.x -
                   diff and pose.x < bucket_pose.x + diff)
    y_pos_state = (pose.y > bucket_pose.y -
                   diff and pose.y < bucket_pose.y + diff)
    print(pose)
    print(bucket_pose)
    if x_pos_state and y_pos_state:
        return True
    else:
        return False


def main_pickup():
    rospy.init_node("pickup_node")
    rospy.Subscriber('/gazebo/model_states', ModelStates,
                     save_cubes_position, queue_size=1000)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("Arm")
    group.compute_cartesian_path

    print "============ Reference frame: %s" % group.get_planning_frame()

    print "============ Robot Groups:"
    print robot.get_group_names()

    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"

    group.set_goal_orientation_tolerance(0.2)
    group.set_goal_tolerance(0.1)
    group.set_goal_joint_tolerance(0.1)
    group.set_planning_time(5)

    # Let subscriber populate position cubes
    rospy.sleep(5)

    # Adding cubes to the scene
    scene = add_cubes_to_planning_scene(scene)
    scene = add_bucket_to_planning_scene(scene)

    start_pos = group.get_current_pose().pose
    # Loop cubes
    for key in positions_cubes.keys():
        if cube_in_bucket_state(key):
            print "Cube is in the bucket:", key
            continue
        else:
            print "Cube is not in the bucket:", key

        print("Grabbing cube: ", key)
        # Open gripper
        open_close_gripper(open=True)
        rospy.sleep(3)

        # Move gripper to be above cube by 10 cm
        move_to_pose = positions_cubes[key]
        move_to_pose.z = move_to_pose.z + 0.1
        move_gripper_to_pose(move_to_pose, group, downward_orientation)
        rospy.sleep(3)

        # Move gripper to be with grabbing range of the cube
        move_gripper_to_pose(positions_cubes[key], group, downward_orientation)
        rospy.sleep(3)

        # Close the gripper
        open_close_gripper(open=False)
        rospy.sleep(3)

        # Move to bucket
        move_to_bucket = bucket_pose
        move_to_bucket.z = move_to_bucket.z + 0.20
        move_gripper_to_pose(move_to_bucket, group, downward_orientation)
        rospy.sleep(3)

        # Throw cube in bucket
        open_close_gripper(open=True)
        rospy.sleep(3)

    # Back to starting position
    move_gripper_to_pose(start_pos.position, group, start_pos.orientation)


if __name__ == "__main__":
    main_pickup()

#!/usr/bin/env python
import random
import rospy
import tf
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Duration

if __name__ == '__main__':
    rospy.init_node('publish_random_trajectory')
    p = rospy.Publisher('/arm_controller/command',
                        JointTrajectory, queue_size=1000)

    rate = rospy.Rate(0.3)

    while not rospy.is_shutdown():
        stance = JointTrajectory()
        point = JointTrajectoryPoint()
        angles = {"hip": random.uniform(-math.pi/2, math.pi/2), "shoulder": random.uniform(
            -math.pi/2, math.pi/2), "elbow": random.uniform(-math.pi/2, math.pi/2)}

        stance.joint_names = list(angles.keys())
        point.positions = [angles["hip"], angles["shoulder"], angles["elbow"]]
        point.time_from_start = rospy.Duration(secs=1)
        stance.points.append(point)

        p.publish(stance)
        rate.sleep()

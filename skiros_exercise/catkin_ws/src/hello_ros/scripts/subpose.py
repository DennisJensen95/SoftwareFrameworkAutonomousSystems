#!/usr/bin/env python
# This program publishes randomly-generated velocity
# messages for turtlesim.
import rospy
import numpy as np  # For random numbers

from std_msgs.msg import String
from turtlesim import msg  # we need to import hte turtlesim msgs in order to use them

# Create callback. This is what happens when a new message is received


def sub_cal(msg):
    rospy.loginfo("position=( %f, %f), direction= %f", msg.x, msg.y, msg.theta)
    rospy.loginfo("Linear velocity=%f angular velocity=%f",
                  msg.linear_velocity, msg.angular_velocity)


# Initialize publisher
rospy.Subscriber('turtle1/pose', msg.Pose, sub_cal, queue_size=1000)

# Initialize node
rospy.init_node('vel_listener')
rospy.spin()
rospy.loginfo()

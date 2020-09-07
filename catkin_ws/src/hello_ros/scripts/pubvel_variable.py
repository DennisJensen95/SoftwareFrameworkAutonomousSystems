#!/usr/bin/env python
# This program publishes randomly-generated velocity
# messages for turtlesim.
from geometry_msgs.msg import Twist  # For geometry_msgs/Twist
from std_msgs.msg import String
from hello_ros.msg import turtle  # For geometry_msgs/Twist
import rospy
import numpy as np  # For random numbers
global speed_var
speed_var = 0.5


# Create callback. This is what happens when a new message is received


def sub_cal(msg):
    rospy.loginfo("name= %s, speed= %f", msg.name, msg.speed)
    global speed_var
    speed_var = msg.speed


# Initialize publisher
p = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1000)
# Initialise subscriber
rospy.Subscriber('turtle1/turtle_speed', turtle, sub_cal, queue_size=1000)


# Initialize node
rospy.init_node('publish_variable_velocity')
r = rospy.Rate(2)  # Set Frequency

# loop until someone shutds us down..
while not rospy.is_shutdown():

    # Initiate Message with zero values
    t = Twist()
    #Fill in message
    t.angular.z = (2*np.random.rand()-1)*speed_var
    t.linear.x = (np.random.rand())*speed_var

    # ROS INFO of the commands
    #rospy.loginfo("Ang.z is%s" % t.angular.z)
    #rospy.loginfo("Lin.x is%s" % t.linear.x)

    # publish the message
    p.publish(t)
    r.sleep()

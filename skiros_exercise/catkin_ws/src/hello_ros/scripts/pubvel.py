#!/usr/bin/env python
# This program publishes randomly-generated velocity
# messages for turtlesim.
import rospy
import numpy as np  # For random numbers


from std_msgs.msg import String
from geometry_msgs.msg import Twist  # For geometry_msgs/Twist

# Initialize publisher
p = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1000)

# Initialize node
rospy.init_node('publish_velocity')
r = rospy.Rate(2)  # Set Frequency

# loop until someone shutds us down..
while not rospy.is_shutdown():
    # Initiate Message with zero values
    t = Twist()
    #Fill in message
    t.angular.z = 2
    t.linear.x = np.random.rand()

    # ROS INFO of the commands
    rospy.loginfo("Ang.z is%s" % t.angular.z)
    rospy.loginfo("Lin.x is%s" % t.linear.x)

    # publish the message
    p.publish(t)
    r.sleep()

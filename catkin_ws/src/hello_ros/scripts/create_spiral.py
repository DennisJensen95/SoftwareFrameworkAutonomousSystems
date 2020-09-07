#!/usr/bin/env python
# This application makes the turtle spiral and resets everytime it hits a wall
from hello_ros.msg import turtle
from geometry_msgs.msg import Twist  # For geometry_msgs/Twist
from turtlesim.srv import SetPen, TeleportAbsolute
from turtlesim.msg import Pose
from std_msgs.msg import String
import rospy
import numpy as np  # For random numbers
speed_var = 1.0
tmp_speed = speed_var
rospy.set_param('speed_of_turtle', speed_var)

global reset_state
global pose_rec
reset_state = False

# Pen color
pen_color = [0, 0, 0]
rospy.set_param('pen_color', pen_color)


def set_pen_color(des_color_pen):
    rospy.wait_for_service('/turtle1/set_pen')

    set_pen_yay = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
    resp1 = set_pen_yay(
        des_color_pen[0], des_color_pen[1], des_color_pen[2], 2, False)


def reset_turtle(initial_pose):
    rospy.wait_for_service('/turtle1/teleport_absolute')
    try:
        reset = rospy.ServiceProxy(
            '/turtle1/teleport_absolute', TeleportAbsolute)
        resp = reset(initial_pose.x, initial_pose.y,
                     np.random.uniform(0, 6.28))
        return True
    except rospy.ServiceException:
        print("Service call failed:")
        return False


def check_if_wall_detected(pose):
    global pose_rec
    global reset_state
    pose_rec = pose
    if (pose.x > 11 or pose.x < 0.1):
        reset_state = True
    elif (pose.y > 11 or pose.y < 0):
        reset_state = True


        # Initialize publisher
p = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1000)

rospy.Subscriber('turtle1/pose', Pose, check_if_wall_detected, queue_size=1000)

# Initialize node
rospy.init_node('create_spiral')
r = rospy.Rate(2)  # Set Frequency

# loop until someone shutds us down..
iterator = 0
a = 0.4
b = 1
while not rospy.is_shutdown():
    if iterator == 0:
        initial_pose = pose_rec

    iterator += 1
    if reset_state:
        reset_turtle(initial_pose)
        iterator = 0
        reset_state = False
    # Initiate Message with zero values
    t = Twist()
    # Set pen color

    des_color_pen = rospy.get_param('pen_color')
    set_pen_color(des_color_pen)

    #Fill in message
    speed_of_turtle = rospy.get_param('speed_of_turtle')

    t.angular.z = 0.6*speed_of_turtle
    t.linear.x = a + b*0.6*speed_of_turtle

    # publish the message
    p.publish(t)
    r.sleep()

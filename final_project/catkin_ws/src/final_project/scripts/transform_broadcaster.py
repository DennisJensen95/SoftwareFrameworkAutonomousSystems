#!/usr/bin/env python

import rospy

import math
import tf2_ros
from geometry_msgs.msg import *
import tf_conversions
import tf


def main():
    # Creating broadcaster node
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    # Creating frames
    frame_transform = geometry_msgs.msg.TransformStamped()
    frame_transform.header.frame_id = "robot"
    frame_transform.child_frame_id = "qr_code"

    # Hardcoded from Gazebo....
    frame_transform.transform.translation.x = -2.97 - 0.1
    frame_transform.transform.translation.y = -0.4 - 3.5
    frame_transform.transform.translation.z = 0
    # No rotation

    frame_transform.transform.rotation.x = 0
    frame_transform.transform.rotation.y = 0
    frame_transform.transform.rotation.z = 0
    frame_transform.transform.rotation.w = 1

    while not rospy.is_shutdown():
        br.sendTransform((-2.97 - 0.1, -0.4 - 3.5, 0), (0, 0, 0, 1), rospy.Time.now(),
                         frame_transform.child_frame_id, frame_transform.header.frame_id)

        rate.sleep()


if __name__ == "__main__":
    main()

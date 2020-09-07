#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String


def hello_ros():
    rospy.init_node('hello_ros', anonymous=True)
    rate = rospy.Rate(5)  # 10hz
    hello_str = "hello ros!!! %s" % rospy.get_time()
    while not rospy.is_shutdown():
        rospy.loginfo(hello_str)
        rate.sleep()


if __name__ == '__main__':
    try:
        hello_ros()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def network():
    rospy.init_node("network")
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        print "test"
        rate.sleep()

if __name__ == "__main__":
    try:
        network()
    except rospy.ROSInterruptException:
        pass
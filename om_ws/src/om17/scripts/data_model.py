#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def data_model():
    rospy.init_node("data_model")
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        print "test"
        rate.sleep()

if __name__== '__main__':
    try:
        data_model()
    except rospy.ROSInterruptException:
        pass

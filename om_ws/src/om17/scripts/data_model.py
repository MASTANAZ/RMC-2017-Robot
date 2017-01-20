#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D

def data_model():
    rospy.init_node("data_model")
    pub = rospy.Publisher("poser", Pose2D, queue_size=10)
    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        position = Pose2D()
        position.x = 2.34
        position.y = 1.45
        position.theta = 2
        pub.publish(position)
        rate.sleep()

if __name__== '__main__':
    try:
        data_model()
    except rospy.ROSInterruptException:
        pass

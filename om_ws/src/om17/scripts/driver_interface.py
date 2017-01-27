#!/usr/bin/env python

# DRIVER_INTERFACE
#
# CREATED BY HARRIS NEWSTEDER
#
# DESCRIPTION:
# 

################################################################################
# IMPORTS
################################################################################

import sys
import json
import serial

import rospy
from std_msgs.msg import String

################################################################################
# MODULE VARIABLES
################################################################################

_arduino = serial.Serial

################################################################################
# PUBLIC FUNCTIONS
################################################################################

def driver_interface():
    rospy.init_node("driver_interface")
    rate = rospy.Rate(3)
    _init()
    while not rospy.is_shutdown():
        _tick()
        rate.sleep()

################################################################################
# PRIVATE FUNCTIONS
################################################################################

def _init():
    pass
    
def _tick():
    pass

def _cleanup():
    pass

################################################################################
# ENTRY POINT
################################################################################

if __name__ == "__main__":
    try:
        driver_interface()
    except rospy.ROSInterruptException:
        pass
        
    _cleanup()


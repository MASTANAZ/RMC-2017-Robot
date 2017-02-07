#!/usr/bin/env python

# WATCHDOG
#
# CREATED BY HARRIS NEWSTEDER
#
# DESCRIPTION:
# 

################################################################################
# IMPORTS
################################################################################

import rospy

################################################################################
# CONSTANTS
################################################################################

################################################################################
# MODULE VARIABLES
################################################################################

################################################################################
# PUBLIC FUNCTIONS
################################################################################

def watchdog():
    rospy.init_node("watchdog")
    rate = rospy.Rate(3)
    _init()
    while not rospy.is_shutdown():
        _tick()
        rate.sleep()

################################################################################
# PRIVATE FUNCTIONS
################################################################################

def _stop_callback(event):
    rospy.loginfo("STOP")
    

def _init():
    rospy.loginfo("INITIALIZING WATCHDOG")
    rospy.Timer(rospy.Duration(2), _stop_callback)
    

def _tick():
    # receive info from the mission control server on a timeout
    pass

def _cleanup():
    rospy.loginfo("STOPPING WATCHDOG")


################################################################################
# ENTRY POINT
################################################################################
        
if __name__ == "__main__":
    try:
        watchdog()
    except rospy.ROSInterruptException:
        pass
        
    _cleanup()
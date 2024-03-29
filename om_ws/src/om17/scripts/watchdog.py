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

from random import randint

from om17.msg import CellCost

################################################################################
# CONSTANTS
################################################################################

################################################################################
# MODULE VARIABLES
################################################################################

_cc_pub = rospy.Publisher("/world_cost", CellCost, queue_size=10)

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

def _init():
    rospy.loginfo("INITIALIZING WATCHDOG")
    

def _tick():
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

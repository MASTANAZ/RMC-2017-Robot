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
    rate = rospy.Rate(0.25)
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
    test = CellCost()
    test.x = 5
    test.y = 5
    test.cost = -1
    _cc_pub.publish(test)
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

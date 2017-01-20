#!/usr/bin/env python

################################################################################
# IMPORTS
################################################################################

import sys
import json

import rospy
from std_msgs.msg import String

################################################################################
# PUBLIC FUNCTIONS
################################################################################

def robot_info():
    rospy.init_node("robot_info")
    
    print "> INITIALIZING ROBOT INFORMATION"

    jsonFilePath = rospy.myargv(argv=sys.argv)[1]

    print "> ATTEMPTING TO READ ROBOT INFORMATION FROM \"" + jsonFilePath + "\""
    
    try:
        with open(jsonFilePath) as jsonFile:
            jsonData = json.load(jsonFile)
        
            self_id = jsonData["SELF_ID"]
            self_ip = jsonData["SELF_IP"]
            other_ip = jsonData["OTHER_IP"]
            mc_ip = jsonData["MC_IP"]
        
            rospy.set_param("om/robot/self_id", self_id)
            rospy.set_param("om/network/self_ip", self_ip)
            rospy.set_param("om/network/other_ip", other_ip)
            rospy.set_param("om/network/mc_ip", mc_ip)
        
            print "> SELF ID  = " + str(self_id)
            print "> SELF IP  = " + jsonData["SELF_IP"]
            print "> OTHER IP = " + jsonData["OTHER_IP"]
            print "> MC IP    = " + jsonData["MC_IP"]
            print "> SUCCESS"
            jsonFile.close()
    except:
        print "! ERROR: FAILED"
        sys.exit()

if __name__ == "__main__":
    try:
        robot_info()
    except rospy.ROSInterruptException:
        pass


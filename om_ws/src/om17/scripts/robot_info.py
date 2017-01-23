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
    
    rospy.loginfo("INITIALIZING ROBOT INFORMATION")

    jsonFilePath = rospy.myargv(argv=sys.argv)[1]

    rospy.loginfo("ATTEMPTING TO READ ROBOT INFORMATION FROM \"" + jsonFilePath + "\"")
    
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
        
            rospy.loginfo("SELF ID  = " + str(self_id))
            rospy.loginfo("SELF IP  = " + jsonData["SELF_IP"])
            rospy.loginfo("OTHER IP = " + jsonData["OTHER_IP"])
            rospy.loginfo("MC IP    = " + jsonData["MC_IP"])
            rospy.loginfo("SUCCESS")
            jsonFile.close()
    except:
        rospy.logfatal("ERROR: FAILED")
        sys.exit()

if __name__ == "__main__":
    try:
        robot_info()
    except rospy.ROSInterruptException:
        pass


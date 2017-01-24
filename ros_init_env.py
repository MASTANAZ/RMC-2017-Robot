#!/usr/bin/env python

################################################################################
# IMPORTS
################################################################################

from subprocess import call
import json
import os

################################################################################
# ENTRY POINT
################################################################################

if __name__=="__main__":
    print "SETTING UP ENVIRONMENT VARIABLES FOR ROS"
    robot_info_path = os.getenv("HOME") + "/robot_info.json"
    try:
        with open(robot_info_path) as json_file:
            data = json.load(json_file)
            print "export ROS_IP=http://" + data["ip"] + ":11311"
            call("export ROS_IP=http://" + data["ip"] + ":11311", shell=True)
            print "export ROS_MASTER_URI=http://" + data["master"] + ":11311"
            call("export ROS_MASTER_URI=http://" + data["master"] + ":11311", shell=True)
            print "SUCCESS"
    except:
        print "FAILED TO LOAD " + robot_info_path

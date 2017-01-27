#!/usr/bin/env python

# NETWORK
#
# CREATED BY HARRIS NEWSTEDER
#
# DESCRIPTION:
# 

################################################################################
# IMPORTS
################################################################################

import sys
import socket
import time
import math
import random
import json
import os

import rospy
from geometry_msgs.msg import Pose2D

################################################################################
# CONSTANTS
################################################################################

_S_END             = 0xFF

_S_P_X             = 0x01
_S_P_Y             = 0x02
_S_P_ORIENTATION   = 0x03

# mission control communication parameters
_MC_PORT         = 12000
_MC_BUFFER_SIZE  = 1024
_MC_RECV_TIMEOUT = 0.05
_MC_IP           = rospy.get_param(os.environ["ROS_NAMESPACE"] + "/mc_ip")

_CONNECTION_KEY = "@"
_CONFIRMATION_KEY = "!"

################################################################################
# MODULE VARIABLES
################################################################################

_mc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
_mc.settimeout(1)

_mc_pending = ""

_mc_connected = False

################################################################################
# PUBLIC FUNCTIONS
################################################################################

def callback(data):
    global _mc_pending
    major = math.floor(data.x)
    minor = math.floor((data.x - major) * 100)
    _mc_pending += chr(_S_P_X)
    _mc_pending += chr(int(minor))
    _mc_pending += chr(int(major))
    _mc_pending += chr(_S_END)
    major = math.floor(data.y)
    minor = math.floor((data.y - major) * 100)
    _mc_pending += chr(_S_P_Y)
    _mc_pending += chr(int(minor))
    _mc_pending += chr(int(major))
    _mc_pending += chr(_S_END)
    major = math.floor(data.theta)
    minor = math.floor((data.theta - major) * 100)
    _mc_pending += chr(_S_P_ORIENTATION)
    _mc_pending += chr(int(minor))
    _mc_pending += chr(int(major))
    _mc_pending += chr(_S_END)

def network():
    rospy.init_node("network")
    rate = rospy.Rate(3)
    rospy.Subscriber("pose", Pose2D, callback)
    _init()
    while not rospy.is_shutdown():
        _tick()
        rate.sleep()

################################################################################
# PRIVATE FUNCTIONS
################################################################################

def _init():
    rospy.loginfo("INITIALIZING MISSION CONTROL CONNECTION")
    _connect() 

def _tick():
    if not _mc_connected: return

    global _mc_pending
    
    if len(_mc_pending) > 0:
        _mc.send(_mc_pending)
        _mc_pending = ""
    
    # receive info from the mission control server on a timeout
    pass

def _connect():
    global _mc, _mc_connected

    rospy.loginfo("ATTEMPTING TO CONNECT TO MISSION CONTROL AT " + _MC_IP)

    try:
        _mc.connect((_MC_IP, _MC_PORT))
        _mc.setblocking(0)
        
        rospy.loginfo("SENDING CONNECTION KEY TO MISSION CONTROL")
        _mc.send(_CONNECTION_KEY)
        
        # wait for mc to process connection key and send back a confirmation
        time.sleep(0.5)
        
        confirmation = _mc.recv(1024)
        
        if confirmation == _CONFIRMATION_KEY:
            rospy.loginfo("CONFIRMATION KEY RECEIVED FROM MISSION CONTROL")
            rospy.loginfo("SUCCESSFULLY CONNECTED")
            
            _mc_connected = True
        else:
            rospy.logfatal("FAILED TO RECEIVE CONFIRMATION KEY FROM MISSION CONTROL")
    except:
        rospy.logfatal("FAILED TO CONNECT TO MISSION CONTROL")

def _cleanup():
    rospy.loginfo("DISCONNECTING FROM MISSION CONTROL")
    _mc.close()


################################################################################
# ENTRY POINT
################################################################################
        
if __name__ == "__main__":
    try:
        network()
    except rospy.ROSInterruptException:
        pass
        
    _cleanup()

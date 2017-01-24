#!/usr/bin/env python

################################################################################
# IMPORTS
################################################################################

import sys
import socket
import time
import math
import random
import json

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
_MC_IP           = rospy.get_param("/om/robot/mc_ip")

_CONNECTION_KEY = "@"
_CONFIRMATION_KEY = "!"

################################################################################
# MODULE VARIABLES
################################################################################

_mc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
_mc.settimeout(1)

_mc_pending = ""

_mc_connected = False

_rb = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
_rb.settimeout(1)

_rb_pending = ""

_rb_hosting = False
_rb_connected = False

_rb_client = None

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
    data.y = data.y + 1.89
    major = math.floor(data.y)
    minor = math.floor((data.y - major) * 100)
    _mc_pending += chr(_S_P_Y)
    _mc_pending += chr(int(minor))
    _mc_pending += chr(int(major))
    _mc_pending += chr(_S_END)
    data.theta += 6.2832
    major = math.floor(data.theta)
    minor = math.floor((data.theta - major) * 100)
    _mc_pending += chr(_S_P_ORIENTATION)
    _mc_pending += chr(int(minor))
    _mc_pending += chr(int(major))
    _mc_pending += chr(_S_END)

def network():
    rospy.init_node("network")
    rate = rospy.Rate(3)
    rospy.Subscriber("poser", Pose2D, callback)
    _init()
    while not rospy.is_shutdown():
        _tick()
        rate.sleep()

################################################################################
# PRIVATE FUNCTIONS
################################################################################

def _init():
    _mc_init()  

def _tick():
    _mc_tick()

def _mc_init():
    rospy.loginfo("INITIALIZING MISSION CONTROL CONNECTION")
    _mc_connect()

def _mc_tick():
    if not _mc_connected: return

    global _mc_pending
    
    if len(_mc_pending) > 0:
        _mc.send(_mc_pending)
        _mc_pending = ""
    
    # receive info from the mission control server on a timeout
    pass
    
def _mc_connect():
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
        
################################################################################
# ENTRY POINT
################################################################################
        
if __name__ == "__main__":
    try:
        network()
    except rospy.ROSInterruptException:
        _mc.close()
        _rb.close()

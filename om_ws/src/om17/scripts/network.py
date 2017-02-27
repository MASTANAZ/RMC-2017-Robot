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
import os
import select

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int16

################################################################################
# CONSTANTS
################################################################################

_S_END             = 0xFF

_S_P_X             = 0x01
_S_P_Y             = 0x02
_S_P_ORIENTATION   = 0x03
_S_P_LCV           = 0x04
_S_P_RCV           = 0x05

# mission control communication parameters
_MC_PORT           = 12000
_MC_BUFFER_SIZE    = 1024
_MC_RECV_TIMEOUT   = 0.05
_MC_IP             = rospy.get_param(os.environ["ROS_NAMESPACE"] + "/mc_ip")

_CONNECTION_KEY    = "@"
_CONFIRMATION_KEY  = "!"

################################################################################
# MODULE VARIABLES
################################################################################

_mc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
_mc.settimeout(1)

_mc_pending    = ""
_mc_to_process = ""

_mc_connected  = False

_lcv_publisher = rospy.Publisher('lcv', Int16, queue_size=10)
_rcv_publisher = rospy.Publisher('rcv', Int16, queue_size=10)

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

    global _mc_pending, _mc_to_process
    
    # push all data to mission control
    if len(_mc_pending) > 0:
        _mc.send(_mc_pending)
        _mc_pending = ""
    
    # receive info from the mission control server on a timeout
    try:
        ready = select.select([_mc], [], [], _MC_RECV_TIMEOUT)

        if ready:
            data = _mc.recv(1024)
            if not data:
                sys.exit()
            else:
                _mc_to_process += data
    except:
        pass

    # parse data incoming from mission control
    _parse_incoming()

def _connect():
    global _mc, _mc_connected

    rospy.loginfo("ATTEMPTING TO CONNECT TO MISSION CONTROL AT " + _MC_IP)

    try:
        _mc.connect((_MC_IP, _MC_PORT))
        
        rospy.loginfo("SENDING CONNECTION KEY TO MISSION CONTROL")
        _mc.send(_CONNECTION_KEY)
        
        # wait for mc to process connection key and send back a confirmation
        time.sleep(0.5)
        
        confirmation = _mc.recv(1024)

        if confirmation[0] == _CONFIRMATION_KEY:
            rospy.loginfo("CONFIRMATION KEY RECEIVED FROM MISSION CONTROL")
            rospy.loginfo("SUCCESSFULLY CONNECTED")
            
            _mc_connected = True
        else:
            rospy.logfatal("FAILED TO RECEIVE CONFIRMATION KEY FROM MISSION CONTROL")
    except:
        rospy.logfatal("FAILED TO CONNECT TO MISSION CONTROL")

def _parse_incoming():
    global _mc_to_process

    plen = len(_mc_to_process)

    # there is no data to process, skip
    if (plen == 0):
        return

    # process then entire string byte by byte
    while (plen > 2):
        # int value of the current byte being processed
        cbval = ord(_mc_to_process[0])
        # remaining length of the data string
        plen = len(_mc_to_process)
        # essentially a giant switch statement for all available data that
        # the mission control can send to the robot
        if cbval == _S_P_LCV:
            if plen >= 3:
                _lcv_publisher.publish(ord(_mc_to_process[1]) - 100)
                print "lcv: " + str(ord(_mc_to_process[1]) - 100)
                _mc_to_process = _mc_to_process[2:]
        elif cbval == _S_P_RCV:
            if plen >= 3:
                _rcv_publisher.publish(ord(_mc_to_process[1]) - 100)
                print "rcv: " + str(ord(_mc_to_process[1]) - 100)
                _mc_to_process = _mc_to_process[2:]
        # discard the current byte and continue processing the rest of the
        # string        
        else:
            _mc_to_process = _mc_to_process[1:]

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

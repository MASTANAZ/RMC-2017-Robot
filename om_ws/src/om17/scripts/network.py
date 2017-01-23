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
_MC_IP           = None

# robot communication parameters
_RB_PORT         = 12001
_RB_BUFFER_SIZE  = 512
_RB_RECV_TIMEOUT = 0.05
_RB_OTHER_IP     = None
_RB_SELF_IP      = None

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
    global _MC_IP, _RB_SELF_IP, _RB_OTHER_IP
    try:
        with open(rospy.get_param("/robot/info_file")) as data_file:
            data = json.load(data_file)
            
            _MC_IP       = data["MC_IP"]
            _RB_OTHER_IP = data["OTHER_IP"]
            _RB_SELF_IP  = data["SELF_IP"]
    except:
        rospy.logfatal("FAILED TO READ " + rospy.get_param("/robot/info_file"))
        
    _mc_init()
    _rb_init()
            

def _tick():
    _mc_tick()
    _rb_tick()

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
    
def _rb_init():
    rospy.loginfo("INITIALIZING ROBOT COMMUNICATIONS")
    rospy.loginfo("SEARCHING FOR HOST ON LAN")

    tmp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tmp.settimeout(1)

    host_found = False

    try:
        tmp.connect((_RB_OTHER_IP, _RB_PORT))
        rospy.loginfo("ROBOT HOST FOUND")
        host_found = True
        
        # sleep so the host can fail the tmp client for the connection handshake
        time.sleep(1)
    except:
        rospy.loginfo("NO HOST FOUND FOR ROBOT COMMUNICATION")

    tmp.close()

    if host_found:
        _rb_connect()
    else:
        _rb_host()

def _rb_tick():
    global _rb_pending

    if _rb_hosting:
        # no client has connected yet
        if not _rb_connected:
            try:
                conn, addr = _rb.accept()
                _rb_process_client(conn)
            except:
                pass
        else:
            pass
    # this robot is the client to the other robot's server
    else:
        if not _rb_connected: return
        
        if len(_rb_pending) > 0:
            _rb.send(_rb_pending)
            _rb_pending = ""

def _rb_connect():
    global _rb, _rb_connected

    rospy.loginfo("ATTEMPTING TO CONNECT TO ROBOT HOST AT " + _RB_OTHER_IP)

    try:
        _rb.connect((_RB_OTHER_IP, _RB_PORT))
        
        rospy.loginfo("SENDING CONNECTION KEY")
        _rb.send(_CONNECTION_KEY)
        
        # wait for host to process connection key and send back a confirmation
        time.sleep(0.5)
        
        confirmation = _rb.recv(16)
        
        if confirmation == _CONFIRMATION_KEY:
            rospy.loginfo("CONFIRMATION KEY RECEIVED FROM HOST")
            rospy.loginfo("SUCCESSFULLY CONNECTED TO ROBOT HOST")
            
            _rb_connected = True
        else:
            rospy.logfatal("FAILED TO RECEIVE CONFIRMATION KEY FROM ROBOT HOST")
    except:
        rospy.logfatal("FAILED TO CONNECT TO ROBOT HOST")
    
    # TODO: keep retrying connection

def _rb_host():
    global _rb, _rb_hosting

    rospy.loginfo("ATTEMPTING TO CREATE ROBOT SERVER AT " + _RB_SELF_IP)

    try:
        _rb.bind((_RB_SELF_IP, _RB_PORT))
        _rb.listen(1)
        
        rospy.loginfo("SUCCESFULLY CREATED ROBOT SERVER AT " + _RB_SELF_IP)
        
        _rb_hosting = True
    except:
        rospy.logfatal("FAILED TO CREATE ROBOT SERVER")

def _rb_process_client(client):
    global _rb_client, _rb_connected

    rospy.loginfo("ATTEMPTING TO PROCESS NEW CLIENT")

    # wait for the connecting client to send the connection key
    time.sleep(0.25)

    try:
        key = client.recv(16)
    except:
        rospy.logwarning("NO CONNECTION KEY RECEIVED")
        rospy.logwarning("TERMINATING CONNECTION")
        
        client.close()
        
        return

    if key == _CONNECTION_KEY:
        rospy.loginfo("CONNECTION KEY ACCEPTED")
        rospy.loginfo("SENDING CONFIRMATION KEY TO CLIENT")
        
        client.send(_CONFIRMATION_KEY)
        
        _rb_client = client
        _rb_connected = True
        
        rospy.loginfo("CLIENT SUCCESSFULLY ADDED")
    else:
        rospy.logwarning("UNKNOWN CLIENT")
        rospy.logwarning("TERMINATING CONNECTION")
        
        client.close()
        
################################################################################
# ENTRY POINT
################################################################################
        
if __name__ == "__main__":
    try:
        network()
    except rospy.ROSInterruptException:
        _mc.close()
        _rb.close()

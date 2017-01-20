#!/usr/bin/env python

################################################################################
# IMPORTS
################################################################################

import sys
import socket
import time
import math
import random

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
_MC_IP           = rospy.get_param("/om/network/mc_ip")

# robot communication parameters
_RB_PORT         = 12001
_RB_BUFFER_SIZE  = 512
_RB_RECV_TIMEOUT = 0.05
_RB_OTHER_IP     = rospy.get_param("/om/network/other_ip")
_RB_SELF_IP      = rospy.get_param("/om/network/self_ip")

_CONNECTION_KEY = "@"
_CONFIRMATION_KEY = "!"

################################################################################
# MODULE VARIABLES
################################################################################

_mc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
_mc.settimeout(1)

_mcPending = ""

_mcConnected = False

_rb = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
_rb.settimeout(1)

_rbPending = ""

_rbHosting = False
_rbConnected = False

_rbClient = None

################################################################################
# PUBLIC FUNCTIONS
################################################################################

def callback(data):
    global _mcPending
    major = math.floor(data.x)
    minor = math.floor((data.x - major) * 100)
    _mcPending += chr(_S_P_X)
    _mcPending += chr(int(minor))
    _mcPending += chr(int(major))
    _mcPending += chr(_S_END)
    major = math.floor(data.y)
    minor = math.floor((data.y - major) * 100)
    _mcPending += chr(_S_P_Y)
    _mcPending += chr(int(minor))
    _mcPending += chr(int(major))
    _mcPending += chr(_S_END)

def network():
    rospy.init_node("network")
    _mc_init()
    rate = rospy.Rate(3)
    rospy.Subscriber("poser", Pose2D, callback)
    while not rospy.is_shutdown():
        _tick()
        rate.sleep()

################################################################################
# PRIVATE FUNCTIONS
################################################################################

def _tick():
    _mc_tick()
    _rb_tick()

def _mc_init():
    print "> INITIALIZING MISSION CONTROL CONNECTION"
    _mc_connect()

def _mc_tick():
    if not _mcConnected: return

    global _mcPending
    
    if len(_mcPending) > 0:
        _mc.send(_mcPending)
        _mcPending = ""
    
    # receive info from the mission control server on a timeout
    pass
    
def _mc_connect():
    global _mc, _mcConnected

    print "> ATTEMPTING TO CONNECT TO MISSION CONTROL AT " + _MC_IP

    try:
        _mc.connect((_MC_IP, _MC_PORT))
        _mc.setblocking(0)
        
        print "> SENDING CONNECTION KEY"
        _mc.send(_CONNECTION_KEY)
        
        # wait for mc to process connection key and send back a confirmation
        time.sleep(0.5)
        
        confirmation = _mc.recv(1024)
        
        if confirmation == _CONFIRMATION_KEY:
            print "> CONFIRMATION KEY RECEIVED FROM MISSION CONTROL"
            print "> SUCCESS"
            
            _mcConnected = True
        else:
            print "! ERROR: FAILED"
    except:
        print "! ERROR: FAILED"
    
def _rb_init():
    print "> INITIALIZING ROBOT COMMUNICATIONS"
    print "> SEARCHING FOR HOST ON LAN"

    tmp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tmp.settimeout(1)

    hostFound = False

    try:
        tmp.connect((_RB_OTHER_IP, _RB_PORT))
        print "> ROBOT HOST FOUND"
        hostFound = True
        
        # sleep so the host can fail the tmp client for the connection handshake
        time.sleep(1)
    except:
        print "> NO HOST FOUND FOR ROBOT COMMUNICATION"

    tmp.close()

    if hostFound:
        _rb_connect()
    else:
        _rb_host()

def _rb_tick():
    global _rbPending

    if _rbHosting:
        # no client has connected yet
        if not _rbConnected:
            try:
                conn, addr = _rb.accept()
                _rb_process_client(conn)
            except:
                pass
        else:
            pass
    # this robot is the client to the other robot's server
    else:
        if not _rbConnected: return
        
        if len(_rbPending) > 0:
            _rb.send(_rbPending)
            _rbPending = ""

def _rb_connect():
    global _rb, _rbConnected

    print "> ATTEMPTING TO CONNECT TO ROBOT HOST AT " + _RB_OTHER_IP

    try:
        _rb.connect((_RB_OTHER_IP, _RB_PORT))
        
        print "> SENDING CONNECTION KEY"
        _rb.send(_CONNECTION_KEY)
        
        # wait for host to process connection key and send back a confirmation
        time.sleep(0.5)
        
        confirmation = _rb.recv(16)
        
        if confirmation == _CONFIRMATION_KEY:
            print "> CONFIRMATION KEY RECEIVED FROM HOST"
            print "> SUCCESS"
            
            _rbConnected = True
        else:
            print "! ERROR: FAILED"
    except:
        print "! ERROR: FAILED"
    
    # TODO: keep retrying connection

def _rb_host():
    global _rb, _rbHosting

    print "> ATTEMPTING TO CREATE ROBOT SERVER AT " + _RB_SELF_IP

    try:
        _rb.bind((_RB_SELF_IP, _RB_PORT))
        _rb.listen(1)
        
        print "> SUCCESS"
        
        _rbHosting = True
    except:
        print "! ERROR: FAILED"

def _rb_process_client(client):
    global _rbClient, _rbConnected

    print "> ATTEMPTING TO PROCESS NEW CLIENT"

    # wait for the connecting client to send the connection key
    time.sleep(0.25)

    try:
        key = client.recv(16)
    except:
        print "! ERROR: NO CONNECTION KEY RECEIVED"
        print "> TERMINATING CONNECTION"
        
        client.close()
        
        return

    if key == _CONNECTION_KEY:
        print "> CONNECTION KEY ACCEPTED"
        print "> SENDING CONFIRMATION KEY TO CLIENT"
        
        client.send(_CONFIRMATION_KEY)
        
        _rbClient = client
        _rbConnected = True
        
        print "> CLIENT SUCCESSFULLY ADDED"
    else:
        print "! ERROR: UNKNOWN CLIENT"
        print "> TERMINATING CONNECTION"
        
        client.close()
        
################################################################################
# ENTRY POINT
################################################################################
        
if __name__ == "__main__":
    try:
        network()
    except rospy.ROSInterruptException:
        pass
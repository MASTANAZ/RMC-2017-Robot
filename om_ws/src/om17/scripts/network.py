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
from std_msgs.msg import Int8
from std_msgs.msg import Bool

from om17.msg import CellCost

################################################################################
# CONSTANTS
################################################################################

_S_POSE            = 0x01
_S_MC1             = 0x02
_S_MC2             = 0x03
_S_CELL_COST       = 0x04
_S_CPU_TEMP        = 0x05
_S_STARTING_PARAMS = 0x06
_S_ROUND_START     = 0x07
_S_ROUND_STOP      = 0x08
_S_AUTONOMY_STOP   = 0x09
_S_AUTONOMY_START  = 0x0A
_S_STATE           = 0x0B
_S_CONTROL_STATE   = 0x0C

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

_mc1_pub             = rospy.Publisher("mc1", Int16, queue_size=10)
_mc2_pub             = rospy.Publisher("mc2", Int16, queue_size=10)
_round_active_pub    = rospy.Publisher("/round_active", Bool, queue_size = 10)
_autonomy_active_pub = rospy.Publisher("/autonomy_active", Bool, queue_size = 10)
_control_state_pub   = rospy.Publisher("control_state", Int8, queue_size = 10)

################################################################################
# PUBLIC FUNCTIONS
################################################################################

def state_callback(msg):
    global _mc_pending
    _mc_pending += chr(_S_STATE)
    _mc_pending += chr(msg.data)

def pose_callback(msg):
    global _mc_pending
    _mc_pending += chr(_S_POSE)
    _mc_pending += _pack_float(msg.x)
    _mc_pending += _pack_float(msg.y)
    if msg.theta > 1.5:
        _mc_pending += chr(180)
    elif msg.theta < -1.5:
        _mc_pending += chr(0)
    else:
        _mc_pending += chr(int(msg.theta * (180.0 / 3.14159)) + 90)

def cell_cost_callback(msg):
    global _mc_pending
    _mc_pending += chr(_S_CELL_COST)
    _mc_pending += chr(int(msg.x))
    _mc_pending += chr(int(msg.y))
    print msg.x
    print msg.y
    print msg.cost
    print "--------------"

def control_state_callback(msg):
    global _mc_pending
    _mc_pending += chr(_S_CONTROL_STATE)
    _mc_pending += chr(int(msg.data))

def network():
    rospy.init_node("network")
    rate = rospy.Rate(3)
    
    rospy.Subscriber("pose", Pose2D, pose_callback)
    rospy.Subscriber("/world_cost", CellCost, cell_cost_callback)
    rospy.Subscriber("state", Int8, state_callback)
    rospy.Subscriber("control_state", Int8, control_state_callback)
    
    rospy.Timer(rospy.Duration(3), _sys_temp)
    _init()
    while not rospy.is_shutdown():
        _tick()
        rate.sleep()

################################################################################
# PRIVATE FUNCTIONS
################################################################################

def _pack_float(num):
    major = int(math.floor(num))
    
    # we can pack all the info into a single byte by reducing the precision
    # of the number to a single decimal place and using the two nybbles to carry
    # the data
    # i.e. first nybble (0-15) + second nybble (0-9)
    if major < 16:
        minor = int(math.floor((num - major) * 10))
        return chr((major << 4) | minor)
    # we need two bytes to pack the number so we will use two decimal place
    # precision
    else:
        minor = int(math.floor((num - major) * 100))
        return chr(major) + chr(minor)

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
        _mc.send(chr(rospy.get_param("id")))
        
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
    while (len(_mc_to_process) > 0):
        # int value of the current byte being processed
        cbval = ord(_mc_to_process[0])
        # remaining length of the data string
        plen = len(_mc_to_process)
        # essentially a giant switch statement for all available data that
        # the mission control can send to the robot
        if cbval == _S_MC1:
            if plen >= 2:
                _mc1_pub.publish(ord(_mc_to_process[1]) - 100)
                print "mc1: " + str(ord(_mc_to_process[1]) - 100)
                _mc_to_process = _mc_to_process[2:]
            else: break
        elif cbval == _S_MC2:
            if plen >= 2:
                _mc2_pub.publish(ord(_mc_to_process[1]) - 100)
                print "mc2: " + str(ord(_mc_to_process[1]) - 100)
                _mc_to_process = _mc_to_process[2:]
            else: break
        elif cbval == _S_STARTING_PARAMS:
            if plen >= 2:
                packed = ord(_mc_to_process[1])
                
                zone = (0xF0 & packed) >> 4
                orientation = (0x0F & packed)
                
                rospy.set_param("/starting_zone", zone)
                rospy.set_param("/starting_orientation", orientation)
                print "zone = " + str(zone)
                print "orientation = " + str(orientation)
                
                _mc_to_process = _mc_to_process[2:]
            else: break
        elif cbval == _S_ROUND_START:
            _round_active_pub.publish(True)
            _mc_to_process = _mc_to_process[1:]
        elif cbval == _S_ROUND_STOP:
            _round_active_pub.publish(False)
            _mc_to_process = _mc_to_process[1:]
        elif cbval == _S_AUTONOMY_STOP:
            _autonomy_active_pub.publish(False)
            _mc_to_process = _mc_to_process[1:]
        elif cbval == _S_AUTONOMY_START:
            _autonomy_active_pub.publish(True)
            _mc_to_process = _mc_to_process[1:]
        elif cbval == _S_CONTROL_STATE:
            if plen >= 2:
              _control_state_pub.publish(ord(_mc_to_process[1]))
              _mc_to_process = _mc_to_process[2:]
            else: break
        # discard the current byte and continue processing the rest of the
        # string
        else:
            _mc_to_process = _mc_to_process[1:]

def _cleanup():
    rospy.loginfo("DISCONNECTING FROM MISSION CONTROL")
    _mc.close()

# state the system cpu and gpu temperature to mission control
def _sys_temp(event):
    global _mc_pending

    # this will only work on the raspberry pi, ;)
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as cpu_thermal:
            for line in cpu_thermal:
                cpu_temp = float(int(line.rstrip())) / 1000
                _mc_pending += chr(_S_CPU_TEMP) + _pack_float(cpu_temp)
    except:
        pass


################################################################################
# ENTRY POINT
################################################################################
        
if __name__ == "__main__":
    try:
        network()
    except rospy.ROSInterruptException:
        pass
        
    _cleanup()

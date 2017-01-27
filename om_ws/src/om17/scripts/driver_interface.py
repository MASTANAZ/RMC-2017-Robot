#!/usr/bin/env python

# DRIVER_INTERFACE
#
# CREATED BY HARRIS NEWSTEDER
#
# DESCRIPTION:
# 

################################################################################
# IMPORTS
################################################################################

import sys
import json
import serial
import serial.tools.list_ports

import rospy
from std_msgs.msg import String

################################################################################
# MODULE VARIABLES
################################################################################

_arduino = serial.Serial()
_arduino.baudrate = 9600
_arduino.bytesize = 8
_arduino.parity = 'N'
_arduino.stopbits = 1
_arduino.timeout = None
_arduino.xonxoff = False
_arduino.rtscts = False
_arduino.dsrdtr = False

################################################################################
# PUBLIC FUNCTIONS
################################################################################

def driver_interface():
    rospy.init_node("driver_interface")
    rate = rospy.Rate(3)
    _init()
    while not rospy.is_shutdown():
        _tick()
        rate.sleep()

################################################################################
# PRIVATE FUNCTIONS
################################################################################

def _init():
    global _arduino

    rospy.loginfo("INITIALIZING SERIAL COMMUNICATION")

    # search all available COM ports for an arduino
    for p in serial.tools.list_ports.comports():
        d = p.description
        if d[:-1] == "ttyACM":
            rospy.loginfo("FOUND ARDUINO ON PORT " + p.device)
            _arduino.port = p.device

    # no open ports found
    if _arduino.port == None:
        rospy.logfatal("FAILED TO CONNECT TO ARDUINO")
        return

    # attempt to open serial communications with arduino
    try:
        _arduino.open()
        rospy.loginfo("SUCCESSFULLY OPENED ARDUINO ON PORT " + _arduino.port)
    except:
        rospy.logfatal("FAILED TO OPEN ARDUINO ON PORT " + _arduino.port)
    
def _tick():
    while _arduino.in_waiting:
        print _arduino.readline()

def _cleanup():
    rospy.loginfo("CLOSING COMMUNICATIONS WITH ARDUINO ON PORT " + _arduino.port)
    _arduino.close()
    pass

################################################################################
# ENTRY POINT
################################################################################

if __name__ == "__main__":
    try:
        driver_interface()
    except rospy.ROSInterruptException:
        pass
        
    _cleanup()


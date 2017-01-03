################################################################################
# IMPORTS
################################################################################

import serial
import sys

################################################################################
# CONSTANTS
################################################################################

_ARDUINO_BAUDRATE = 9600
_ARDUINO_PORT = "COM3"

################################################################################
# MODULE VARIABLES
################################################################################

_arduino = serial.Serial()
_arduino.baudrate = _ARDUINO_BAUDRATE
_arduino.port = _ARDUINO_PORT

################################################################################
# PUBLIC FUNCTIONS
################################################################################

def init():
    print "> INITIALIZING DRIVERS"
    print "> ATTEMPTING TO OPEN SERIAL COMMUNICATION WITH ARDUINO"
    _arduino.open()
    if not _arduino.is_open:
        print "! ERROR: FAILED"
        sys.exit()
    print "> SUCCESS"
    
def tick():
    if _arduino.in_waiting > 0:
        print _arduino.readline().rstrip()
    
def cleanup():
    _arduino.close()
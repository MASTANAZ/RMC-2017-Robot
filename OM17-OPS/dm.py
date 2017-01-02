################################################################################
# IMPORTS
################################################################################

import network

################################################################################
# CONSTANTS
################################################################################

PROP_X           = network._S_P_X
PROP_Y           = network._S_P_Y
PROP_ORIENTATION = network._S_P_ORIENTATION

################################################################################
# MODULE VARIABLES
################################################################################

_selfDM = {
    PROP_X : 0,
    PROP_Y : 0,
    PROP_ORIENTATION : 0,
}

_otherDM = {
    PROP_X : 0,
    PROP_Y : 0,
    PROP_ORIENTATION : 0,
}

################################################################################
# PUBLIC FUNCTIONS
################################################################################

def init():
    print "> INITIALIZING DATA MODEL"

def self_set(key, value):
    global _selfDM
    _selfDM[key] = value 

def self_get(key):
    global _selfDM
    return _selfDM.get(key)
    pass

def other_set():
    pass
    
def other_get():
    pass
################################################################################
# imports
################################################################################

import network

################################################################################
# constants
################################################################################

PROP_LCV         = network._S_P_LCV
PROP_RCV         = network._S_P_RCV
PROP_X           = network._S_P_X
PROP_Y           = network._S_P_Y
PROP_ORIENTATION = network._S_P_ORIENTATION

################################################################################
# module variables
################################################################################

_propertyMap = {
    PROP_LCV : 1,
    PROP_RCV : 2,
    PROP_X : 3,
    PROP_Y : 4,
    PROP_ORIENTATION : 5,
}

################################################################################
# functions
################################################################################

def initialize():
    print "> INITIALIZING DATA MODEL"
    
def get_property(key):
    global _propertyMap
    
    ret = _propertyMap.get(key)
    
    if ret == None:
        print "> PROPERTY LOOKUP FAILED"
    
    return ret
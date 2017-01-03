################################################################################
# IMPORTS
################################################################################

import sys
import json

################################################################################
# CONSTANTS
################################################################################

SELF_ID = ""
SELF_IP = ""

OTHER_IP = ""

MC_IP = ""

################################################################################
# PUBLIC FUNCTIONS
################################################################################

def init():
    print "> INITIALIZING ROBOT INFORMATION"

    global SELF_ID, SELF_IP, OTHER_IP, MC_IP

    print "> ATTEMPTING TO READ ROBOT INFORMATION FROM \"rbinfo.json\""
    
    try:
        with open("rbinfo.json") as jsonFile:
            jsonData = json.load(jsonFile)
        
        SELF_ID  = jsonData["SELF_ID"]
        SELF_IP  = jsonData["SELF_IP"]
        OTHER_IP = jsonData["OTHER_IP"]
        MC_IP    = jsonData["MC_IP"]
        
        print "> SELF ID = " + SELF_ID
        print "> SELF IP = " + SELF_IP
        print "> OTHER IP = " + OTHER_IP
        print "> MISSION CONTROL IP = " + MC_IP
        print "> SUCCESS"
    except:
        print "! ERROR: FAILED"
        sys.exit()
    
    jsonFile.close()
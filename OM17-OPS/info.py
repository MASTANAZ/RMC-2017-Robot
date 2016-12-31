import sys

ROBOT_ID = ""
ROBOT_IP = ""
OTHER_IP = ""

def initialize():
    global ROBOT_ID, ROBOT_IP, OTHER_IP

    print "> ATTEMPTING TO READ ROBOT INFORMATION FROM \"robot.info\""
    
    infoFile = None
    
    try:
        infoFile = open("a.info")
        
        ROBOT_ID = infoFile.readline().rstrip()
        ROBOT_IP = infoFile.readline().rstrip()
        OTHER_IP = infoFile.readline().rstrip()
        
        print "> ROBOT ID = " + ROBOT_ID
        print "> ROBOT IP = " + ROBOT_IP
        print "> OTHER IP = " + OTHER_IP
    except:
        print "! ERROR: FAILED"
        sys.exit()
    
    infoFile.close()
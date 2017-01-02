import sys

SELF_ID = ""
SELF_IP = ""
OTHER_IP = ""

def init():
    global SELF_ID, SELF_IP, OTHER_IP

    print "> ATTEMPTING TO READ ROBOT INFORMATION FROM \"robot.info\""
    
    infoFile = None
    
    try:
        infoFile = open("robot.info")
        
        SELF_ID = infoFile.readline().rstrip()
        SELF_IP = infoFile.readline().rstrip()
        OTHER_IP = infoFile.readline().rstrip()
        
        print "> SELF ID = " + SELF_ID
        print "> SELF IP = " + SELF_IP
        print "> OTHER IP = " + OTHER_IP
    except:
        print "! ERROR: FAILED"
        sys.exit()
    
    infoFile.close()
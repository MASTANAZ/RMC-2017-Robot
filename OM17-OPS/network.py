################################################################################
# imports
################################################################################

import socket
import sys
import math
import select

################################################################################
# constants
################################################################################

_S_END             = 0xFF
_S_P_LCV           = 0x01
_S_P_RCV           = 0x02
_S_P_X             = 0x03
_S_P_Y             = 0x04
_S_P_ORIENTATION   = 0x05

# connection settings for communications with mission control
_MC_IP             = "localhost"
_MC_PORT           = 12000
_MC_BUFFER_SIZE    = 1024
_MC_RECV_TIMEOUT   = 0.05
_MC_CONNECTION_KEY = "@MC"

# connection settings for communications with the other robot
_RB_IP             = "localhost"
_RB_PORT           = 12001
_RB_BUFFER_SIZE    = 512
_RB_RECV_TIMEOUT   = 0.05
_RB_CONNECTION_KEY = "@RB"

################################################################################
# module variables
################################################################################

_mc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
_rb = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

_mc.settimeout(3)
_rb.settimeout(3)

# whether or not we have an active connection
_mcConnected = False

# whether or not we are hosting a robot server
_rbHosting = False
# whether or not we have an active client connection or are actively connected
# to a robot server
_rbConnected = False

# the unsigned byte arrays that will be sent over TCP during the next tick()
# NOTE: these arrays will be cleared after they are sent over the network
_mcPending = ""
_rbPending = ""

################################################################################
# public management functions
################################################################################

def initialize():
    print "> INITIALIZING NETWORK COMMUNICATIONS"
    _initialize_rb()
    _initialize_mc()
    
'''
def probe():
	global _SERVER_IP
	
	serverFound = False
	
	print "> SCANNING FOR SERVER ON LAN"
	for i in range(0, 256):
		ip = "192.168.1." + str(i)
		sys.stdout.write("\r> PROBING IP %s" % ip)
		sys.stdout.flush()
		server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		server.settimeout(0.1)
		try:
			server.connect((ip, _SERVER_PORT))
			print "\n> SERVER FOUND"
			_SERVER_IP = ip
			serverFound = True
			break
		except:
			server.close()
			continue
		
	if not serverFound: print "\n> NO SERVER FOUND ON LAN"
	
	return serverFound
'''

def tick():
    _tick_rb()
    _tick_mc()

def cleanup():
    _rb.close()
    _mc.close()

################################################################################
# private functions
################################################################################

def _initialize_rb():
    global _rbHosting

    print "> DETERMINING IF ROBOT SERVER ALREADY EXISTS"
    
    exists = False
    
    tmp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tmp.settimeout(1)
    
    try:
        tmp.connect((_RB_IP, _RB_PORT))
        print "> ROBOT SERVER FOUND AT " + _RB_IP
        exists = True
    except:
        print "> NO ROBOT SERVER FOUND"
        exists = False
        
    tmp.close()
        
    if not exists:
        print "> ATTEMPTING TO CREATE ROBOT SERVER AT localhost"
        try:
            _rb.bind(("localhost", _RB_PORT))
            _rb.listen(1)
            print "> SUCCESS"
            _rbHosting = True
        except:
            print "> FAILED"
    else:
        print "> ATTEMPTING TO CONNECT TO ROBOT SERVER AT " + _RB_IP
        try:
            _rb.connect((_RB_IP, _RB_PORT))
            _rb.send(_RB_CONNECTION_KEY)
            _rb.setblocking(0)
            print "> SUCCESS"
            _rbConnected = True
        except:
            print "> FAILED"

def _initialize_mc():
    global _mcConnected
    
    print "> ATTEMPTING TO CONNECT TO MISSION CONTROL AT " + _MC_IP
    try:
        _mc.connect((_MC_IP, _MC_PORT))
        _mc.send(_MC_CONNECTION_KEY)
        _mc.setblocking(0)
        print "> SUCCESS"
        _mcConnected = True
    except:
        print "> FAILED TO CONNECT TO MISSION CONTROL"
        
    return _mcConnected

def _tick_rb():
    global _rbConnected, _rbHosting
    
    # acting as a server for the other robot
    if _rbHosting:
        if not _rbConnected:
            try:
                conn, addr = _rb.accept()
                _rbConnected = _process_client(conn)
            except:
                _rbConnected = False
            return
    # acting as a client to the other robot
    else:
        # make sure we have an active connection with our server
        if not _rbConnected: return
        
        # receive data from our server with a timeout
        ready = select.select([_rb], [], [], _RB_RECV_TIMEOUT)
        
        # read any available statements from the server and parse
        if ready[0]:
            data = _server.recv(_SERVER_BUFFER_SIZE)
            if not data:
                _connected = False
            else:
                _parse_statements(data)
                
        # send pending statements to the server

def _tick_mc():
    global _mcConnected
    
def _process_client(conn):
    print "> PROCESSING NEW CLIENT CONNECTION"
    key = conn.recv(16)
    if key == _RB_CONNECTION_KEY:
        print "> CLIENT KEY ACCEPTED"
        print "> SUCCESSFULLY ADDED NEW CLIENT"
        return True
    else:
        print "! ERROR: UNRECOGNIZED CLIENT, TERMINATING CONNECTION"
        conn.close()
        return False
    
################################################################################
# public access functions
################################################################################

'''
def state_x(x):
	global _pending
	
	major = int(x)
	minor = int((x - major) * 100)
	
	_pending += chr(_S_P_X)
	_pending += chr(minor)
	_pending += chr(major)
	_pending += chr(_S_END)

def state_y(y):
	global _pending
	
	major = int(y)
	minor = int((y - major) * 100)
	
	_pending += chr(_S_P_Y)
	_pending += chr(minor)
	_pending += chr(major)
	_pending += chr(_S_END)

    
def state_orientation(orientation):
    global _pending
    
    major = int(orientation)
    minor = int((orientation - major) * 100)
    
    _pending += chr(_S_P_ORIENTATION)
    _pending += chr(minor)
    _pending += chr(major)
    _pending += chr(_S_END)
'''
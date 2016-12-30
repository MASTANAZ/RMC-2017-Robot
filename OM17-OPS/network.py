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

_S_END           = 0xFF

_S_P_LCV         = 0x01
_S_P_RCV         = 0x02
_S_P_X           = 0x03
_S_P_Y           = 0x04
_S_P_ORIENTATION = 0x05

_SERVER_IP             = "localhost"
_SERVER_PORT           = 12000
_SERVER_BUFFER_SIZE    = 1024
_SERVER_RECV_TIMEOUT   = 0.05
_SERVER_CONNECTION_KEY = "MINERS_WIN"

################################################################################
# module variables
################################################################################

_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
_server.settimeout(3)

_connected = False
_pending = ""

################################################################################
# public management functions
################################################################################

def initialize():
	_server.settimeout(3)

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

def connect():
	global _connected
	
	print "> ATTEMPTING TO CONNECT TO SERVER AT " + _SERVER_IP
	try:
		_server.connect((_SERVER_IP, _SERVER_PORT))
		_server.send(_SERVER_CONNECTION_KEY)
		_server.setblocking(0)
		print "> SUCCESS"
		_connected = True
	except:
		print "> FAILED TO CONNECT TO SERVER"
		_connected = False
		
	return _connected

def tick():
	global _connected

	# make sure we have an active connection with our server
	if not _connected: return
	
	# receive data from our server with a timeout
	ready = select.select([_server], [], [], _SERVER_RECV_TIMEOUT)
	
	# read any available statements from the server and parse
	if ready[0]:
		data = _server.recv(_SERVER_BUFFER_SIZE)
		if not data:
			_connected = False
		else:
			_parse_statements(data)
			
	# send pending statements to the server
	_server.send(_pending)
	_clear_pending()
	
def cleanup():
	_server.close()
	
################################################################################
# private functions
################################################################################
	
def _parse_statements(statementString):
	statementString = ""

def _clear_pending():
	global _pending
	_pending = ""
	
################################################################################
# public access functions
################################################################################

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
    
    major = int(y)
	minor = int((y - major) * 100)
	
	_pending += chr(_S_P_ORIENTATION)
	_pending += chr(minor)
	_pending += chr(major)
	_pending += chr(_S_END)
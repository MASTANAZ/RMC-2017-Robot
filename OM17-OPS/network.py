import socket
import sys

# constants

CB_END = 0xFF
CB_LCV = 0x01
CB_RCV = 0x02
CB_PSX = 0x03
CB_PSY = 0x04
CB_ORN = 0x05

SERVER_IP = "192.168.1.166"
SERVER_PORT = 12000
SERVER_BUFFER_SIZE = 1024
SERVER_CONNECTION_KEY = "MINERS_WIN"

_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# functions

def probe():
	print "> SCANNING FOR SERVER ON LAN"
	for i in range(0, 256):
		ip = "192.168.1." + str(i)
		sys.stdout.write("\r> PROBING IP %s" % ip)
		sys.stdout.flush()
		server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		server.settimeout(0.1)
		try:
			server.connect((ip, SERVER_PORT))
			print "\n> SERVER FOUND"
			SERVER_IP = ip
			break
		except:
			server.close()
			continue

def connect():
	print "> ATTEMPTING TO CONNECT TO SERVER AT " + SERVER_IP
	try:
		_server.connect((SERVER_IP, SERVER_PORT))
		_server.send(SERVER_CONNECTION_KEY)
		_server.setblocking(0)
		print "> SUCCESS"
	except:
		print "> FAILED TO CONNECT TO SERVER"

		
def tick():
	ready = select.select([_server], [], [], 0.1)
	if ready[0]:
		data = _server.recv(SERVER_BUFFER_SIZE)
		if not data:
			print "test"

def cleanup():
	_server.close()

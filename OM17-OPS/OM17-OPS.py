import time
import sys
import socket
import select
import random

# constants

CB_END = 0x00
CB_LCV = 0x01
CB_RCV = 0x02
CB_PSX = 0x03
CB_PSY = 0x04

SERVER_IP = 'localhost'
SERVER_PORT = 12000
SERVER_BUFFER_SIZE = 1024
SERVER_CONNECTION_KEY = "MINERS_WIN"

#

print "--------------------------------------------------------------------------------"
print "OM17-OPS"
print "--------------------------------------------------------------------------------"

#

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.connect((SERVER_IP, SERVER_PORT))
server.setblocking(0)
server.send(SERVER_CONNECTION_KEY)

print "> CONNECTED TO OM17-CLIENT PROGRAM"

#

time.sleep(1)

while True:
        time.sleep(2)
        tosend = bytearray()
        tosend.append(CB_LCV)
        server.send(tosend)
	ready = select.select([server], [], [], 0.5)
	if ready[0]:
		data = server.recv(SERVER_BUFFER_SIZE)
		if not data: break

# cleanup

server.close()

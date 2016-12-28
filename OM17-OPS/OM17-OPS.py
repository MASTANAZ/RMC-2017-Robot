import time
import sys
import socket
import select
import random

# functions


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
	time.sleep(0.5)
	ready = select.select([server], [], [], 0.5)
	if ready[0]:
		data = server.recv(SERVER_BUFFER_SIZE)
		if not data: break
	tosend = bytearray([CB_PSX, random.randint(0, 38), random.randint(0, 7), CB_END])
	server.send(tosend)
	tosend = bytearray([CB_PSY, random.randint(0, 88), random.randint(0, 3), CB_END])
	server.send(tosend)

# cleanup

server.close()

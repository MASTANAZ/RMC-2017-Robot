import time
import sys
import socket
import select
import random
import serial
import math

# functions


# constants

CB_END = 127
CB_LCV = 0x01
CB_RCV = 0x02
CB_PSX = 0x03
CB_PSY = 0x04
CB_ORN = 0x05

SERVER_IP = 'localhost'
SERVER_PORT = 12000
SERVER_BUFFER_SIZE = 1024
SERVER_CONNECTION_KEY = "MINERS_WIN"

#

print "--------------------------------------------------------------------------------"
print "OM17-OPS"
print "--------------------------------------------------------------------------------"

#

arduino = serial.Serial('COM3', 9600)

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.connect((SERVER_IP, SERVER_PORT))
server.setblocking(0)
server.send(SERVER_CONNECTION_KEY)

print "> CONNECTED TO OM17-CLIENT PROGRAM"

#

time.sleep(1)

while True:
	ready = select.select([server], [], [], 0.1)
	if ready[0]:
		data = server.recv(SERVER_BUFFER_SIZE)
		if not data: break
	if (arduino.inWaiting() > 0):
		angleString = arduino.readline()
		distString = arduino.readline()
		try:
			angle = float(angleString)
			dist = float(distString)
			angle = (angle * math.pi) / 180.0
			x = math.cos(angle) * dist
			y = math.sin(angle) * dist
			y = y + 1.89

			major = int(math.floor(x))
			minor = int((x * 100) - (major * 100))
			tosend = bytearray([CB_PSX, minor, major, CB_END])
			server.send(tosend)
			
			print("%s %s" % (major, minor))
			
			major = int(math.floor(y))
			minor = int((y * 100) - (major * 100))
			tosend = bytearray([CB_PSY, minor, major, CB_END])
			server.send(tosend)
			
			print("%s %s" % (major, minor))
			
			major = int(math.floor(angle))
			minor = int((angle * 100) - (major * 100))
			tosend = bytearray([CB_ORN, minor, major, CB_END])
			server.send(tosend)
			print("%s %s" % (major, minor))
			print ""
		except:
			print "bad line"

# cleanup

arduino.close()
server.close()

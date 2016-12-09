import time
import sys
import socket
import select
import random

print "\n--------------------------------------------------------------------------------"
print "OSPREY MINERS 2017 OPERATIONS PROGRAM"
print "--------------------------------------------------------------------------------\n"

# constants

SERVER_IP = '192.168.1.178'
SERVER_PORT = 12000

# TCP server
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# ensure that the address can be reused incase we run the program more than once
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((SERVER_IP, SERVER_PORT))
server.listen(1)

print "ROBOT SERVER CREATION SUCCESSFUL"

time.sleep(2)

# server loop

client, address = server.accept()

client.setblocking(0)

print "CONNECTED TO OM17-CLIENT PROGRAM"

while True:
	time.sleep(1)
	x = random.uniform(0.0, 7.38)
	client.send(str(x) + '\n')
	ready = select.select([client], [], [], 0.5)
	if ready[0]:
		data = client.recv(4096)
		if not data: break

# cleanup

client.close()
server.close()

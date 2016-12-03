import time
import sys
import socket

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

print "CONNECTED TO OM17-CLIENT PROGRAM"

while True:
	data = client.recv(16)
	if not data: break
	print(data);

# cleanup

client.close()
server.close()

import network
import dm
import sys
import time
import os
import rbinfo

os.system("cls")

print "--------------------------------------------------------------------------------"
print "OM17-OPS"
print "--------------------------------------------------------------------------------"

rbinfo.init()
network.init()

while True:
    network.tick()

network.cleanup()
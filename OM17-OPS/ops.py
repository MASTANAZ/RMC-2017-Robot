import network
import dm
import sys
import time

print "--------------------------------------------------------------------------------"
print "OM17-OPS"
print "--------------------------------------------------------------------------------"

network.initialize()

while True:
    network.tick()

network.cleanup()
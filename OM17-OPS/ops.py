import network
import dm
import sys
import time

print "--------------------------------------------------------------------------------"
print "OM17-OPS"
print "--------------------------------------------------------------------------------"

network.initialize()
if network.connect() == False: sys.exit()

time.sleep(5)

network.state_x(2.55)
network.tick()

time.sleep(5)

network.state_x(6.55)
network.tick()

network.cleanup()
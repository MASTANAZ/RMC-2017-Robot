import network
import dm
import sys
import time

print "--------------------------------------------------------------------------------"
print "OM17-OPS"
print "--------------------------------------------------------------------------------"

network.initialize()

print dm.get_property(dm.PROP_X)

if network.connect() == False: sys.exit()

network.cleanup()
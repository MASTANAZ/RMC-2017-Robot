import network
import dm
import rbinfo

import sys
import time
import os

os.system("cls")

print "--------------------------------------------------------------------------------"
print "OM17-OPS"
print "--------------------------------------------------------------------------------"

dm.init()

dm.self_set(dm.PROP_X, 7.77)

print dm.self_get(dm.PROP_X)

rbinfo.init()
#network.init()

#while True:
#    network.tick()

#network.cleanup()
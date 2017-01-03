import network
import dm
import rbinfo
import driver

import sys
import time
import os

os.system("cls")

print "--------------------------------------------------------------------------------"
print "OM17-OPS"
print "--------------------------------------------------------------------------------"

dm.init()
rbinfo.init()
driver.init()

#network.init()

while True:
    driver.tick()
    #network.tick()

#network.cleanup()
driver.cleanup()
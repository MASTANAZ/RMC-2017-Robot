import network
import dm
import rbinfo
import driver
import common

import sys
import time
import os

# determine which platform we're on and clear the terminal
if sys.platform == "linux" or sys.platform == "linux2":
    os.system("clear")
elif sys.platform == "darwin":
    os.system("clear")
elif sys.platform == "win32":
    os.system("cls")

print "--------------------------------------------------------------------------------"
print "OM17-OPS"
print "--------------------------------------------------------------------------------"

dm.init()
rbinfo.init()
#driver.init()

#network.init()

try:
    while True:
        pass
except KeyboardInterrupt:
    pass

#network.cleanup()
#driver.cleanup()
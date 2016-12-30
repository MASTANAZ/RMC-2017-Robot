import network
import sys

print "--------------------------------------------------------------------------------"
print "OM17-OPS"
print "--------------------------------------------------------------------------------"

network.initialize()
network.connect()
network.state_bot_x(2.55)

network.tick()

network.cleanup()
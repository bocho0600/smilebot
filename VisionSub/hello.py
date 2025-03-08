from __future__ import print_function
import sys
import os
import time
from RP2040 import I2C
from mobi import MobilityModule

# Initialize the I2C communication
#I2C.init(bus_number=1, addr=0x08)

# Test movement function
MobilityModule.Move(0, 25)

time.sleep(4)
MobilityModule.Move(0, 0)

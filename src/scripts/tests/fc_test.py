import os
import sys

# Support direct execution from any working directory.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from h2rMultiWii import MultiWii
import time

board = MultiWii("/dev/ttyUSB0")

def g():
    return board.getData(MultiWii.ATTITUDE)

board.arm()
board.disarm()
print(g())

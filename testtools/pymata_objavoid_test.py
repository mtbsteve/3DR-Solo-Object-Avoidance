#!/usr/bin/python
"""
 Copyright (c) 2017 Stephan Schindewolf All rights reserved.
 This file demonstrates how to use some of the basic PyMata operations.
"""

import time
import sys
import signal

from PyMata.pymata import PyMata

# define Digital pin 12-6
COLL_RIGHT =4
COLL_CENTER = 3
COLL_LEFT = 2
TOGGLE_SWITCH = 5
SCAN_ENABLE = 6

# Create a PyMata instance
board = PyMata("/dev/ttyACM0", verbose=True)


def signal_handler(sig, frame):
    print('You pressed Ctrl+C')
    print("Stop Scanning")
    board.digital_write(SCAN_ENABLE, 0)
    time.sleep(2)
    if board is not None:
        board.reset()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

# Set digital pins
board.set_pin_mode(COLL_RIGHT, board.INPUT, board.DIGITAL)
board.set_pin_mode(COLL_CENTER, board.INPUT, board.DIGITAL)
board.set_pin_mode(COLL_LEFT, board.INPUT, board.DIGITAL)
board.set_pin_mode(TOGGLE_SWITCH, board.OUTPUT, board.DIGITAL)
board.set_pin_mode(SCAN_ENABLE, board.OUTPUT, board.DIGITAL)

time.sleep(2)
print("Start Scanning")
board.digital_write(SCAN_ENABLE, 1)
time.sleep(5)
print("toggle on")
board.digital_write(TOGGLE_SWITCH, 1)
time.sleep(5)
print("toggle off")
board.digital_write(TOGGLE_SWITCH, 0)
time.sleep(5)

while 1:
    if (board.digital_read(COLL_RIGHT) == 1):
		print "Collision on the right"
    elif (board.digital_read(COLL_CENTER) == 1):
        print "Collision on center"
    elif (board.digital_read(COLL_LEFT) == 1):
        print "Collision on the left"
    else:
        print ("no obstacle ahead")
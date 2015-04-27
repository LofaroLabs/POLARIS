#!/usr/bin/env python

#IMPORTS
import ach
import sys
import time
#from ctypes import *
from ctypes import Structure,c_uint16,c_double,c_ubyte,c_uint32,c_int16,c_char
import socket
import cgposition_include as pi
import localization_include as li
import localization_include2 as li2
import slam_include as si

import os

import math
import time

#CONSTANTS
TAGSIZE = 0.1778

#FUNCTIONS


s = ach.Channel(si.SLAM_CHANNEL_NAME)

slam = si.SLAM_TYPE()
s.put(slam)


while True:

	#get SLAM odometry data
	[statuss, framesizes] = s.get(slam, wait=False, last=True)
	xSLAM = slam.xSLAM
	ySLAM = slam.ySLAM
	yawSLAM = slam.yawSLAM * (180.0/math.pi)

	print "xSLAM: ", xSLAM
	print "ySLAM: ", ySLAM
	print "yawSLAM: ", yawSLAM


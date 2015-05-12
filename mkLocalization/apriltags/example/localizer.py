#!/usr/bin/env python

import ach 
import sys 
import time 
from ctypes import * 
import socket 
import os 
import cgposition_include as pi
import localizer_include as mi 

p = ach.Channel(pi.POSITION_CHANNEL_NAME) 
# feed-forward will now be refered to as "positionTransfer"
positionTransfer = pi.CHAN_TYPE() 
p.put(positionTransfer) 
p.flush()


# make a channel 
makeAch = os.popen('ach mk localizer') 
print makeAch
print "WAIT 5 SECONDS, MAKING A CHANNEL..." 
time.sleep(5) 
c = ach.Channel(mi.LOCALIZER_CHANNEL_NAME) 
localizerTransfer = mi.CHAN_TYPE()

positionArrayX = []
positionArrayY = []


validBit = 0

while True:

	# get most recent position from localization module 
	[status, framesize] = p.get(positionTransfer, wait=False, last=True)


	if(positionTransfer.valid == 0):
		print "ERR: NO TAG, view a tag before beginning test"
		
		localizerTransfer.valid = validBit
		localizerTransfer.x = 0.0
		localizerTransfer.y = 0.0
		localizerTransfer.z = 0
		localizerTransfer.THETA = 0.0
		c.put(localizerTransfer)
		

	else:
		validBit = 1
 
		print "X IS: ",positionTransfer.x
		print "Y IS: ",positionTransfer.y
		print "Z IS: ",positionTransfer.z
		print "THETA IS: ",positionTransfer.THETA

		# get up to 5 localization data points
		# if you already have 5, get rid of the oldest one
		# to make room for the newest one
		if(len(positionArrayX) < 5):
			positionArrayX.append(positionTransfer.x)
			positionArrayY.append(positionTransfer.y)
		else:
			positionArrayX = positionArrayX[1:]
			positionArrayY = positionArrayY[1:]
			positionArrayX.append(positionTransfer.x)
			positionArrayY.append(positionTransfer.y)
		
		# get rollingAvg of x and y
		rollAvgX = 0.0
		rollAvgY = 0.0
		sumX = 0.0
		sumY = 0.0
		div = len(positionArrayX)
		for i in range(len(positionArrayX)):
			sumX = sumX + positionArrayX[i]
			sumY = sumY + positionArrayY[i]
		rollAvgX = sumX/div
		rollAvgY = sumY/div

		# transfer the final localization data
		localizerTransfer.valid = validBit
		localizerTransfer.x = rollAvgX
		localizerTransfer.y = rollAvgY
		localizerTransfer.z = positionTransfer.z
		localizerTransfer.THETA = positionTransfer.THETA
		c.put(localizerTransfer)	
 
	print "\n"

print "GOODBYE <3"
exit()

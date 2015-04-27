#!/usr/bin/env python

#IMPORTS
import ach
import sys
import time
#from ctypes import *
from ctypes import Structure,c_uint16,c_double,c_ubyte,c_uint32,c_int16,c_char
import socket
import recognition_include as ri
import cgposition_include as pi
import localization_include as li
import localization_include2 as li2

import os

import math
import time

#CONSTANTS
TAGSIZE = 0.1778

#FUNCTIONS


r = ach.Channel(ri.RECOGNITION_CHANNEL_NAME)

recognition = ri.RECOGNITION_TYPE()
r.put(recognition)

#read in LUT
LUT = li.readInLUT()

# make a channel 
makeAch = os.popen('ach mk position') 
print makeAch
print "WAIT 5 SECONDS, MAKING A CHANNEL..." 
time.sleep(5) 
c = ach.Channel(pi.POSITION_CHANNEL_NAME) 
positionTransfer = pi.CHAN_TYPE()

validBit = 0

# tags that have recently been spotted
IDsaved = [-1,-1,-1,-1,-1,-1,-1,-1]

# RPY data associated with spotted tags
roll0 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
roll1 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
roll2 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
roll3 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
roll4 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
roll5 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
roll6 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
roll7 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
pitch0 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
pitch1 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
pitch2 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
pitch3 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
pitch4 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
pitch5 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
pitch6 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
pitch7 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
yaw0 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
yaw1 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
yaw2 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
yaw3 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
yaw4 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
yaw5 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
yaw6 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
yaw7 = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
RPYarray = [[roll0,pitch0,yaw0],[roll1,pitch1,yaw1],[roll2,pitch2,yaw2],[roll3,pitch3,yaw3],[roll4,pitch4,yaw4],[roll5,pitch5,yaw5],[roll6,pitch6,yaw6],[roll7,pitch7,yaw7]]

# counter for how long a tag can go undetected
ditch = [0,0,0,0,0,0,0,0]



while True:

	#get key
	[statuss, framesizes] = r.get(recognition, wait=False, last=True)
	theKey = recognition.key

	if(theKey == 0):
		#make IPC data easier to deal with:
		ID = [recognition.detectionID0, recognition.detectionID1, recognition.detectionID2, recognition.detectionID3, recognition.detectionID4,recognition.detectionID5,recognition.detectionID6,recognition.detectionID7]
		XCoord = [recognition.detectionX0, recognition.detectionX1, recognition.detectionX2, recognition.detectionX3, recognition.detectionX4,recognition.detectionX5,recognition.detectionX6,recognition.detectionX7]
		YCoord = [recognition.detectionY0, recognition.detectionY1, recognition.detectionY2, recognition.detectionY3, recognition.detectionY4,recognition.detectionY5,recognition.detectionY6,recognition.detectionY7]
		ZCoord = [recognition.detectionZ0/2.0, recognition.detectionZ1/2.0, recognition.detectionZ2/2.0, recognition.detectionZ3/2.0, recognition.detectionZ4/2.0,recognition.detectionZ5/2.0,recognition.detectionZ6/2.0,recognition.detectionZ7/2.0]
		YAW = [recognition.detectionTHETA0, recognition.detectionTHETA1, recognition.detectionTHETA2, recognition.detectionTHETA3, recognition.detectionTHETA4, recognition.detectionTHETA5, recognition.detectionTHETA6, recognition.detectionTHETA7] 
		PITCH = [recognition.detectionPITCH0, recognition.detectionPITCH1, recognition.detectionPITCH2, recognition.detectionPITCH3, recognition.detectionPITCH4, recognition.detectionPITCH5, recognition.detectionPITCH6, recognition.detectionPITCH7]
		ROLL = [recognition.detectionROLL0, recognition.detectionROLL1, recognition.detectionROLL2, recognition.detectionROLL3, recognition.detectionROLL4, recognition.detectionROLL5
, recognition.detectionROLL6, recognition.detectionROLL7]

#		print "RECOGNIZED IDS ZZZ: ", ID

		#TESTED: CORRECT
		#begin looking for tags:
		#recognizedTags array is a variable length array
		recognizedTags = li2.getRecognizedTags(ID)

		#GOOD
		#check if any recently spotted tags have not been seen in awhile
		ditch = li2.checkForGarbage(recognizedTags,ditch,ID,IDsaved)

		#GARBAGE DAY D:<
		newData = li2.garbageDay(ditch,IDsaved,RPYarray)
		IDsaved = newData[0]
		RPYarray = newData[1]
		ditch = newData[2]
		#print "ID: ", IDsaved


		#shift in new RPY data for all recognized tags
		newRPYData = li2.placeNewData(recognizedTags,RPYarray,ROLL,PITCH,YAW,ditch,ID,IDsaved)
		RPYarray = newRPYData[0]
		ditch = newRPYData[1]
		IDsaved = newRPYData[2]



		#calculate rolling averages for RPY of all tags
		calculatedRollingAvg = li2.calculateRollingAvg(IDsaved,RPYarray)

#------CONTINUE? XXX XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX


		#index into LUT to get glyph globals for each tag
		xgg = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
		ygg = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
		THETAgg = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
		THETAgl = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
		zFINAL = 0
		#in addition, get glyph locals
		xgl = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
		ygl = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
		zgl = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
		for i in range(8):
			for j in range(len(recognizedTags)):
				if(IDsaved[i] == ID[recognizedTags[j]]):
					index = IDsaved[i]

					#print "recognizedTags: ",recognizedTags
					#print "index: ",index
					xgg[i] = LUT[index][0]
					ygg[i] = LUT[index][1]
					THETAgg[i] = LUT[index][2]
					zFINAL = LUT[index][3]
					

					xgl[i] = XCoord[recognizedTags[j]]
					ygl[i] = YCoord[recognizedTags[j]]
					zgl[i] = ZCoord[recognizedTags[j]]
					THETAgl[i] = YAW[recognizedTags[j]]

					break
				
# --------

		#get camera global from each glyph
		GL = [["NULL","NULL","NULL"],["NULL","NULL","NULL"],["NULL","NULL","NULL"],["NULL","NULL","NULL"],["NULL","NULL","NULL"],["NULL","NULL","NULL"],["NULL","NULL","NULL"],["NULL","NULL","NULL"]]

		CG = [["NULL","NULL","NULL"],["NULL","NULL","NULL"],["NULL","NULL","NULL"],["NULL","NULL","NULL"],["NULL","NULL","NULL"],["NULL","NULL","NULL"],["NULL","NULL","NULL"],["NULL","NULL","NULL"]]

		for i in range(8):
			if(str(xgl[i]) != "NULL"):
				GL[i] = li.getLocalTransformed([xgl[i],ygl[i],zgl[i]],calculatedRollingAvg[i][1],calculatedRollingAvg[i][0],calculatedRollingAvg[i][2])

				# Get CG = Camera Global
				CG[i] = li.getGlobalCameraTransformed([GL[i][0], GL[i][1], GL[i][2]], [xgg[i],ygg[i]])

		#get weighted average for camera globals
		distance = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
		distance2D = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
		invDistanceSum = 0.0
		for i in range(8):
			if(str(xgl[i]) != "NULL"):
				distance2D[i] = math.sqrt((xgl[i]**2) + (ygl[i]**2))
				distance[i] = math.sqrt((xgl[i]**2) + (ygl[i]**2) + (zgl[i]**2))
				invDistanceSum = invDistanceSum + (1.0/distance[i])

		CGSumX = 0.0
		CGSumY = 0.0
		for i in range(8):
			if(str(xgl[i]) != "NULL"):
				CGSumX = CGSumX + ((1.0/distance[i])*(CG[i][0]))
				CGSumY = CGSumY + ((1.0/distance[i])*(CG[i][1]))
		CGWeightedAvgX = 0.0
		CGWeightedAvgY = 0.0
		if(invDistanceSum != 0.0):
			CGWeightedAvgX = CGSumX/invDistanceSum
			CGWeightedAvgY = CGSumY/invDistanceSum

		#calculate THETA
		minDistance = 0.0
		minDistanceIndex = 0
		for i in range(8):
			if(distance2D[i] != "NULL"):
				minDistance = distance2D[i]
				minDistanceIndex = i
				break
		for i in range(8):
			if(distance2D[i] < minDistance):
				minDistance = distance2D[i]
				minDistanceIndex = i
		THETAcg = (THETAgl[minDistanceIndex] + THETAgg[minDistanceIndex])



		if(len(recognizedTags) != 0):
			validBit = 1
			THETAcg = THETAcg * (180.0/math.pi)
			if(THETAcg < 0.0):
				THETAcg = THETAcg + 360.0
			print "Camera Global X: ",CGWeightedAvgX
			print "Camera Global Y: ",CGWeightedAvgY
			print "Camera Global Z: ",zFINAL
			print "Camera Global YAW: ",THETAcg
			print "\n"

			positionTransfer.valid = validBit
			positionTransfer.x = CGWeightedAvgX
			positionTransfer.y = CGWeightedAvgY
			positionTransfer.z = zFINAL
			positionTransfer.THETA = THETAcg
			c.put(positionTransfer)
			#print "Count: ",recognition.count
		else:
			if(validBit == 0):
				positionTransfer.valid = validBit
				c.put(positionTransfer)
			print "ERR: NO TAG"
			

	#user has chosen to quit
	else:
		print "PLEASE WAIT, CLOSING CHANNEL..."
		time.sleep(5)
		removeAch = os.popen('ach rm position')
		exit()




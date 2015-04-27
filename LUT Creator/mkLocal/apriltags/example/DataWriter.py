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
import slam_include as si

import os

import math
import time

#CONSTANTS
TAGSIZE = 0.1778

#FUNCTIONS


r = ach.Channel(ri.RECOGNITION_CHANNEL_NAME)

recognition = ri.RECOGNITION_TYPE()
r.put(recognition)

s = ach.Channel(si.SLAM_CHANNEL_NAME)

slam = si.SLAM_TYPE()
s.put(slam)

#read in collected array
collectedArray = li.readInCollectedArray()

#read in localTagPose
localTagPose = li.readInLocalTagPose()

#read in globalCameraPosition
globalCameraPosition = li.readInGlobalCameraPosition()

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

	#get SLAM odometry data
	[statuss, framesizes] = s.get(slam, wait=False, last=True)
	xSLAM = slam.xSLAM
	ySLAM = slam.ySLAM
	yawSLAM = slam.yawSLAM * (180.0/math.pi)

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


		#TESTED: CORRECT
		#begin looking for tags:
		#recognizedTags array is a variable length array
		recognizedTags = li2.getRecognizedTags(ID)

		#if a tag has already been collected, remove its data
		i = 0
		while(i < (len(recognizedTags))):
			if(collectedArray[ID[recognizedTags[i]]] != 0):
				#data already collected for this ID, remove it!
				recognizedTags.remove(recognizedTags[i])
			else:
				i = i+1

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

		#writing data to the files under certain conditions
		for i in range(len(recognizedTags)):
			for j in range(len(IDsaved)):

				#find the RPY values corresponding to the reccognized tag being considered
				if(ID[recognizedTags[i]] == IDsaved[j]):

					#check that 10 angles have been received so that rolling avg is over correct window size
					countAngles = 0
					for k in range(len(RPYarray[j][0])):
						if(RPYarray[j][0][k] != "NULL"):
							countAngles = countAngles + 1
					if(countAngles == 10):

						#only transmit data if within certain x and y distance
						if((abs(XCoord[recognizedTags[i]]) < 0.5) and (abs(YCoord[recognizedTags[i]]) < 0.5)):
							#write positions XCoord[recognizedTags[i],YCoord[recognizedTags[i],
								#ZCoord[recognizedTags[i]
							#write angles calculatedRollingAvg[j][0],[j][1],[j][2]
							localTagPose[ID[recognizedTags[i]]] = [XCoord[recognizedTags[i]],YCoord[recognizedTags[i]],ZCoord[recognizedTags[i]],calculatedRollingAvg[j][0],calculatedRollingAvg[j][1],calculatedRollingAvg[j][2]]

							#write x,y,YAW from SLAM
							globalCameraPosition[ID[recognizedTags[i]]] = [xSLAM,ySLAM,yawSLAM]
							
							#set collectedArray[IDsaved[j]] bit
							collectedArray[ID[recognizedTags[i]]] = 1

							#remove RPYarray[j][0],[1],[2] and replace with NULL lists
							RPYarray[j][0] = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
							RPYarray[j][1] = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
							RPYarray[j][2] = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]

							#set ditch[j] to zero
							ditch[j] = 0

							#set IDsaved[j] to -1
							IDsaved[j] = -1

							#say something as verification that tag has been written
							print "TAG "+str(ID[recognizedTags[i]])+" WRITTEN TO ARRAY."


	#user has chosen to quit
	else:

		# write arrays to respective files

		#write collected array
		f = open("collectedArray.txt", "w")
		for i in range(587):
			f.write(str(collectedArray[i])+"\n")	
		f.close()

		#write local 6D tag pose
		f = open("localTagPose.txt", "w")
		for i in range(587):
			#x,y,z,R,P,Y
			f.write(str(localTagPose[i][0])+","+str(localTagPose[i][1])+","+str(localTagPose[i][2])+","+str(localTagPose[i][3])+","+str(localTagPose[i][4])+","+str(localTagPose[i][5])+"\n")
		f.close()

		#write global camera position
		f = open("globalCameraPosition.txt", "w")
		for i in range(587):
			#x,y,YAW
			f.write(str(globalCameraPosition[i][0])+","+str(globalCameraPosition[i][1])+","+str(globalCameraPosition[i][2])+"\n")
		f.close()

		exit()




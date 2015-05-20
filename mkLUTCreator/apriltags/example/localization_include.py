import math
import time
import numpy as np



def getLocalTransformed(glyphLocal,PITCH,ROLL,YAW):
	xgl = glyphLocal[0]
	ygl = glyphLocal[1]
	zgl = glyphLocal[2]
	wgl = 1.0

	xl = np.array([[xgl],[ygl],[zgl],[wgl]])

	Tpitch = np.array([[math.cos(PITCH),0.0,math.sin(PITCH),0.0],[0.0,1.0,0.0,0.0],[-math.sin(PITCH),0.0,math.cos(PITCH),0.0],[0.0,0.0,0.0,1.0]])

	Troll = np.array([[1.0,0.0,0.0,0.0],[0,math.cos(ROLL),-math.sin(ROLL),0.0],[0.0,math.sin(ROLL),math.cos(ROLL),0.0],[0.0,0.0,0.0,1.0]])

	Tyaw = np.array([[math.cos(YAW),-math.sin(YAW),0.0,0.0],[math.sin(YAW),math.cos(YAW),0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0]])


	xlyaw = np.dot(Tyaw,xl)
	xlpitchyaw = np.dot(Tpitch,xlyaw)
	xlnew = np.dot(Troll,xlpitchyaw)
	
	xnew = xlnew.item(0)
	ynew = xlnew.item(1)
	znew = xlnew.item(2)
	wnew = xlnew.item(3)

	return [(xnew),(ynew),(znew)]

def getGlobalCameraTransformed(gl,gg):
	# get camera local position when local frame origin is shifted to glyph point
	xcl = -gl[0]
	ycl = -gl[1]
	zcl = -gl[2]

	# global glyph x,y
	xgg = gg[0]
	ygg = gg[1]

	x = np.array([[xcl],[ycl],[zcl],[1.0]])

	Ttranslate = np.array([[1.0,0.0,0.0,xgg],[0.0,1.0,0.0,ygg],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0]])
	
	cg = np.dot(Ttranslate,x)
	
	xcg = cg.item(0)/cg.item(3)
	ycg = cg.item(1)/cg.item(3)

	return [xcg,ycg]

def readInCollectedArray():
	#read in collectedArray
	with open("collectedArray.txt","r") as f:
		collectedArray = f.readlines()
	for i in range(587):
		collectedArray[i] = collectedArray[i][:-1]
		collectedArray[i] = collectedArray[i].split(",")
		collectedArray[i] = int(collectedArray[i][0])
		#print collectedArray[i]
		#time.sleep(1)
	return collectedArray

def readInLocalTagPose():
	#read in localTagPose
	with open("localTagPose.txt","r") as f:
		localTagPose = f.readlines()
	for i in range(587):
		localTagPose[i] = localTagPose[i][:-1]
		localTagPose[i] = localTagPose[i].split(",")
		localTagPose[i][0] = float(localTagPose[i][0])
		localTagPose[i][1] = float(localTagPose[i][1])
		localTagPose[i][2] = float(localTagPose[i][2])
		localTagPose[i][3] = float(localTagPose[i][3])
		localTagPose[i][4] = float(localTagPose[i][4])
		localTagPose[i][5] = float(localTagPose[i][5])
		#print localTagPose[i]
		#time.sleep(1)
	return localTagPose

def readInGlobalCameraPosition():
	#read in globalCameraPosition
	with open("globalCameraPosition.txt","r") as f:
		globalCameraPosition = f.readlines()
	for i in range(587):
		globalCameraPosition[i] = globalCameraPosition[i][:-1]
		globalCameraPosition[i] = globalCameraPosition[i].split(",")
		globalCameraPosition[i][0] = float(globalCameraPosition[i][0])
		globalCameraPosition[i][1] = float(globalCameraPosition[i][1])
		globalCameraPosition[i][2] = float(globalCameraPosition[i][2])
		#print globalCameraPosition[i]
		#time.sleep(1)
	return globalCameraPosition

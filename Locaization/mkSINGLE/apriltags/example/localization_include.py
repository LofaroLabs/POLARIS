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



def readInLUT():
	#read in LUT
	with open("LUT.txt","r") as f:
		LUT = f.readlines()
	for i in range(587):
		LUT[i] = LUT[i][:-1]
		LUT[i] = LUT[i].split(",")
		LUT[i][0] = float(LUT[i][0])
		LUT[i][1] = float(LUT[i][1])
		LUT[i][2] = float(LUT[i][2])
		LUT[i][3] = int(LUT[i][3])
		print LUT[i]
		#time.sleep(1)
	return LUT

#getCameraPositionGlobal([-4.0,1.0],[-2.12,0.5],-0.7854)

#LUT = readInLUT()

#print str(LUT[(25*50)+9][0]+LUT[(25*50)+9][1])

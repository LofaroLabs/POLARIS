import math

TAGSIZE = 0.1778

def getRecognizedTags(ID):
	recognizedTags = []
#	print "RECOGNIZED IDS ZZZ: ", ID

	#for i in range(len(ID)):
		#print "ID IS : ", int(ID[i])

	count = 0
	for i in range(len(ID)):
		if(int(ID[i]) == 0):
			count += 1
			#print "count is: ", count

	for i in range(len(ID)):
		if(count == len(ID)):
			break
		elif(int(ID[i]) != 600):
			#print "RECOGNIZED ID: ",ID[i]
			recognizedTags.append(i)
		else:
			break

	return recognizedTags

#GOOD SINGLE
#check if any recently spotted tags have not been seen in awhile and should be ditched
def checkForGarbage(recognizedTags,ditch,ID,IDsaved):
	for i in range(8): #check for every recently spotted tag if it was recognized in current loop
		recognized = 0
		for j in range(len(recognizedTags)): 
			if(IDsaved[i] == ID[recognizedTags[j]]):
				recognized = 1
				break
		if(recognized == 1):
			ditch[i] = 0
		else:
			ditch[i] = ditch[i] + 1
	return ditch

#GARBAGE DAY D:< SINGLE
def garbageDay(ditch,IDsaved,RPYarray):
	for i in range(8):
		if(ditch[i] == 10):
			# ditch IDsaved[i], it hasn't been seen for 10 detections
			IDsaved[i] = -1 #make ID invalid
			ditch[i] = 0 #reset ditch value
			RPYarray[i][0] = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"] #get rid of
			RPYarray[i][1] = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"] #RPY data
			RPYarray[i][2] = ["NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"] #for ith pair
	return [IDsaved,RPYarray,ditch]



def placeNewData(recognizedTags,RPYarray,ROLL,PITCH,YAW,ditch,ID,IDsaved):
	#shift in new RPY data for all recognized tags
	for i in range(len(recognizedTags)):
		leadingTagIndex = recognizedTags[i]
		tagExists = -1
		for j in range(8):
			if(ID[recognizedTags[i]] == IDsaved[j]):
				tagExists = j
				break
			
		#if tag already has saved data
		if(tagExists != -1):
			#shift in new data at tagExists
			nullSpace = -1
			for m in range(10):
				if(str(RPYarray[tagExists][0][m]) == "NULL"):
					nullSpace = m
					break
			if(nullSpace != -1):
				#put data in nullSpace
				RPYarray[tagExists][0][nullSpace] = ROLL[leadingTagIndex]
				RPYarray[tagExists][1][nullSpace] = PITCH[leadingTagIndex]
				RPYarray[tagExists][2][nullSpace] = YAW[leadingTagIndex]
			else:
				#shift out old data and in new data
				old0 = RPYarray[tagExists][0][:-1]
				old1 = RPYarray[tagExists][1][:-1]
				old2 = RPYarray[tagExists][2][:-1]
				old0.append(ROLL[leadingTagIndex])
				old1.append(PITCH[leadingTagIndex])
				old2.append(YAW[leadingTagIndex])
				RPYarray[tagExists][0] = old0
				RPYarray[tagExists][1] = old1
				RPYarray[tagExists][2] = old2
		else: 
			#recognized tag is not already stored, look for free spot
			freeSpot = -1
			for k in range(8):
				if(IDsaved[k] == -1):
					freeSpot = k
					break
			#if a free spot exists for new data
			if(freeSpot != -1):			
				#put in new data at freeSpot
				#print "ID:", IDsaved
				IDsaved[freeSpot] = ID[recognizedTags[i]]
				ditch[freeSpot] = 0
				RPYarray[freeSpot][0][0] = ROLL[leadingTagIndex]
				RPYarray[freeSpot][1][0] = PITCH[leadingTagIndex]
				RPYarray[freeSpot][2][0] = YAW[leadingTagIndex]
			else:
				#find highest ditch value
				maxDitch = -1
				maxDitchIndex = -1
				for l in range(8):
					if(ditch[l] > maxDitch):
						maxDitch = ditch[l]
						maxDitchIndex = l
				#ditch old data and replace with new data at maxDitchIndex
				IDsaved[maxDitchIndex] = ID[recognizedPairs[i]]
				ditch[maxDitchIndex] = 0
				RPYarray[maxDitchIndex][0] = [ROLL[leadingTagIndex],"NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
				RPYarray[maxDitchIndex][1] = [PITCH[leadingTagIndex],"NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
				RPYarray[maxDitchIndex][2] = [YAW[leadingTagIndex],"NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL","NULL"]
	return [RPYarray,ditch,IDsaved]


def calculateRollingAvg(IDsaved,RPYarray):
	#calculate rolling averages for RPY of all pairs
	calculatedRollingAvg = [["NULL","NULL","NULL"],["NULL","NULL","NULL"],["NULL","NULL","NULL"],["NULL","NULL","NULL"],["NULL","NULL","NULL"],["NULL","NULL","NULL"],["NULL","NULL","NULL"],["NULL","NULL","NULL"]]

	# calculate distance vector for weighted averages
#	distance  = [0,0,0,0,0,0,0,0]
#	for l in range(len(recognizedPairs)):
#		for m in range(8):
#			if((IDsaved[m][0] == ID[recognizedPairs[l][0]) and (IDsaved[m][1] == ID[recognizedPairs[l][1])):
#				distance[m] = math.sqrt((XCoord[recognizedPairs[l][0]]*XCoord[recognizedPairs[l][0]])+(YCoord[recognizedPairs[l][0]]*YCoord[recognizedPairs[l][0]]))
#				break	

	for i in range(8):
		if(IDsaved[i] != -1):
			

			# roll
			sumR = 0.0
			nR = 0.0
			for j in range(10):
				if(str(RPYarray[i][0][j]) != "NULL"):
					sumR = sumR + RPYarray[i][0][j]
					nR = nR + 1.0
			if(nR != 0):
				calculatedRollingAvg[i][0] = sumR/nR

			#pitch
			sumP = 0.0
			nP = 0.0
			for j in range(10):
				if(str(RPYarray[i][1][j]) != "NULL"):
					sumP = sumP + RPYarray[i][1][j]
					nP = nP + 1.0
			if(nP != 0):
				calculatedRollingAvg[i][1] = sumP/nP

			#yaw
			sumY = 0.0
			nY = 0.0
			for j in range(10):
				if(str(RPYarray[i][2][j]) != "NULL"):
					sumY = sumY + RPYarray[i][2][j]
					nY = nY + 1.0
			if(nY != 0):
				calculatedRollingAvg[i][2] = sumY/nY

	return calculatedRollingAvg

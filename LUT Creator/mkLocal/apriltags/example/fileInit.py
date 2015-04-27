#write collected array
f = open("collectedArray.txt", "w")

for i in range(587):
	f.write("0"+"\n")

f.close()

#write local 6D tag pose
f = open("localTagPose.txt", "w")

for i in range(587):
	#x,y,z,R,P,Y
	f.write("0.0"+","+"0.0"+","+"0.0"+","+"0.0"+","+"0.0"+","+"0.0"+"\n")

f.close()

#write global camera position
f = open("globalCameraPosition.txt", "w")

for i in range(587):
	#x,y,YAW
	f.write("0.0"+","+"0.0"+","+"0.0"+"\n")

f.close()


f = open("LUT.txt", "w")

for i in range(587):
	f.write(str(i)+","+str(i)+","+"0.0"+","+"4"+"\n")

f.close()

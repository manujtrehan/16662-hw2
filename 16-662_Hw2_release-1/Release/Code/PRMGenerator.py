import Locobot
import numpy as np
import matplotlib.pyplot as plt
import random
import pickle
import RobotUtil as rt
import time

random.seed(13)

#Initialize robot object
mybot=Locobot.Locobot()


#Create environment obstacles - # these are blocks in the environment/scene (not part of robot) 
pointsObs=[]
axesObs=[]

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[0.275,-0.15,0.]),[0.1,0.1,1.05])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[0.275,0.05,0.425]),[0.1,0.3,0.1])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[0.275,0.25,0.4]),[0.1,0.1,0.15])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[0.425,0.25,0.375]),[0.2,0.1,0.1])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[-0.1,0.0,0.675]),[0.45,0.15,0.1])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[-0.275,0.0,0.]),[0.1,1.0,1.25])
pointsObs.append(envpoints), axesObs.append(envaxes)

# base and mount
envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[0.,0.0,0.05996]),[0.35004,0.3521,0.12276])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[-0.03768,0.0,0.36142]),[0.12001,0.26,0.5])
pointsObs.append(envpoints), axesObs.append(envaxes)


#TODO_ Create PRM - generate collision-free vertices
prmVertices=[]
prmEdges=[]
start = time.time()

while len(prmVertices)<1000:
	# sample random poses 
	print(len(prmVertices))

	angs = np.array(mybot.SampleRobotConfig())
	if not mybot.DetectCollision(angs, pointsObs, axesObs):
		prmVertices.append(angs)
		prmEdges.append([])
		for idx, val in enumerate(prmVertices[:-1]):
			if np.linalg.norm(angs - val) <= 2:
				# valid neighbour
				if not mybot.DetectCollisionEdge(val, angs, pointsObs, axesObs, 25):
					prmEdges[-1].append(idx)
					prmEdges[idx].append(len(prmEdges)-1)



#Save the PRM such that it can be run by PRMQuery.py
f = open("myPRM.p", 'wb')
pickle.dump(prmVertices, f)
pickle.dump(prmEdges, f)
pickle.dump(pointsObs, f)
pickle.dump(axesObs, f)
f.close

print("\n",len(prmVertices),": ", time.time()-start)






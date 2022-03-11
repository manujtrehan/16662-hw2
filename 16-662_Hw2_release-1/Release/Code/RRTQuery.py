import time
import pickle
from matplotlib.pyplot import axes
import numpy as np

import vrep_interface as vpi
import RobotUtil as rt
import Locobot

def FindNearest(prevPoints,newPoint):
	D=np.array([np.linalg.norm(np.array(point)-np.array(newPoint)) for point in prevPoints])
	return D.argmin()


np.random.seed(1234)
deg_to_rad = np.pi/180.

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
	

# define start and goal
deg_to_rad = np.pi/180.
qInit=[-80.*deg_to_rad, 0., 0., 0., 0.] 
qGoal=[0., 60*deg_to_rad, -75*deg_to_rad, -75*deg_to_rad, 0.] 


#TODO - Create RRT to find path to a goal configuration
rrtVertices=[]
rrtEdges=[]

rrtVertices.append(np.array(qInit))
rrtEdges.append(0)

thresh = 0.1 # 0.25
stopping_thresh = 0.15 # 0.3
FoundSolution=False

while len(rrtVertices)<3000 and not FoundSolution:
	print(len(rrtVertices))

	angs = np.array(mybot.SampleRobotConfig())

	if np.random.uniform(0, 1) < 0.1: angs = qGoal # goal bias

	# find nearest
	idx = FindNearest(rrtVertices, angs)
	near_point = rrtVertices[idx]
	old_point = near_point.copy()
	# print("=====================")
	
	# keep extending near_point until obstacle or angs
	while np.linalg.norm(np.array(angs) - near_point) > thresh:
		near_point = near_point + 0.3*thresh*(np.array(angs) - near_point)/np.linalg.norm(np.array(angs) - near_point)
		# check collisions
		if mybot.DetectCollision(near_point, pointsObs, axesObs): break # continue
		if mybot.DetectCollisionEdge(old_point, near_point, pointsObs, axesObs, 25): break # continue
		old_point = near_point

	# if np.linalg.norm(np.array(angs) - near_point) > thresh:
	# 	angs = near_point + thresh*(np.array(angs) - near_point)/np.linalg.norm(np.array(angs) - near_point)

	# # check collisions
	# if mybot.DetectCollision(angs, pointsObs, axesObs): continue
	# if mybot.DetectCollisionEdge(near_point, angs, pointsObs, axesObs): continue

	
	# if np.linalg.norm(old_point - rrtVertices[idx]) < thresh/10: continue # no steps were taken, sample another point
	# print("added")
	
	# rrtVertices.append(angs)
	rrtVertices.append(old_point)
	rrtEdges.append(idx)

	# check if near goal
	# if np.linalg.norm(np.array(qGoal) - angs) < stopping_thresh:
	if np.linalg.norm(np.array(qGoal) - old_point) < stopping_thresh:
		rrtVertices.append(np.array(qGoal))
		rrtEdges.append(len(rrtEdges)-1)
		FoundSolution = True
		print("Found")
		# break
			


### if a solution was found
		
if FoundSolution:
	# Extract path
	plan=[]
	c=-1 #Assume last added vertex is at goal 
	plan.insert(0, rrtVertices[c])

	while True:
		c=rrtEdges[c]
		plan.insert(0, rrtVertices[c])
		if c==0:
			break

	# TODO - Path shortening
	for i in range(500):
		idxA = np.random.randint(0, len(plan)-2)
		idxB = np.random.randint(idxA+1, len(plan)-1)

		moveA = np.random.uniform(0, 1) # move ratio along edge
		moveB = np.random.uniform(0, 1)

		newA = plan[idxA] + moveA*(plan[idxA+1] - plan[idxA]) # /np.linalg.norm(plan[idxA+1] - plan[idxA])
		newB = plan[idxB] + moveB*(plan[idxB+1] - plan[idxB]) # /np.linalg.norm(plan[idxB+1] - plan[idxB])

		if not mybot.DetectCollisionEdge(newA, newB, pointsObs, axesObs, int(np.linalg.norm(newB-newA)/0.1)):
			while idxB > idxA:
				plan.pop(idxB)
				idxB -= 1
			plan.insert(idxA+1, newB)
			plan.insert(idxA+1, newA)



		
	robot=vpi.vBot()
	robot.connect()
	
	for q in plan:
		robot.move(q)
		time.sleep(1)

	robot.destroy()

else:
	print("No solution found")




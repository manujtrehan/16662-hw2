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


np.random.seed(0)
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

rrtVertices.append(qInit)
rrtEdges.append(0)

thresh=0.25
stopping_thresh = 0.1
FoundSolution=False

while len(rrtVertices)<3000 and not FoundSolution:
	print(len(rrtVertices))

	angs = mybot.SampleRobotConfig()

	if np.random.uniform(0, 1) < 0.07: angs = qGoal # goal bias

	# find nearest
	idx = FindNearest(rrtVertices, angs)
	new_point = rrtVertices[idx]
	old_point = rrtVertices[idx]

	while(np.linalg.norm(np.array(angs - new_point)) > stopping_thresh):
		# keep extending new_point until obstacle or angs
		new_point += list(thresh*(np.array(angs - new_point)/np.linalg.norm(np.array(angs - new_point))))

		# check collisions
		if mybot.DetectCollision(new_point, pointsObs, axesObs): break
		if mybot.DetectCollisionEdge(old_point, new_point, pointsObs, axesObs): break

		old_point = new_point
	
	if old_point == rrtVertices[idx]: continue # no steps were taken, sample another point

	rrtVertices.append(old_point)
	rrtEdges.append(idx)

	# check if near goal
	if(np.linalg.norm(np.array(qGoal - old_point)) < stopping_thresh):
		FoundSolution = True
		break
			


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
	# for i in range(150):
	# 	idxA = np.random.randint(0, len(plan)-2)
	# 	idxB = np.random.randint(idxA, len(plan)-1)

	# 	moveA = np.random.uniform(0, 1) # move ratio along edge
	# 	moveB = np.random.uniform(0, 1)

	# 	newA = plan[idxA] + list(moveA*(np.array(plan[idxA+1] - plan[idxA]))) # /np.linalg.norm(np.array(plan[idxA+1] - plan[idxA]))
	# 	newB = plan[idxB] + list(moveB*(np.array(plan[idxB+1] - plan[idxB]))) # /np.linalg.norm(np.array(plan[idxB+1] - plan[idxB]))

	# 	if not mybot.DetectCollisionEdge(newA, newB, pointsObs, axesObs):
	# 		while idxB >= idxA:
	# 			plan.pop(idxA)
	# 			idxB -= 1
	# 		plan.insert(idxA, newB)
	# 		plan.insert(idxA, newA)



		
	robot=vpi.vBot()
	robot.connect()
	
	for q in plan:
		robot.move(q)
		time.sleep(1)

	robot.destroy()

else:
	print("No solution found")




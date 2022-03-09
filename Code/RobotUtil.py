import numpy as np 
import math

def rpyxyz2H(rpy,xyz):
	Ht=[[1,0,0,xyz[0]],
	    [0,1,0,xyz[1]],
            [0,0,1,xyz[2]],
            [0,0,0,1]]

	Hx=[[1,0,0,0],
	    [0,math.cos(rpy[0]),-math.sin(rpy[0]),0],
            [0,math.sin(rpy[0]),math.cos(rpy[0]),0],
            [0,0,0,1]]

	Hy=[[math.cos(rpy[1]),0,math.sin(rpy[1]),0],
            [0,1,0,0],
            [-math.sin(rpy[1]),0,math.cos(rpy[1]),0],
            [0,0,0,1]]

	Hz=[[math.cos(rpy[2]),-math.sin(rpy[2]),0,0],
            [math.sin(rpy[2]),math.cos(rpy[2]),0,0],
            [0,0,1,0],
            [0,0,0,1]]

	H=np.matmul(np.matmul(np.matmul(Ht,Hz),Hy),Hx)

	return H

def R2axisang(R):
	ang = math.acos(( R[0,0] + R[1,1] + R[2,2] - 1)/2)
	Z = np.linalg.norm([R[2,1] - R[1,2], R[0,2] - R[2,0], R[1,0] - R[0,1]])
	if Z==0:
		return[1,0,0], 0.
	x = (R[2,1] - R[1,2])/Z
	y = (R[0,2] - R[2,0])/Z
	z = (R[1,0] - R[0,1])/Z 	

	return[x, y, z], ang


def BlockDesc2Points(H, Dim):
	center = H[0:3,3]
	axes=[ H[0:3,0],H[0:3,1],H[0:3,2]]	

	corners=[
		center,
		center+(axes[0]*Dim[0]/2.)+(axes[1]*Dim[1]/2.)+(axes[2]*Dim[2]/2.),
		center+(axes[0]*Dim[0]/2.)+(axes[1]*Dim[1]/2.)-(axes[2]*Dim[2]/2.),
		center+(axes[0]*Dim[0]/2.)-(axes[1]*Dim[1]/2.)+(axes[2]*Dim[2]/2.),
		center+(axes[0]*Dim[0]/2.)-(axes[1]*Dim[1]/2.)-(axes[2]*Dim[2]/2.),
		center-(axes[0]*Dim[0]/2.)+(axes[1]*Dim[1]/2.)+(axes[2]*Dim[2]/2.),
		center-(axes[0]*Dim[0]/2.)+(axes[1]*Dim[1]/2.)-(axes[2]*Dim[2]/2.),
		center-(axes[0]*Dim[0]/2.)-(axes[1]*Dim[1]/2.)+(axes[2]*Dim[2]/2.),
		center-(axes[0]*Dim[0]/2.)-(axes[1]*Dim[1]/2.)-(axes[2]*Dim[2]/2.)
		]	
	# returns corners of BB and axes
	return corners, axes



def CheckPointOverlap(pointsA,pointsB,axis):	
	# TODO: check if sets of points projected on axis are overlapping
	
	projA = np.matmul(np.array(axis), np.array(pointsA).T)
	projB = np.matmul(np.array(axis), np.array(pointsB).T)
	
	min_a = np.min(projA)
	max_a = np.max(projA)
	min_b = np.min(projB)
	max_b = np.max(projB)

	if max_a < min_b or max_b < min_a:
		return False

	return True



def CheckBoxBoxCollision(pointsA,axesA,pointsB,axesB):

	#Sphere check
	if np.linalg.norm(pointsA[0]-pointsB[0])> (np.linalg.norm(pointsA[0]-pointsA[1])+np.linalg.norm(pointsB[0]-pointsB[1])):
		return False

	#TODO - SAT cuboid-cuboid collision check

	# Surface normal check
	for a, b in zip(axesA, axesB):
		if not CheckPointOverlap(pointsA, pointsB, a): return False
		if not CheckPointOverlap(pointsA, pointsB, b): return False
	
	# Edge check
	for a in axesA:
		for b in axesB:
			if not CheckPointOverlap(pointsA, pointsB, np.cross(np.array(a), np.array(b))): return False

	return True
	

# H_ref = rpyxyz2H([0, 0, 0], [0, 0, 0])
# Dim_ref = [3, 1, 2]

# H_list = []
# H_list.append(rpyxyz2H([0, 0, 0], [0, 1, 0]))
# H_list.append(rpyxyz2H([1, 0, 1.5], [1.5, -1.5, 0]))
# H_list.append(rpyxyz2H([0, 0, 0], [0, 0, -1]))
# H_list.append(rpyxyz2H([0, 0, 0], [3, 0, 0]))
# H_list.append(rpyxyz2H([0.5, 0, 0.4], [-1, 0, -2]))
# H_list.append(rpyxyz2H([-0.2, 0.5, 0], [1.8, 0.5, 1.5]))
# H_list.append(rpyxyz2H([0, 0.785, 0.785], [0, -1.2, 0.4]))
# H_list.append(rpyxyz2H([0, 0, 0.2], [-0.8, 0, -0.5]))

# Dim_list = []
# Dim_list.append([0.8, 0.8, 0.8])
# Dim_list.append([1, 3, 3])
# Dim_list.append([2, 3, 1])
# Dim_list.append([3, 1, 1])
# Dim_list.append([2, 0.7, 2])
# Dim_list.append([1, 3, 1])
# Dim_list.append([1, 1, 1])
# Dim_list.append([1, 0.5, 0.5])

# ref_corners, ref_axis = BlockDesc2Points(H_ref, Dim_ref)

# for h, d in zip(H_list, Dim_list):
# 	corners, axis = BlockDesc2Points(h, d)
# 	print(CheckBoxBoxCollision(ref_corners, ref_axis, corners, axis))
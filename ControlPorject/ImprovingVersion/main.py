from BuggySimulator import *
import numpy as np
from controller import *
from util import *


# get the trajectory
traj = get_trajectory('buggyTrace.csv')
# initial the Buggy
vehicle = initail(traj, 0)
n = 20000
X = []
Y = []
delta = []
xd = []
yd = []
phi = []
phid = []
deltad = []
F = []
minDist =[]
'''
your code starts here
'''
# preprocess the trajectory
passMiddlePoint = False
nearGoal = False
for i in range(n):
    command = controller()
    vehicle.update(command = command)



    # termination check
    disError,nearIdx = closest_node(vehicle.state.X, vehicle.state.Y, traj)
    stepToMiddle = nearIdx - len(traj)/2.0
    if abs(stepToMiddle) < 100.0:
        passMiddlePoint = True
        print('middle point passed')
    nearGoal = nearIdx >= len(traj)-50
    if nearGoal and passMiddlePoint:
        print('destination reached!')
        break
    # record states
    X.append(vehicle.state.X)
    Y.append(vehicle.state.Y)
    delta.append(vehicle.state.delta)
    xd.append(vehicle.state.xd)
    yd.append(vehicle.state.yd)
    phid.append(vehicle.state.phid)
    phi.append(vehicle.state.phi)
    deltad.append(command.deltad)
    F.append(command.F)
    minDist.append(disError)


showResult(traj,X,Y,delta,xd,yd,F,phi,phid,minDist)
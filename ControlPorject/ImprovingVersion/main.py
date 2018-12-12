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
forward = 20
vD = 5
prevDelta = 0
eV = vD - 0
'''
your code starts here
'''
# preprocess the trajectory
passMiddlePoint = False
nearGoal = False
ctrl = Controller(traj, vehicle, vD)
for i in range(n):
    
    _, idx = closest_node(vehicle.state.X, vehicle.state.Y, traj)
    currentPos = [vehicle.state.X, vehicle.state.Y]
    e1 = (traj[idx, 0] - currentPos[0]) / np.sin(vehicle.state.phi)
    
    trakLenth = len(traj)
    ahead = idx + forward
    if (ahead >= trakLenth):
        ahead = trakLenth - 1
        
    aheadPos = traj[ahead, :]
    posDifference = aheadPos - currentPos
    desiredPsi = wrap2pi(np.arctan2(posDifference[1], posDifference[0]))
    e2 = wrap2pi(vehicle.state.phi) - desiredPsi
    e1d = vehicle.state.yd + vehicle.state.xd * e2
    e2d = vehicle.state.phid - vehicle.state.xd * ctrl.curvatureK[ahead]
    deltaD, force, deltaPrev = ctrl.getController(e1, e1d, e2, e2d, eV, prevDelta)
    command = vehicle.command(force, deltaD)
    eV = vD - vehicle.state.xd
    
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
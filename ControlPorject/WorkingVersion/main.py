from BuggySimulator import *
import numpy as np
from controller import *
from util import *
from BuggySimulator import vehicle
from util import wrap2pi
from scipy.ndimage import gaussian_filter1d
from Evaluation import *

'''
THIS IS A POORLY WRITTEN AND DOCUMENTED CODE
THE GREEK LETTER IS PSI NOT PHI
INDICATE WHICH PART OF THE CODE CAN BE MODIFIED
'''

# get the trajectory
traj = get_trajectory('buggyTrace.csv')
# initial****ize*** the Buggy
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

lr = vehicle.lr
lf = vehicle.lf
Ca = vehicle.Ca
Iz = vehicle.Iz
f = vehicle.f
m = vehicle.m
g = vehicle.g
    # random vehicle speed
forward = 130
evPrev = 0.0

'''
your code starts here
'''
def lqrd(A, B, Q, R):
    '''Continus or discrete'''
    X = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q,R))
    K = np.matrix(scipy.linalg.inv(B.T*X*B+R)*(B.T*X*A))
    ev, _ = scipy.linalg.eig(A-B*K)
    
    return K, X, ev

def lqr(A, B, Q, R):
    X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))
 
#compute the LQR gain
    K = np.matrix(scipy.linalg.inv(R)*(B.T*X))
 
    eigVals,_ = scipy.linalg.eig(A-B*K)
 
    return K, X, eigVals

Vx = 8.3
A22 = -4 * Ca / (m * Vx)
A23 = 4 * Ca / m
A24 = 2 * Ca * (lr - lf) / (m * Vx)
A42 = -2 * Ca * (lf - lr) / (Iz * Vx)
A43 = 2 * Ca * (lf - lr) / Iz
A44 = -2 * Ca * (lf**2 + lr**2) / (Iz * Vx)
A = np.matrix([[0,  1,  0,  0],
               [0,  A22,A23,A24],
               [0,  0,  0,  1],
               [0,  A42,A43,A44]])


B2 = 2 * Ca / m
B4 = 2 * Ca * lf / Iz
B = np.matrix([[0],
               [B2],
               [0],
               [B4]])

Q = np.matrix([[1,    0,  0,  0],
               [0,     10, 0,  0],
               [0,      0,  1,  0],
               [0,      0,  0,  25]])

R = np.matrix([[1]])

K, S, E = lqr(A, B, Q, R)

firstDev = np.gradient(traj, axis = 0)
firstDev = gaussian_filter1d(firstDev,2)
secondDev = np.gradient(firstDev, axis = 0)
secondDevDev = gaussian_filter1d(secondDev,2)
kNum = ((firstDev[:,0] * secondDev[:,1]) - (firstDev[:,1] * secondDev[:,0]))
kDen = np.power(np.square(firstDev[:,0]) + np.square(firstDev[:,1]), 3/2)
KCurve = kNum / kDen

#psiDes = np.arctan(firstDev[:,1]/firstDev[:,0])


deltaPrev = 0
# preprocess the trajectory
passMiddlePoint = False
nearGoal = False


for i in range(n):
    Vx = 8.3
    _ , idx = closest_node(vehicle.state.X, vehicle.state.Y, traj)
    
    if (idx + forward >= len(KCurve)):
        ahead = idx + forward - len(KCurve)
    else:
        ahead = idx + forward
        
    
        
    currentPos = [vehicle.state.X, vehicle.state.Y]
    #desiredPos = traj[ahead,:]
    desiredPos = traj[idx,:]
    diffPos = currentPos - desiredPos
    psiDesired = wrap2pi(np.arctan2(diffPos[1], diffPos[0]))
    normalVec = [-np.sin(vehicle.state.phi), np.cos(vehicle.state.phi)]
    #normalVec = normalVec / np.linalg.norm(normalVec)
    #diffPos = currentPos - desiredPos
    #normVec = findOrthVec(firstDev[idx])
    
    #normVec = normVec / np.linalg.norm(normVec)
    e1 = np.inner(diffPos, normalVec)
    
    print(e1)
    #print(idx)
    
    
    
    aheadPos = traj[ahead]
    aheadDiff = (aheadPos - currentPos)
    psiDes = np.arctan2(aheadDiff[1], aheadDiff[0])  
    e2 = wrap2pi(vehicle.state.phi - psiDes)
    #e2 = wrap2pi(vehicle.state.phi) - psiDesired  
    e1d = vehicle.state.yd + vehicle.state.xd * e2
    e2d = vehicle.state.phid - vehicle.state.xd * KCurve[ahead]
    eV = Vx - vehicle.state.xd
    Fout, currDeltad, deltaPrev = controller(e1, e1d, e2, e2d, K, eV, evPrev, vehicle.state.delta)
   
    command = vehicle.command(Fout, currDeltad)
    vehicle.update(command = command)
    evPrev = -(vehicle.state.xd - Vx)



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

toReturn = np.array([xd, yd, phid, delta, X, Y, phi]).T
np.save("24-677_Project_2_BuggyStates_Qiaojie.npy", toReturn)
taskNum = 3
evaluation(minDist, traj, X, Y, taskNum)
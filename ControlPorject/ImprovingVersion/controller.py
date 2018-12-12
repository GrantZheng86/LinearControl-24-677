'''
This is a realization of the controller from Vehicle Dynamics and Control by Rajesh Rajamani.
Yaohui Guo
'''
from BuggySimulator import *
import numpy as np
import scipy
import control
from scipy.ndimage import gaussian_filter1d
from util import *

class Controller:
    
    def __init__ (self, traj, vehicle, vD):
        self.traj = traj
        self.vehicle = vehicle
        self.lookForward = 50
        self.vD = vD
        self.curvatureK = self.calucalteCurvatureK()
        self.gainK = self.calculateGainK()
    
    def calculateABMatrix(self):
        lr = self.vehicle.lr
        lf = self.vehicle.lf
        Ca = self.vehicle.Ca
        Iz = self.vehicle.Iz
        f  = self.vehicle.f
        m  = self.vehicle.m
        
        A22 = -4 * Ca / (m * self.vD)
        A23 = 4 * Ca / m
        A24 = 2 * Ca * (lr - lf) / (m * self.vD)
        A42 = -2 * Ca * (lf - lr) / (Iz * self.vD)
        A43 = 2 * Ca * (lf - lr) / Iz
        A44 = -2 * Ca * (lf**2 + lr**2) / (Iz * self.vD)
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
        
        return A, B
        
    def calculateGainK(self):
        K, _, _ = self.lqr()
        return K 
       
    def calucalteCurvatureK(self):
        firstDer = np.gradient(self.traj, axis = 0)
        secondDer = np.gradient(firstDer, axis = 0)
        num = firstDer[:,0] * secondDer[:,1] - firstDer[:,1] * secondDer[:, 0]
        num = np.abs(num)
        denom = np.power(np.square(firstDer[:, 0]) + np.square(firstDer[:, 1]), 3/2)
        return num /denom
    
    def lqr(self):
        A, B = self.calculateABMatrix()
        Q = np.matrix([[18, 0, 0, 0],
                       [0, 10, 0, 0],
                       [0, 0, 25, 0],
                       [0, 0, 0, 0.01]])
        R = np.matrix([[0.01]])
        X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))
        K = np.matrix(scipy.linalg.inv(R)*(B.T*X))  
        eigVals,_ = scipy.linalg.eig(A-B*K)
        return K, X, eigVals
    
          
    def getController(self, e1, e1d, e2, e2d, eV, prevDelta):
        x = np.array([e1, e1d, e2, e2d])
        delta = np.asscalar(np.dot(self.gainK, x.T))
        toReturnDeltaDot = (prevDelta - delta) / 0.05
        toReturnF = eV * 100
        return toReturnDeltaDot, toReturnF, delta
        #TODO: Return controller parameters for the control
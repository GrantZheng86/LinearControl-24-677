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
        self.lookForward = 25
    
    def calculateABMatrix(self):
        lr = self.vehicle.lr
        lf = self.vehicle.lf
        Ca = self.vehicle.Ca
        Iz = self.vehicle.Iz
        f  = self.vehicle.f
        m  = self.vehicle.m
        
    def calucalteCurvatureK(self):
        firstDer = gaussian_filter1d(self.traj, 1)
        secondDer = gaussian_filter1d(self.traj, 2)
            
    def getController(self):
        #TODO: Return controller parameters for the control
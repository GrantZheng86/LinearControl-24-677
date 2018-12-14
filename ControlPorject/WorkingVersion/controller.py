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
from BuggySimulator import wrap2pi
'''
WHY DO YOU JUST HAVE A FUNCTION THAT LITERALLY DOES NOTHING
BUT RETURN A ZERO HERE? AND IMPORT BUNCH OF NOT USED PACKAGES???
IF IT IS A PLACE HOLDER, SAY THAT IN THE DOCUMENT!!!
'''
def controller(e1, e1d, e2, e2d, K, eV, evPrev, deltaPrev):
    """
    controller(e1, e1Dot, e2, e2Dot, controllerGain, velocityError, previousVelocityError, previousDelta)
    
    Parameters
    ----------
    e1: error in lateral distances, the first element in the state
    e1d: first derivative of e1
    e2: error in yaw angle, the current desired yaw angle - current angle
    """
    # pd controller for calculating F
    kp = 185
    kd = 0
    toReturn_F = kp * eV + kd * (eV - evPrev)
    
    
    
    x = np.array([e1,e1d,e2,e2d])
    
    
    delta = np.asscalar(np.dot(-K, x.T))
    toReturn_deltad = (delta - deltaPrev) / 0.05
    
    return toReturn_F, toReturn_deltad, delta
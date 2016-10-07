import math
import numpy as np

from rexarm import Rexarm

PI = np.pi
D2R = PI/180.00
R2D = 180.0 / 3.1415926
ANGLE_TOL = 2*PI/180

"""
FK Constant
"""
DH1_D = 116
DH3_A = 100
DH4_A = 110


L1 = DH1_D
L2 = DH3_A
L3 = DH3_A
L4 = DH4_A


"""State Class"""
class StateManager():
    def __init__(self, rexarm):
        self.rexarm = rexarm
        State_MoveToFinalTarget state_MTFT(rexarm);
        state_MTFT.finaltarget = [152,224,36,35+90]




class State_MoveToFinalTarget():

    def __init__(self,rexarm):
        self.intermediatelocation = []
        self.finaltarget = finaltarget
        realtimelocation = [self.rexarm.P0[0],self.rexarm.P0[1],self.rexarm.P0[2],self.rexarm.T]

        

    def state_MTFT_iCheckIfArrived(self):
        self.rexarm.rexarm_FK(self.rexarm.joint_angles_fb)
        
    
    def state_MTFT_iAddIntermediateLocation(self,intermediatelocation):
        self.intermediatelocation.append(intermediatelocation);



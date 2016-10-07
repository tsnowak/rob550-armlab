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




ERROR_LOCAL_TOL_X = 30
ERROR_LOCAL_TOL_Y = 30
ERROR_LOCAL_TOL_Z = 30
ERROR_LOCAL_TOL_T = PI/40


"""State Class"""
class StateManager():
    def __init__(self, rexarm):
        self.rexarm = rexarm
        State_MoveToFinalTarget state_MTFT(rexarm);
        state_MTFT.finaltarget = [152,224,36,(35+90)*D2R]




class State_MoveToFinalTarget():

    """
    Innitialize the state State_MoveToFinalTarget
    """
    def __init__(self,rexarm):
        self.intermediatelocation = []
        self.finaltarget = finaltarget
        
        


    """
    Name: state_MTFT_iCheckIfArrived
    Function: check if the current arm has arrived the final target.
    """
    def state_MTFT_iCheckIfArrivedFinalTarget(self):
        self.rexarm.rexarm_FK(self.rexarm.joint_angles_fb)
        realtimelocationgesture = [self.rexarm.P0[0],self.rexarm.P0[1],self.rexarm.P0[2],self.rexarm.T]
        errorX = abs(realtimelocationgesture[0] - self.finaltarget[0]);
        errorY = abs(realtimelocationgesture[1] - self.finaltarget[1]);
        errorZ = abs(realtimelocationgesture[2] - self.finaltarget[2]);
        errorT = abs(realtimelocationgesture[3] - self.finaltarget[3]);
        if (errorX < ERROR_LOCAL_TOL_T and errorY < ERROR_LOCAL_TOL_Y and errorZ < ERROR_LOCAL_TOL_Z and errorT < ERROR_LOCAL_TOL_T):
            return True
        else:
            return False

    """
    name: state_MTFT_iAddIntermediateLocation
    Function: Add some intermediate location and orientation into the path, (probably can avoid some obstacles.)
    """
    def state_MTFT_iAddIntermediateLocation(self,intermediatelocation):
        self.intermediatelocation.append(intermediatelocation);

    def state_MTFT_iMoveArmToFinalLocation(self):
        self.rexarm.

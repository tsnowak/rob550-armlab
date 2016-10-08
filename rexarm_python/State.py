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


#Be sure to update the state definition at the head of the rexarm.

STATE_CODE_INIT  = 1
STATE_CODE_CCACFP = 2
STATE_CODE_OG = 3
STATE_CODE_MTFT = 4
STATE_CODE_CP = 5
STATE_CODE_MTB = 6
STATE_CODE_RP = 7
STATE_CODE_END = 8

STATE_CODE_MT_CI = 41
STATE_CODE_MT_GTNWPT = 42
STATE_CODE_MT_END = 43



"""State Class"""
class StateManager():

    def __init__(self, rexarm, video):
        
        self.rexarm = rexarm
        self.video = video

        self.state_INIT = State_InitialState(self.rexarm);
        self.state_CCAFNP = State_CameraCalibrationAndCalculateFindNextPokmon(self.rexarm);
        self.state_OG = State_OpenGripper(self.rexarm);
        self.state_MTFT = State_MoveToFinalTarget(self.rexarm);
        self.state_CP = State_CatchPokmon(self.rexarm);
        self.state_MTB = State_MoveToBall(self.rexarm);
        self.state_RP = State_ReleashPokmon(self.rexarm);
        self.State_END = State_END(self.rexarm);

        #self.state_MTFT_CI = State_MTFT_CalculateIntermediate(self.state_WTFT);
        #self.state_MTFT_GTNW = State_MTFT_GoToNextWaypoint(self.rexarm, self.state_WTFT);
        #self.state_MRFT_END = State_MTFT_End(self.rexarm);

        self.currentState = STATE_CODE_INIT

        """
        State Definition:
            
        1. INIT: State_InitialState
            This is the entrance state, People need to do the camera calibration.

        2. CCAFNP: State_CameraCalibrationAndCalculateFindNextPokmon
            This is the state when and after people have finished the caemra calibration.

        3. OG: State_OpenGripper
            Open the gripper.

        4. MTFT: State_MoveToFinalTarget
            The robot arm move to one certain pokmon location.

        5. CP: State_CatchPokmon
            The gripper catch the pokmon

        6. MTB: State_MoveToBall
            The arm move to the pokball's location.

        7. RP: State_ReleashPokmon
            Release the pokmon to the State_MoveToBall

        8. END: State_END:
            Finished the competition.

                    ________(not found pokmon)____________________________________________
                    |                                                                     |
                    |                                                                     |---> END
        INIT ---> CCAFNP ----(found pokmon) ---> OP ---> MTFT ----> CP --->MTB ---> RP
                    ^                                                               |
                    |                                                               |
                    |_______________________________________________________________| 
        
        



        ------MTFT---------------------------------------------------------------------


        MT_CI ---> GTNWPT


        """




    def Statemanager_MoveToNextState(self):



        """

        INIT ---> CCAFNP 

        """
        if (self.currentState == STATE_CODE_INIT):
            if (self.video.aff_flag == 2):#finished affine
                self.currentState = STATE_CODE_CCACFP
        
        
        """


        (Not Finished calibration.)
        -----
        |    |
        |   \/
        CCAFNP  ----- (found pokmon) ------>  END
            |
            |
            -----------(not found pokmon)---> OG

        
        """
        if (self.currentState == STATE_CODE_CCACFP):
            #CameraCalibrationAndCalculateFindNextPokmon
            
            #TODO: Send the calibration command.
            
            #self.video.blobDetector();

            #Check if the camera has finished calculation.
            print([self.video.whetherFinishedCam, self.video.numPokRemain])
            if (self.video.whetherFinishedCam == 1):#If finished 
                if (self.video.numPokRemain  == 0):  #IF no pokmon is remained.
                    self.currentState = STATE_CODE_END
                else:                                  #If found some pokmon:
                    self.currentState = STATE_CODE_OG
            #else:
            #    pass
        

        #Keep finding the location.
        """
        State_OpenGripper
        -----
        |    |
        |   \/
          OG ---------> MTFT
        
        """
        if (self.currentState == STATE_CODE_OG):
            if (self.state_OG.iOpenGripper(self.rexarm) == 1):
                print('[msg] Gripper opened.')
                self.currentState = STATE_CODE_MTFT
            else:# gripper not fully opened
                pass
        '''
        State_MoveToFinalTarget
        MTFT
        '''

        if (self.currentState == STATE_CODE_MTFT):
            print('[msg] Move to target')
            
            #initial
            if (self.state_MTFT.MT_currentstate == STATE_CODE_MT_CI):
                self.state_MTFT.state_MTFT_iSetCurrentLocationAsInitialLocation() #TODO
                #self.state_WTFT_CI.calculate()
                self.state_MTFT.MT_currentstate = STATE_CODE_MT_GTNWPT#go to next way point

            elif (self.state_MTFT.MT_currentstate == STATE_CODE_MT_GTNWPT):
                self.currentState = STATE_CODE_CP
            
            elif (self.state_MTFT.MT_currentstate == STATE_CODE_MT_END):# the last pokemon
                self.state_MTFT.MT_currentstate = STATE_CODE_MT_CI
                self.currentState = STATE_CODE_CP


        '''
        catch pokemon
        CP
        '''
        if (self.currentState == STATE_CODE_CP):

            #if finished
            self.currentState = STATE_CODE_MTB


        '''
        State_MoveToBall
        MTB
        '''
        if (self.currentState == STATE_CODE_MTB):

            #if finished
            self.currentState = STATE_CODE_RP


        '''
        State_ReleashPokmon
        RP 
        '''
        if (self.currentState == STATE_CODE_RP):

            #if finished
            self.currentState = STATE_CODE_CCACFP

        

    def StateManager_Test(self):

        self.state_MTFT.finaltarget = [152,224,36,(45+90)*D2R]
        self.state_MTFT.state_MTFT_iMoveArmToFinalLocation();

class State_InitialState():
    def __init__(self, rexarm):
        pass


class State_CameraCalibrationAndCalculateFindNextPokmon():
    def __init__(self, rexarm):
        pass

class  State_OpenGripper():
    def __init__(self,rexarm):
        self.rexarm = rexarm
        pass

    def iOpenGripper(self, rexarm):
        return self.rexarm.rexarm_gripper_grab(1);

    


class State_MoveToFinalTarget():

    """
    Innitialize the state State_MoveToFinalTarget
    """
    def __init__(self,rexarm):
        
        self.initialLocation = [0.0,0.0,0.0,0.0] #x,y,z,phi
        self.MT_currentstate = STATE_CODE_MT_CI


        self.intermediatelocation = [[0,0,0,0]]
        self.intermediatelocationnumber = 0
        self.intermediatelocationcurrentnumber = 0

        self.finaltarget  = [0.0]*4  #The data structure is [x,y,z,T]
        self.rexarm = rexarm


    """
    Name: state_MTFT_iCheckIfArrived
    Function: check if the current arm has arrived the final target.
    """
    def state_MTFT_iSetCurrentLocationAsInitialLocation(self):
        pass


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

    """
    name: state_MTFT_iMoveArmToFinalLocation(self).
    Function:Move the arm to the target position.
    """

    def state_MTFT_iMoveArmToFinalLocation(self):
        IK_return = self.rexarm.rexarm_IK(self.finaltarget,0)
        validity_1 = IK_return[0]
        validity_2 = IK_return[2]
        validity_3 = IK_return[4]
        validity_4 = IK_return[6]
        configuration_1 = IK_return[1]
        configuration_2 = IK_return[3]
        configuration_3 = IK_return[5]
        configuration_4 = IK_return[7]

        #TODO: Do the configuration selection.

        self.rexarm.iSetJointAngle(0,configuration_1[0])
        self.rexarm.iSetJointAngle(1,configuration_1[1])
        self.rexarm.iSetJointAngle(2,configuration_1[2])
        self.rexarm.iSetJointAngle(3,configuration_1[3])

        self.rexarm.cmd_publish();
        







class State_MTFT_CalculateIntermediate():#add points above pokemon and ball
    def __init__(self,wtft):
        pass
    def calculate():
        pass

class State_MTFT_GoToNextWaypoint():#cmd, check if arrived
    def __init__(self, rexarm, wtft):
        pass

class State_MTFT_End():# change state
    def __init__(self, rexarm):
        pass





class State_CatchPokmon():
    def __init__(self, rexarm):
        pass

class State_MoveToBall():
    def __init__(self, rexarm):
        pass

class State_ReleashPokmon():
    def __init__(self, rexarm):
        pass


class State_END():
    def __init__(self, rexarm):
        pass








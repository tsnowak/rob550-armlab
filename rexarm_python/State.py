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
ERROR_LOCAL_TOL_Z = 20
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
STATE_CODE_RAP = 9

STATE_CODE_MTFT_CI = 41
STATE_CODE_MTFT_GTNWPT = 42
STATE_CODE_MTFT_END = 43



STATE_CODE_MTB_CI = 41
STATE_CODE_MTB_GTNWPT = 42
STATE_CODE_MTB_END = 43



REACHABLE_MAX_DISTANCE  = 280
SHRINK_RANGE = 240
SHRINK_DISTANCE = 240

"""State Class"""
class StateManager():

    def __init__(self, rexarm, video):
        
        self.rexarm = rexarm
        self.video = video

        self.state_INIT = State_InitialState(self.rexarm)
        self.state_CCAFNP = State_CameraCalibrationAndCalculateFindNextPokmon(self.rexarm)
        self.state_OG = State_OpenGripper(self.rexarm)
        self.state_MTFT = State_MoveToFinalTarget(self.rexarm, self.video)
        self.state_CP = State_CatchPokmon(self.rexarm)
        self.state_MTB = State_MoveToBall(self.rexarm, self.video)
        self.state_RP = State_ReleashPokmon(self.rexarm)
        self.state_RAP = State_ResetArmPosition(self.rexarm, self.video)
        self.state_END = State_END(self.rexarm)
        
        self.state_MTFT_CI = State_MTFT_CalculateIntermediate(self.rexarm, self.state_MTFT, self.video)
        self.state_MTFT_GTNW = State_MTFT_GoToNextWaypoint(self.rexarm, self.state_MTFT)
        self.state_MTFT_END = State_MTFT_End(self.rexarm)


        self.state_MTB_CI = State_MTB_CalculateIntermediate(self.rexarm, self.state_MTB, self.video)
        self.state_MTB_GTNW = State_MTB_GoToNextWaypoint(self.rexarm, self.state_MTB)
        self.state_MTB_END = State_MTB_End(self.rexarm)



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

        8. RAP: State_ResetArmPosition
            Reset the arm's position to the vertical position.

        9. END: State_END:
            Finished the competition.



                    ________(not found pokmon)___________________________________________________
                    |                                                                            |
                    |                                                                            |---> END
        INIT ---> CCAFNP ----(found pokmon) ---> OP ---> MTFT ----> CP --->MTB ---> RP ---> RAP
                    ^                                     |        |                         |
                    |                                     |        |                         |
                    |____________________________________(|)______(|)________________________| 
                                                          |        |  
                                                          |        |________________
                                                          |                        |
                                                       MT_CI ---> MT_GTNWPT ----> MT_END
                                                                  |   ^
                                                                  |   |
                                                                  |___|

        ------MTFT---------------------------------------------------------------------




        """




    def Statemanager_MoveToNextState(self):



        """

        INIT ---> CCAFNP 

        """
        if (self.currentState == STATE_CODE_INIT):
            if (self.video.aff_flag == 2):#finished affine
                self.currentState = STATE_CODE_CCACFP
                print("[Sts]: STATE_CODE_CCACFP")
        
        
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
            
            self.video.blobDetector();

            #Check if the camera has finished calculation.
            #print([self.video.whetherFinishedCam, self.video.numPokRemain])
            if (self.video.whetherFinishedCam == 1):#If finished 
                if (self.video.numPokRemain  == 0):  #IF no pokmon is remained.
                    self.currentState = STATE_CODE_END
                    print("[Sts]: STATE_CODE_END")
                else:                                  #If found some pokmon:
                    self.currentState = STATE_CODE_OG
                    print("[Sts]: STATE_CODE_OG")
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
            self.rexarm.iSetSpeed(0.4);
            self.rexarm.iSetTorque(0.7);
            if (self.state_OG.iOpenGripper(self.rexarm) == 1):
                print('[msg] Gripper opened.')
                self.currentState = STATE_CODE_MTFT
                print("[Sts]: STATE_CODE_MTFT")
            else:# gripper not fully opened
                pass






        '''
        State_MoveToFinalTarget
        MTFT

                          MTFT     CP 
                          |        ^    
                          |        |                        
                          |        |________________
                          |                        |
                        MT_CI ---> MT_GTNWPT ----> MT_END
                                    |   ^
                                    |   |
                                    |___|


        '''

        if (self.currentState == STATE_CODE_MTFT):
            
            
            #initial
            if (self.state_MTFT.MT_currentstate == STATE_CODE_MTFT_CI):
                self.state_MTFT.state_MTFT_initialize()

                self.state_MTFT.state_MTFT_iSetCurrentLocationAsInitialLocation(self.rexarm) #TODO
                """
                Calculate the intermediate way point.
                """
                self.state_MTFT_CI.calculate()

                
                self.state_MTFT.MT_currentstate = STATE_CODE_MTFT_GTNWPT#go to next way point

            elif (self.state_MTFT.MT_currentstate == STATE_CODE_MTFT_GTNWPT):
                # Write function in the class state_MTFT_GTNW so that the arm can keep moving to the next way points.
                # 
                # state_MTFT_GTNW: calculate the inverse kinematics.
                # state_MTFT_GTNW: iSetJointAngles()
                # state_MTFT_GTNW: cmd_publish()
                # state_MTFT_GTNW: compare the current location with the way point's configuration.
                #                   If within error torrance, then up date the state_MTFT.intermediatelocationcurrentnumber += 1.
                #                    
                # state_MTFT_GTNW: if the currentintermediatewaypointnumber = total way point number, then go to the next state: state_MTFT_END
                if self.state_MTFT_GTNW.iGoToNextWayPoint() == True:
                    #change state -> end
                    self.state_MTFT.MT_currentstate = STATE_CODE_MTFT_END
                    
               
                
                #Temp comment here.
                #self.state_MTFT.MT_currentstate = STATE_CODE_MTFT_END
            elif (self.state_MTFT.MT_currentstate == STATE_CODE_MTFT_END):
                print('[Msg]: Done moving arm')
                self.state_MTFT.MT_currentstate = STATE_CODE_MTFT_CI#MT set back to initial state
                self.currentState = STATE_CODE_CP
                print("[Sts]: STATE_CODE_CP")

                

        '''
        catch pokemon: close gripper
        CP --> MTB
        '''
        if (self.currentState == STATE_CODE_CP):

            if (self.state_CP.iCloseGripper(self.rexarm) == 2):
                print('[Msg]: Gripper closed.')
                self.currentState = STATE_CODE_MTB
                print("[Sts]: STATE_CODE_MTB")

            else:# gripper not fully closed
                pass


        '''
        State_MoveToBall
        MTB-->RP
        '''
        if (self.currentState == STATE_CODE_MTB):
            
            
            #initial
            if (self.state_MTB.MTB_currentstate == STATE_CODE_MTB_CI):
                self.state_MTB.state_MTB_initialize()

                self.state_MTB.state_MTB_iSetCurrentLocationAsInitialLocation(self.rexarm) #TODO
                """
                Calculate the intermediate way point.
                """
                self.state_MTB_CI.calculate()

                
                self.state_MTB.MTB_currentstate = STATE_CODE_MTB_GTNWPT#go to next way point

            elif (self.state_MTB.MTB_currentstate == STATE_CODE_MTB_GTNWPT):
                # Write function in the class state_MTFT_GTNW so that the arm can keep moving to the next way points.
                # 
                # state_MTFT_GTNW: calculate the inverse kinematics.
                # state_MTFT_GTNW: iSetJointAngles()
                # state_MTFT_GTNW: cmd_publish()
                # state_MTFT_GTNW: compare the current location with the way point's configuration.
                #                   If within error torrance, then up date the state_MTFT.intermediatelocationcurrentnumber += 1.
                #                    
                # state_MTFT_GTNW: if the currentintermediatewaypointnumber = total way point number, then go to the next state: state_MTFT_END
                if self.state_MTB_GTNW.iGoToNextWayPoint() == True:
                    #change state -> end
                    self.state_MTB.MTB_currentstate = STATE_CODE_MTB_END
                    
               
                
                #Temp comment here.
                #self.state_MTFT.MT_currentstate = STATE_CODE_MTFT_END
            elif (self.state_MTB.MTB_currentstate == STATE_CODE_MTB_END):
                print('[Msg]: Done moving arm')
                self.state_MTB.MTB_currentstate = STATE_CODE_MTB_CI#MT set back to initial state
                self.currentState = STATE_CODE_RP
                print("[Sts]: STATE_CODE_RP")






        '''
        State_ReleashPokmon
        RP --> RAP
        '''
        if (self.currentState == STATE_CODE_RP):
            if (self.state_RP.iOpenGripper(self.rexarm) == 1):
                print('[Msg]: Gripper closed.')
                self.currentState = STATE_CODE_RAP
                print("[Sts]: STATE_CODE_RAP")

            else:# gripper not fully closed
                pass

       
            


        '''
         RAP ---> CCAFNP
        '''
        if (self.currentState == STATE_CODE_RAP):
            
            self.rexarm.iSetSpeed(1.0)
            self.rexarm.iSetTorque(1.0)

            if (self.state_RAP.iResetArmPosition() == True):

                #
                # iSetJointAngle.
                # cmd_publish() 
                # check if has arrived.
                #TODO: THe following code is not useful probably for competition. need to be changed.
                self.state_RAP.iResetOneLoopVariables()
                self.currentState = STATE_CODE_CCACFP


            else:
                pass


            

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

    







"""
#####################################################################################################################
                                         Class   M    T    F     T    Start
#####################################################################################################################
"""


class State_MoveToFinalTarget():

    """
    Innitialize the state State_MoveToFinalTarget
    """
    def __init__(self,rexarm, video):
        
        self.initialLocation = [0.0,0.0,0.0,0.0] #x,y,z,phi in [mm,mm,mm,rad]
        self.MT_currentstate = STATE_CODE_MTFT_CI


        self.intermediatelocation = [[0,0,0,0]]
        self.intermediatejointangle = [[0,0,0,0]]   
        self.intermediatelocationnumber = 0
        self.intermediatelocationcurrentnumber = 0
        self.rexarm = rexarm
        self.video = video
        self.finaltarget = [0,0]


    def state_MTFT_initialize(self):

        self.intermediatelocationnumber = 0
        self.intermediatelocationcurrentnumber = 0
        self.intermediatelocation = []
        self.intermediatejointangle = []
        self.finaltarget = []

    """
    Name: state_MTFT_iCheckIfArrived
    Function: check if the current arm has arrived the final target.
    """
    def state_MTFT_iSetCurrentLocationAsInitialLocation(self, rexarm):
        configuration = rexarm.joint_angles_fb;

        self.initialLocation =  rexarm.rexarm_FK(configuration)

        pass

    '''
    def state_MTFT_iCheckIfArrivedFinalTarget(self):
        self.rexarm.rexarm_FK(self.rexarm.joint_angles_fb)
        realtimelocationgesture = [self.rexarm.P0[0],self.rexarm.P0[1],self.rexarm.P0[2],self.rexarm.T]
        errorX = abs(realtimelocationgesture[0] - self.finaltarget[0]);
        errorY = abs(realtimelocationgesture[1] - self.finaltarget[1]);
        errorZ = abs(realtimelocationgesture[2] - self.finaltarget[2]);
        errorT = abs(realtimelocationgesture[3] - self.finaltarget[3]);
        if (errorX < ERROR_LOCAL_TOL_X and errorY < ERROR_LOCAL_TOL_Y and errorZ < ERROR_LOCAL_TOL_Z and errorT < ERROR_LOCAL_TOL_T):
            return True
        else:
            return False
    '''

    """
    name: state_MTFT_iAddIntermediateLocation
    Function: Add some intermediate location and orientation into the path, (probably can avoid some obstacles.)
    """
    def state_MTFT_iAddIntermediateLocation(self,intermediatelocation): #[x,y,z,phi]
         ###456
         

        configuration =  self.rexarm.rexarm_IK(intermediatelocation, 1)

        #print("=================+++++>> configuation:"),
        #print(configuration)
        
        if (configuration[0] == 0):
            print("[ERROR]:Inreachable!!!=========================================") 
            self.mtft.MTFT_currentstate = STATE_CODE_MTFT_END
            exit(1)
        else:

            self.intermediatejointangle.append(configuration[1])
            self.intermediatelocation.append(intermediatelocation)
            self.intermediatelocationnumber = self.intermediatelocationnumber + 1

   

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

        self.rexarm.cmd_publish()
        







class State_MTFT_CalculateIntermediate():#add points above pokemon and ball
    def __init__(self,rexarm, mtft, video):
        self.mtft = mtft
        self.rexarm = rexarm
        self.video = video



        pass
    


    """
    Name: calculate()
    Function: calculate the intermediate location of the path and change the following variables:

        self.intermediatelocation = [[0,0,0,0]]   : A list of 4 x 1 vectors, each vector mean the x,y,z,phi configuration of a certain way point
        self.intermediatelocationnumber = 0 : The number of vectors (i.e. Every location apart from the first one.)
        self.intermediatelocationcurrentnumber = 0: This is the current location number.

    """
    def calculate(self):

        self.mtft.finaltarget  = self.video.nextLocationofPokmon  #The data structure is [x,y]
        


        """
        Judge if the distance is reachable, if yes, then go ahead. if no, then use the largest possible radius 
        """

        r_current = math.sqrt(self.mtft.finaltarget[0]**2 + self.mtft.finaltarget[1]**2)
        if (r_current >= REACHABLE_MAX_DISTANCE):
            self.mtft.finaltarget[0] = self.mtft.finaltarget[0] / r_current * REACHABLE_MAX_DISTANCE
            self.mtft.finaltarget[1] = self.mtft.finaltarget[1] / r_current * REACHABLE_MAX_DISTANCE


        self.mtft.state_MTFT_iSetCurrentLocationAsInitialLocation(self.rexarm);
        
        self.mtft.intermediatelocationnumber = 0

        #First way point.
        

        x_3 = self.mtft.finaltarget[0]
        y_3 = self.mtft.finaltarget[1]
        z_3 = 35;#35
        phi_3 = self.rexarm.rexarm_IK_CatchAnglePlaner([x_3,y_3,z_3])
        
        print("[Msg]: Catching Potion"),
        print([x_3,y_3,z_3,phi_3*R2D])

        x_2 = x_3
        y_2 = y_3
        z_2 = z_3 + 70
        phi_2 = phi_3


        r_target = math.sqrt( x_3**2 + y_3**2 );

        x_1 = x_3 *1.0 / r_target * 200;
        y_1 = y_3 *1.0 / r_target * 200;

        z_1 = 202

        phi_1 = PI/2

        self.mtft.intermediatelocation=[]

        self.mtft.state_MTFT_iAddIntermediateLocation([x_1,y_1,z_1,phi_1])
        self.mtft.state_MTFT_iAddIntermediateLocation([x_3,y_3,z_3,phi_3])


        #self.mtft.intermediatelocation.append([x_1,y_1,z_1,phi_1]);
        #self.mtft.intermediatelocationnumber = self.mtft.intermediatelocationnumber + 1

        #self.mtft.intermediatelocation.append([x_2,y_2,z_2,phi_2]);
        #self.mtft.intermediatelocationnumber = self.mtft.intermediatelocationnumber + 1

        #self.mtft.intermediatelocation.append([x_3,y_3,z_3,phi_3]);
        #self.mtft.intermediatelocationnumber = self.mtft.intermediatelocationnumber + 1

        


class State_MTFT_GoToNextWaypoint():#cmd, check if arrived
    def __init__(self, rexarm, mtft):
        self.rexarm = rexarm
        self.mtft = mtft
        self.configuration = []
        


    def iGoToNextWayPoint(self):####123
        isFinished = False

        if self.mtft.intermediatelocationcurrentnumber == self.mtft.intermediatelocationnumber:
            isFinished = True
            return isFinished

        

        #set joints
        #print("=================================================>"),
        #print(self.mtft.intermediatelocationcurrentnumber)
        #print(self.mtft.intermediatejointangle)

        self.rexarm.iSetJointAngle(0,self.mtft.intermediatejointangle[self.mtft.intermediatelocationcurrentnumber][0])
        self.rexarm.iSetJointAngle(1,self.mtft.intermediatejointangle[self.mtft.intermediatelocationcurrentnumber][1])
        self.rexarm.iSetJointAngle(2,self.mtft.intermediatejointangle[self.mtft.intermediatelocationcurrentnumber][2])
        self.rexarm.iSetJointAngle(3,self.mtft.intermediatejointangle[self.mtft.intermediatelocationcurrentnumber][3])

        
        #publish
        self.rexarm.cmd_publish()

        #FK
        #target = self.rexarm.rexarm_FK(self.mtft.intermediatejointangle[self.mtft.intermediatelocationcurrentnumber])

        target = self.mtft.intermediatelocation[self.mtft.intermediatelocationcurrentnumber]
        #check arrived?
        isArrived = False
        isArrived = self.iCheckIfArrived(target)
        

        if (self.mtft.intermediatelocationcurrentnumber == 0):
            self.rexarm.iSetSpeed(1.0)
            self.rexarm.iSetTorque(1.0)
        else:
            self.rexarm.iSetSpeed(0.4)
            self.rexarm.iSetTorque(0.7);



        if isArrived == True:
            #move to next point

            self.mtft.intermediatelocationcurrentnumber = self.mtft.intermediatelocationcurrentnumber + 1
            






            # arrived
            print("[Msg]: "),
            print(self.mtft.intermediatelocationcurrentnumber-1),
            print(" Way Point Arrived.")

            

        return isFinished


    def iCheckIfArrived(self, target):
        self.rexarm.rexarm_FK(self.rexarm.joint_angles_fb)
        realtimelocationgesture = [self.rexarm.P0[0],self.rexarm.P0[1],self.rexarm.P0[2],self.rexarm.T]
        errorX = abs(realtimelocationgesture[0] - target[0]);
        errorY = abs(realtimelocationgesture[1] - target[1]);
        errorZ = abs(realtimelocationgesture[2] - target[2]);
        errorT = abs(realtimelocationgesture[3] - target[3]);
        if (errorX < ERROR_LOCAL_TOL_X and errorY < ERROR_LOCAL_TOL_Y and errorZ < ERROR_LOCAL_TOL_Z and errorT < ERROR_LOCAL_TOL_T):
            return True
        else:
            return False

    def iCalculateInverseKinematics(self):
        return self.rexarm.rexarm_IK(self.mtft.intermediatelocation[self.mtft.intermediatelocationcurrentnumber], 1)
            

class State_MTFT_End():# change state
    def __init__(self, rexarm):
        pass





class State_CatchPokmon():
    def __init__(self,rexarm):
        self.rexarm = rexarm
        pass

    def iCloseGripper(self, rexarm):
        return self.rexarm.rexarm_gripper_grab(0);
























"""
#####################################################################################################################
                                         Class   M    T    B    Start
#####################################################################################################################
"""



class State_MoveToBall():

    """
    Innitialize the state State_MoveToFinalTarget
    """
    def __init__(self,rexarm, video):
       
        self.initialLocation = [0.0,0.0,0.0,0.0] #x,y,z,phi in [mm,mm,mm,rad]
        self.MTB_currentstate = STATE_CODE_MTB_CI


        self.intermediatelocation = [[0,0,0,0]]
        self.intermediatejointangle = [[0,0,0,0]]
        self.intermediatelocationnumber = 0
        self.intermediatelocationcurrentnumber = 0
        self.rexarm = rexarm
        self.video = video
        self.finaltarget = [0,0]


    def state_MTB_initialize(self):
        self.intermediatelocationnumber = 0
        self.intermediatelocationcurrentnumber = 0
        self.intermediatelocation = []
        self.intermediatejointangle = []
        self.finaltarget = []

    """
    Name: state_MTB_iCheckIfArrived
    Function: check if the current arm has arrived the final target.
    """
    def state_MTB_iSetCurrentLocationAsInitialLocation(self, rexarm):
        configuration = rexarm.joint_angles_fb;
        self.initialLocation =  rexarm.rexarm_FK(configuration)
        pass

    '''
    def state_MTB_iCheckIfArrivedFinalTarget(self):
        self.rexarm.rexarm_FK(self.rexarm.joint_angles_fb)
        realtimelocationgesture = [self.rexarm.P0[0],self.rexarm.P0[1],self.rexarm.P0[2],self.rexarm.T]
        errorX = abs(realtimelocationgesture[0] - self.finaltarget[0]);
        errorY = abs(realtimelocationgesture[1] - self.finaltarget[1]);
        errorZ = abs(realtimelocationgesture[2] - self.finaltarget[2]);
        errorT = abs(realtimelocationgesture[3] - self.finaltarget[3]);
        if (errorX < ERROR_LOCAL_TOL_X and errorY < ERROR_LOCAL_TOL_Y and errorZ < ERROR_LOCAL_TOL_Z and errorT < ERROR_LOCAL_TOL_T):
            return True
        else:
            return False
    '''

    """
    name: state_MTB_iAddIntermediateLocation
    Function: Add some intermediate location and orientation into the path, (probably can avoid some obstacles.)
    """
    def state_MTB_iAddIntermediateLocation(self,intermediatelocation):
        configuration =  self.rexarm.rexarm_IK(intermediatelocation, 1)

        #print("=================+++++>> configuation:"),
        #print(configuration)
        
        if (configuration[0] == 0):
            print("[ERROR]:Inreachable!!!=========================================") 
            self.mtb.MTB_currentstate = STATE_CODE_MTB_END
            exit(1)
        else:

            self.intermediatejointangle.append(configuration[1])
            self.intermediatelocation.append(intermediatelocation)
            self.intermediatelocationnumber = self.intermediatelocationnumber + 1

    """
    name: state_MTB_iMoveArmToFinalLocation(self).
    Function:Move the arm to the target position.
    """

    def state_MTB_iMoveArmToFinalLocation(self):
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

        self.rexarm.cmd_publish()
       







class State_MTB_CalculateIntermediate():#add points above pokemon and ball
    def __init__(self,rexarm, mtb, video):
        self.mtb = mtb
        self.rexarm = rexarm
        self.video = video



        pass
   


    """
    Name: calculate()
    Function: calculate the intermediate location of the path and change the following variables:

        self.intermediatelocation = [[0,0,0,0]]   : A list of 4 x 1 vectors, each vector mean the x,y,z,phi configuration of a certain way point
        self.intermediatelocationnumber = 0 : The number of vectors (i.e. Every location apart from the first one.)
        self.intermediatelocationcurrentnumber = 0: This is the current location number.

    """
    def calculate(self):

        self.mtb.finaltarget  = self.video.nextLocationofPokmon  #The data structure is [x,y]
       

        """
        Judge if the distance is reachable, if yes, then go ahead. if no, then use the largest possible radius 
        """

        r_current = math.sqrt(self.mtb.finaltarget[0]**2 + self.mtb.finaltarget[1]**2)

        if (r_current >= REACHABLE_MAX_DISTANCE):
            self.mtb.finaltarget[0] = self.mtb.finaltarget[0] / r_current * REACHABLE_MAX_DISTANCE
            self.mtb.finaltarget[1] = self.mtb.finaltarget[1] / r_current * REACHABLE_MAX_DISTANCE





        self.mtb.state_MTB_iSetCurrentLocationAsInitialLocation(self.rexarm);
       
        self.mtb.intermediatelocationnumber = 0

        #First way point.
        phi_3 = PI/2;

        x_1 = self.mtb.initialLocation[0]
        y_1 = self.mtb.initialLocation[1]
        z_1 = 140
        phi_1  = self.rexarm.rexarm_IK_CatchAnglePlaner([x_1,y_1,z_1])
        

        #r_init = math.sqrt( x_1**2 + y_1**2 );
        #x_2 = x_1 *1.0 / r_init * 100;
        #y_2 = y_1 *1.0 / r_init * 100;
        #z_2 = 220
        #phi_2 = PI/2

        #Decision on which half ball to go.
        
        """
        if y_1 >0:
            x_3 = -180
            y_3 = 1
            z_3 = 220
        else:
            x_3 = -180
            y_3 = -1
            z_3 = 220
        phi_3 = PI/2


        y_4 = y_3
        x_4 = -220
        z_4 = 45#130
        phi_4 = PI/2


        """
        
        if y_1 >0:
            x_3 = -250
            y_3 = 1
            z_3 = 128
        else:
            x_3 = -250
            y_3 = -1
            z_3 = 128
        phi_3 = 134*D2R


      
        
        y_4 = y_3
        x_4 = -220
        z_4 = 45#130
        phi_4 = PI/2
        

        self.mtb.intermediatelocation=[]

        #self.mtb.intermediatelocation.append([x_1,y_1,z_1,phi_1]);
        #self.mtb.intermediatelocationnumber = self.mtb.intermediatelocationnumber + 1





        r_1  = math.sqrt(x_1 **2 + y_1 **2)
        if (r_1 > SHRINK_RANGE):
            x_1 = x_1 *1.0 / r_1 * SHRINK_DISTANCE
            y_1 = y_1 *1.0 / r_1 * SHRINK_DISTANCE



        self.mtb.state_MTB_iAddIntermediateLocation([x_1,y_1,z_1,phi_1])








        self.mtb.state_MTB_iAddIntermediateLocation([x_3,y_3,z_3,phi_3])

#        self.mtb.intermediatelocation.append([x_2,y_2,z_2,phi_2]);
#        self.mtb.intermediatelocationnumber = self.mtb.intermediatelocationnumber + 1

        #self.mtb.intermediatelocation.append([x_3,y_3,z_3,phi_3]);
        #self.mtb.intermediatelocationnumber = self.mtb.intermediatelocationnumber + 1
        
#        self.mtb.intermediatelocation.append([x_4,y_4,z_4,phi_4]);
#        self.mtb.intermediatelocationnumber = self.mtb.intermediatelocationnumber + 1

       


class State_MTB_GoToNextWaypoint():#cmd, check if arrived

    def __init__(self, rexarm, mtb):
        self.rexarm = rexarm;
        self.mtb = mtb


    def iGoToNextWayPoint(self):
        isFinished = False

        if self.mtb.intermediatelocationcurrentnumber == self.mtb.intermediatelocationnumber:
            isFinished = True
            return isFinished

        

        #set joints
        self.rexarm.iSetJointAngle(0,self.mtb.intermediatejointangle[self.mtb.intermediatelocationcurrentnumber][0])
        self.rexarm.iSetJointAngle(1,self.mtb.intermediatejointangle[self.mtb.intermediatelocationcurrentnumber][1])
        self.rexarm.iSetJointAngle(2,self.mtb.intermediatejointangle[self.mtb.intermediatelocationcurrentnumber][2])
        self.rexarm.iSetJointAngle(3,self.mtb.intermediatejointangle[self.mtb.intermediatelocationcurrentnumber][3])

        
        #publish
        self.rexarm.cmd_publish()

        #FK
        #target = self.rexarm.rexarm_FK(self.mtft.intermediatejointangle[self.mtft.intermediatelocationcurrentnumber])

        target = self.mtb.intermediatelocation[self.mtb.intermediatelocationcurrentnumber]
        #check arrived?
        isArrived = False
        isArrived = self.iCheckIfArrived(target)
        
        if isArrived == True:
            #move to next point

            self.mtb.intermediatelocationcurrentnumber = self.mtb.intermediatelocationcurrentnumber + 1
            


            # arrived
            print("[Msg]: "),
            print(self.mtb.intermediatelocationcurrentnumber-1),
            print(" Way Point Arrived.")

            

        return isFinished


    def iCheckIfArrived(self, target):
        self.rexarm.rexarm_FK(self.rexarm.joint_angles_fb)
        realtimelocationgesture = [self.rexarm.P0[0],self.rexarm.P0[1],self.rexarm.P0[2],self.rexarm.T]
        errorX = abs(realtimelocationgesture[0] - target[0]);
        errorY = abs(realtimelocationgesture[1] - target[1]);
        errorZ = abs(realtimelocationgesture[2] - target[2]);
        errorT = abs(realtimelocationgesture[3] - target[3]);
        if (errorX < ERROR_LOCAL_TOL_X and errorY < ERROR_LOCAL_TOL_Y and errorZ < ERROR_LOCAL_TOL_Z and errorT < ERROR_LOCAL_TOL_T):
            return True
        else:
            return False

    def iCalculateInverseKinematics(self):
        return self.rexarm.rexarm_IK(self.mtb.intermediatelocation[self.mtb.intermediatelocationcurrentnumber], 1)
           


class State_MTB_End():# change state
    def __init__(self, rexarm):
        pass



"""
###########################################################
MTBFINISHED
###########################################################
"""














































class State_ReleashPokmon():
    def __init__(self,rexarm):
        self.rexarm = rexarm
        pass

    def iOpenGripper(self, rexarm):
        return self.rexarm.rexarm_gripper_grab(1);














class State_ResetArmPosition():
    def __init__(self, rexarm, video):
        self.video = video
        self.rexarm = rexarm

        pass


    def iResetArmPosition(self):
        self.rexarm.iResetPosition();
        return self.iCheckIfArrived([0,0,411,0])
        

    def iCheckIfArrived(self, target):
        self.rexarm.rexarm_FK(self.rexarm.joint_angles_fb)
        realtimelocationgesture = [self.rexarm.P0[0],self.rexarm.P0[1],self.rexarm.P0[2],self.rexarm.T]
        errorX = abs(realtimelocationgesture[0] - target[0]);
        errorY = abs(realtimelocationgesture[1] - target[1]);
        errorZ = abs(realtimelocationgesture[2] - target[2]);
        errorT = abs(realtimelocationgesture[3] - target[3]);
        if (errorX < ERROR_LOCAL_TOL_X and errorY < ERROR_LOCAL_TOL_Y and errorZ < ERROR_LOCAL_TOL_Z and errorT < ERROR_LOCAL_TOL_T):
            return True
        else:
            return False



    def iResetOneLoopVariables(self):
        self.video.numPokRemain = 0
        self.video.whetherFinishedCam = False
        self.video.nextLocationofPokmon = [0,0]



class State_END():
    def __init__(self, rexarm):
        pass








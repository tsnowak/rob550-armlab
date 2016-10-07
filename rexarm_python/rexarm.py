import lcm
import time
import numpy as np
import math

from lcmtypes import dynamixel_command_t
from lcmtypes import dynamixel_command_list_t
from lcmtypes import dynamixel_status_t
from lcmtypes import dynamixel_status_list_t

PI = np.pi
D2R = PI/180.0
R2D = 180.0/3.141592
ANGLE_TOL = 2*PI/180.0 


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

""" Rexarm Class """
class Rexarm():
    def __init__(self,ui):
        self.ui = ui;
        """ Commanded Values """
        self.num_joints = 6
        self.joint_angles = [0.0] * self.num_joints # radians
        # you must change this to control each joint speed separately 
        self.speed = 0.5                         # 0 to 1
        self.max_torque = 0.5                    # 0 to 1

        """ Feedback Values """
        self.joint_angles_fb = [0.0] * self.num_joints # radians
        self.speed_fb = [0.0] * self.num_joints        # 0 to 1   
        self.load_fb = [0.0] * self.num_joints         # -1 to 1  
        self.temp_fb = [0.0] * self.num_joints         # Celsius               

        """ Waypoint Plan - TO BE USED LATER """
        self.plan_status = 0

        #Only the way point.
        self.wpt = []
        self.wpt_number = 0
        self.wpt_total = 0
        
        #The entire way.
        self.way = []
        self.way_number = 0
        self.way_total = 0

        #Record the realtime feedback
        

        self.recfast = []
        self.recfast_number = 0  #no use, just to keep similar to way and wpt's style.
        self.recfast_total = 0

        self.recslow = []
        self.recslow_number = 0  #no use, just to keep similar to way and wpt's style.
        self.recslow_total = 0

        # variables for the cubic polynomial fit routine
        self.cubic_coeffs = [0.0] * 4      # list of np arrays that are 4x1
        self.st = 0                 # the start time of the cubic function 
        
        """ Setup LCM and subscribe """
        self.lc = lcm.LCM()
        lcmMotorSub = self.lc.subscribe("ARM_STATUS",
                                        self.feedback_handler)

        self.P0 = [0] * 3;
        self.T = 0
    def cmd_publish(self):
        """ 
        Publish the commands to the arm using LCM. 
        You need to activelly call this function to command the arm.
        You can uncomment the print statement to check commanded values.
        """    
        msg = dynamixel_command_list_t()
        msg.len = 6
        self.clamp()
        for i in range(msg.len):
            cmd = dynamixel_command_t()
            cmd.utime = int(time.time() * 1e6)
            cmd.position_radians = self.joint_angles[i]
            # you SHOULD change this to contorl each joint speed separately 
            cmd.speed = self.speed
            cmd.max_torque = self.max_torque
            #print cmd.position_radians
            msg.commands.append(cmd)
        self.lc.publish("ARM_COMMAND",msg.encode())
    
    def get_feedback(self):
        """
        LCM Handler function
        Called continuously from the GUI 
        """
        self.lc.handle_timeout(50)

    def feedback_handler(self, channel, data):
        """
        Feedback Handler for LCM
        """
        msg = dynamixel_status_list_t.decode(data)
        for i in range(msg.len):
            self.joint_angles_fb[i] = msg.statuses[i].position_radians 
            self.speed_fb[i] = msg.statuses[i].speed 
            self.load_fb[i] = msg.statuses[i].load 
            self.temp_fb[i] = msg.statuses[i].temperature

    def clamp(self):
        """
        Clamp Function
        Limit the commanded joint angles to ones physically possible so the 
        arm is not damaged.
        LAB TASK: IMPLEMENT A CLAMP FUNCTION
        """
        #B = {-179.99,180}
        #S = {125.8, -124.3}
        #E = {125.8, -123.12}
        #W = {128.9, -125.39}

        if self.joint_angles[0]*R2D < -179.99:
            self.joint_angles[0] = -179.99*D2R
        elif self.joint_angles[0]*R2D > 179.99:
            self.joint_angles[0] = 179.99*D2R

        if self.joint_angles[1]*R2D > 125.8:
            self.joint_angles[1] = 125.8*D2R
        elif self.joint_angles[1]*R2D < -124.3:
            self.joint_angles[1] = -124.3 *D2R

        if self.joint_angles[2]*R2D > 125.8:
            self.joint_angles[2] = 125.8*D2R
        elif self.joint_angles[2]*R2D < -123.12:
            self.joint_angles[2] = -123.12 *D2R

        if self.joint_angles[3]*R2D > 128.9:
            self.joint_angles[3] = 128.9*D2R
        elif self.joint_angles[3]*R2D < -125.39:
            self.joint_angles[3] = -125.39*D2R

        ## TODO: IMPLEMENT GRIP LIMITS ##

        #pass

    def plan_command(self):
        """ Command planned waypoints """
        pass

    def rexarm_FK(self,angles):

        """
        Note: the angles are the angles directly from the sensors, it should be in RAD
        """
        """
        Calculates forward kinematics for rexarm
        takes a DH table filled with DH parameters of the arm
        and the link to return the position for
        returns a 4-tuple (x, y, z, phi) representing the pose of the 
        desired link
        """
        


        c1 = math.cos(angles[0])
        s1 = math.sin(angles[0])
        c2 = math.cos(angles[1])
        s2 = math.sin(angles[1])
        c3 = math.cos(angles[2])
        s3 = math.sin(angles[2])
        c4 = math.cos(angles[3])
        s4 = math.sin(angles[3])

        R01 = [[c1,-s1,0,0],[s1,c1,0,0],[0,0,1,DH1_D],[0,0,0,1]]
        R12 = [[-s2,c2,0,0],[0,0,1,0],[c2,s2,0,0],[0,0,0,1]]
        R23 = [[c3,s3,0,DH3_A],[-s3,c3,0,0],[0,0,1,0],[0,0,0,1]]
        R34 = [[c4,s4,0.0,DH4_A],[-s4,c4,0,0],[0,0,1,0],[0.0,0.0,0.0,1.0]]
        P4 = [[100],[0],[0],[1]]

        P3 = np.dot(R34,P4)
        P2 = np.dot(R23,P3)
        P1 = np.dot(R12,P2)
        #he point with respect to the base. in the format[[],[],[],[]]
        Pbase = np.dot(R01,P1)
        self.P0[0] = Pbase[0][0]
        self.P0[1] = Pbase[1][0]
        self.P0[2] = Pbase[2][0]

        self.T =  (- 1 * (angles[1] + angles[2] + angles[3] ) - PI/2 ) * R2D;




    """
    The helper function for IK
    Function: private function for rexarm_IK, used to do the angle changing from configuration 1 2 to configuration 3 4.
    """
    def rexarm_IK_helper(self,value):
        while (not (value > -PI and value <= PI)):
            if (value <= -PI):
                value = value + 2 * PI
            if (value > PI):
                value  = value - 2 * PI
        return value
    """
    The helper function fo rrexarm_IK:
    Function: Help rexarm_IK to see if a point is reachable.
    """    
    def CosSinRangeCheck(self, value):
        if (value >= -1 and value <= 1):
            return True
        else:
            return False

    """
    Name: rexarm_IK
    Input: pose: 4 x 1 tuple: [x, y, z, phi], the target location and orientation.
            cfg: not used yet.
    Output: validity, configuration_1, configuration_2, configuration_3, configuration_4.
            where:
            1. Validity: Should if the given location is reachable. If not reachable, then return 0,0,0,0,0
            2. Configuration_1: The first configuration, Elbow is Up, and face to x-positive (Note: this x is defined by Zhentao, not global X)
            3. Configuration_2: The first configuration, Elbow is Down, and face to x-positive
            4. Configuration_3: The first configuration, Elbow is Up, and face to x-negative
            5. Configuration_4: The first configuration, Elbow is Down, and face to x-nagative.
            Note: each configuration is an 4 x 1 tuple, which represent the command for four servos.
    """

    def rexarm_IK(self, pose, cfg):
        """
        Calculates inverse kinematics for the rexarm
        pose is a tuple (x, y, z, phi) which describes the desired
        end effector position and orientation.  
        cfg describe elbow down (0) or elbow up (1) configuration
        returns a 4-tuple of joint angles or NONE if configuration is impossible
        """

        G_x = pose[0];
        G_y = pose[1];
        G_z = pose[2];
        G_phi=pose[3];

        if (not (G_y == 0 and G_x < 0)):
            configuration_base = math.atan2(G_y, G_x);
        else:
            print("ERROR: in rexarm_IK(), besure that not (y ==0 and x < 0), because it exceed the posible moving configuration.");
            #TODO: the above equation need to be refined, so that the singular condition is also reachable.

        #Convert to plane coordinate frame (x,z)
        G_x = math.sqrt(G_x **2 + G_y **2);
        Angle_ZWG = G_phi;

        #calculate the location of W point. (Wrist joint.)

       

        W_x = G_x  - L4 * math.sin(Angle_ZWG);
        W_z = G_z  - L4 * math.cos(Angle_ZWG);
        
        L_SW = math.sqrt(W_x ** 2 + (W_z - L1)**2);
      
        if (self.CosSinRangeCheck((L2**2 + L3**2 - L_SW**2) *1.0 / (2 * L2 * L3))):
            Angle_WES = math.acos((L2**2 + L3**2 - L_SW**2) *1.0 / (2 * L2 * L3))
        else:
            return 0,0,0,0,0
        
        if (self.CosSinRangeCheck((L_SW **2 + L2**2  - L3**2) / (2 * L2 * L_SW))):
            Angle_WSE = math.acos((L_SW **2 + L2**2  - L3**2) / (2 * L2 * L_SW))
        else:
            return 0,0,0,0,0

        Angle_SWE = PI - Angle_WES - Angle_WSE

       
        configuration_elbow_up = PI  - Angle_WES;
        configuration_elbow_up = - configuration_elbow_up;
        Angle_ZSW = math.atan2(W_x, W_z - L1)
        configuration_shoulder_up = Angle_ZSW - Angle_WSE
        configuration_shoulder_up =  - configuration_shoulder_up
        configuration_wrist_up = Angle_ZWG + configuration_shoulder_up + configuration_elbow_up;

        configuration_wrist_up = - configuration_wrist_up;

        configuration_elbow_down = PI  - Angle_WES;
        
        configuration_elbow_down = + configuration_elbow_down;
        Angle_ZSW = math.atan2(W_x, W_z - L1)
        configuration_shoulder_down = Angle_ZSW + Angle_WSE
        configuration_shoulder_down =  - configuration_shoulder_down


        configuration_wrist_down = Angle_ZWG + configuration_shoulder_down + configuration_elbow_down;
        configuration_wrist_down = - configuration_wrist_down;



        configuration_1 = [configuration_base, configuration_shoulder_up,configuration_elbow_up,configuration_wrist_up];
        configuration_2 = [configuration_base, configuration_shoulder_down,configuration_elbow_down,0];
        

        configuration_3 = [ self.rexarm_IK_helper(configuration_1[0] + PI) , -configuration_1[1],-configuration_1[2],-configuration_1[3]]
        configuration_4 = [ self.rexarm_IK_helper(configuration_2[0] + PI) , -configuration_2[1],-configuration_2[2],-configuration_2[3]]

        return 1,configuration_1, configuration_2, configuration_3, configuration_4


        """
        #Configuration of base servo is here.
        configuration_base = math.atan2(G_y, G_x) ;


#       Here change the coordinate system so that the x-z plane is paralle to the arm's plane.
        G_x = math.sqrt(G_x ** 2 + G_y ** 2);

        Angle_ZWG = G_phi


        W_x = G_x  - L4 * math.sin(Angle_ZWG);
        W_z = G_z  - L4 * math.cos(Angle_ZWG);
        
        

        L_SW = math.sqrt(W_x ** 2 + (W_z - L1)**2);

        
        print([L2,L3,L_SW, ((L2**2 + L3**2 - L_SW**2) *1.0 / (2 * L2 * L3))])
        Angle_WES = math.acos((L2**2 + L3**2 - L_SW**2) *1.0 / (2 * L2 * L3))
        Angle_WSE = math.acos((L_SW **2 + L2**2  - L3**2) / (2 * L2 * L_SW))
        Angle_SWE = PI - Angle_WES - Angle_WSE

        Angle_ZSW = math.atan2(W_x, W_z - L1)




        configuration_shoulder_1 = Angle_ZSW - Angle_WSE; #The one that you can get by rotation from z to x w.r.t. positive Y axis,
        configuration_shoulder_2 = Angle_ZSW + Angle_WSE;#The second one that ...

        configuration_shoulder_1 = -configuration_shoulder_1
        configuration_shoulder_2 = -configuration_shoulder_2



        print([Angle_WES, Angle_WES])        
        configuration_elbow_1 = PI - Angle_WES;
        configuration_elbow_2 = -PI + Angle_WES;

        configuration_elbow_1 =  - configuration_elbow_1 
        configuration_elbow_2 = -configuration_elbow_2


        Angle_ZEW_1 = configuration_shoulder_1 + configuration_elbow_1;
        Angle_ZEW_2 = configuration_shoulder_2 + configuration_elbow_2;

        configuration_wrist_1 = Angle_ZWG - Angle_ZEW_1;
        configuration_wrist_2 = Angle_ZWG - Angle_ZEW_2;

        configuration_wrist_1 = -configuration_wrist_1;
        configuration_wrist_2 = -configuration_wrist_2;

        configuration_1 = [configuration_base, configuration_shoulder_1, configuration_elbow_1, configuration_wrist_1]
        configuration_2 = [configuration_base, configuration_shoulder_2, configuration_elbow_2, configuration_wrist_2]

        """

        return configuration_1;
        
    def rexarm_collision_check(q):
        """
        Perform a collision check with the ground and the base
        takes a 4-tuple of joint angles q
        returns true if no collision occurs
        """
        pass 


    def iSetJointAngle(self, jointIndex, value):
        
        
        if (not (jointIndex == 0 or jointIndex == 1 or jointIndex == 2 or jointIndex == 3)):
            print("Error: in iSetJointAngle(self, jointIndex, value): jointIndex should be integer in {0,1,2,3}")
            pass
        elif (not (value <= PI and value >= - PI)):
            print("Error: in iSetJointAngle(self, jointIndex, value): value should be from - PI to PI")
            pass
        else:
            self.joint_angles[jointIndex] = value
            if (jointIndex == 0):
                self.ui.sldrBase.setProperty("value",0)
            elif (jointIndex == 1):
                self.ui.sldrShoulder.setProperty("value",0)
            elif (jointIndex == 2):
                self.ui.sldrElbow.setProperty("value",0)
            elif (jointIndex == 3):
                self.ui.sldrWrist.setProperty("value",0)
            else:
                print("iSetJointAngle(self,joitnIndex,value): Unexpected jointIndex value.")
                pass
    """
    Reset Torque and Speed
    """
    def iSetTorque(self, value):
        if (not(value >= 0 and value <= 1)):
            print("ERROR: In iSetTorque(self, value): value should be in range [0,1]");
            pass
        else:
            self.max_torque = value;
            self.ui.rdoutTorq.setText(str(100 * value) + "%")
            self.ui.sldrMaxTorque.setProperty("value",value*100)        
    """
    Set the speed to desired value.
    """ 
    def iSetSpeed(self, value):
        if (not(value >= 0 and value <= 1)):
            print("ERROR: In iSetSpeed(self, value): value should be in range [0,1]");
            pass
        else:
            self.speed = value;
            self.ui.rdoutSpeed.setText(str(100 * value) + "%")
            self.ui.sldrSpeed.setProperty("value",value*100)        

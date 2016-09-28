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




""" Rexarm Class """
class Rexarm():
    def __init__(self):

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





    def rexarm_IK(pose, cfg):
        """
        Calculates inverse kinematics for the rexarm
        pose is a tuple (x, y, z, phi) which describes the desired
        end effector position and orientation.  
        cfg describe elbow down (0) or elbow up (1) configuration
        returns a 4-tuple of joint angles or NONE if configuration is impossible
        """
        pass
        
    def rexarm_collision_check(q):
        """
        Perform a collision check with the ground and the base
        takes a 4-tuple of joint angles q
        returns true if no collision occurs
        """
        pass 

import cv2
import numpy as np
#from collections import namedtuple
from PyQt4 import QtGui, QtCore, Qt

class Video():
    def __init__(self,capture):
        self.capture = capture
        self.capture.set(3, 1280)
        self.capture.set(4, 960)
        self.currentFrame=np.array([])
        
        """
        Blob detection variables
        """
        # lb, db, o, y, g
        self.lower_thresh = [np.array([88,63,95]),np.array([105,111,43]),np.array([5,157,166]),np.array([17,165,159]),np.array([48,149,100])]
        self.upper_thresh = [np.array([104,178,209]),np.array([113,181,92]),np.array([7,237,255]),np.array([30,218,255]),np.array([56,174,187])]
        
        # Pokemon characteristics (should be in a struct, but it's python)
        #switch to 1 if the pokemon is being targeted
        self.poke_color = []
        # note: initialized locations as x,y tuples. MIGHT NEED TO DO LIST OF LISTS
        self.location = []
        # initialize the pokemon identity to 0
        self.identity = 0
        # initialize distance to an unreasonably large value
        self.distance = 10000
        
        # four coordinates of corners of pokemon table
        # Top R;Bot R;Bot L;Top L
        #self.w_boundary_mask = [(-300,300),(300,300),(300,-300),(-300,-300)]
        # -x,y ; x,y ; x,-y;-x,-y
        self.p_boundary_mask = [(0,0),(0,0)]
        self.w_boundary_mask = [(300,300),(-300,-300)]
        
        # mask coordinates to block out the arm
        #self.w_arm_mask = [(-300,35),(10,35),(10,-35),(-300,-35)]
        self.p_arm_mask = [(0,0),(0,0)]
        self.w_arm_mask = [(10,35),(-300,-35)]
        
        # find the lower and upper x and y values for cropping the image        
        self.b_lower_xbound = 0
        self.b_lower_ybound = 0
        self.b_upper_xbound = 0
        self.b_upper_ybound = 0
        
        # find the lower and upper x and y values for masking the arm
        self.a_lower_xbound = 0
        self.a_lower_ybound = 0
        self.a_upper_xbound = 0
        self.a_upper_ybound = 0
        
        # declare kernels of various sizes for filtering
        self.r2_kernel = np.ones((2,2),np.uint8)
        self.r3_kernel = np.ones((3,3),np.uint8)
        self.r4_kernel = np.ones((4,4),np.uint8)

        # the sizeo f the qt display
        self.QT_frame_size = (640,480)
        self.scaling_factor = (0,0)
        self.boundary_mask = np.zeros((1280,960,3), np.uint8)

        """
        Statemachine trigger.

        """
        self.numPokRemain  = 0
        self.whetherFinishedCam = 0
        self.nextLocationofPokmon = [0,0]

        """ 
        Affine Calibration Variables 
        Note that OpenCV requires float32 arrays
        """
        self.aff_npoints = 6
        # center, pos. y, pos. x., neg. y, neg. x, 15cm 15cm (bot right)
        # labeled on table, 1-6
        self.real_coord = np.float32([[0., 0.], [0.,280.], [280.,0.],[0.,-280.],[-280.,0.],[150.,150.]])
        self.mouse_coord = np.float32([[0.0, 0.0],[0.0, 0.0],[0.0, 0.0],[0.0, 0.0],[0.0, 0.0],[0.0, 0.0]])      
        self.mouse_click_id = 0
        # 0 if unrun, 1 if running, 2 if completed
        self.aff_flag = 0
        self.aff_matrix = np.float32((2,3))

 
    def captureNextFrame(self):
        """                      
        Capture frame, convert to RGB, and return opencv image      
        """

        ret, frame=self.capture.read()
        if(ret==True):
            self.currentFrame=cv2.cvtColor(frame, cv2.COLOR_BAYER_GR2RGB)
        
    def convertFrame(self):
        """ Converts frame to format suitable for QtGui  """
        try:
            height,width=self.currentFrame.shape[:2]
            img=QtGui.QImage(self.currentFrame,
                              width,
                              height,
                              #QtGui.QImage.Format_RGB888)
                              QtGui.QImage.Format_RGB888)
            img=QtGui.QPixmap.fromImage(img)
            self.previousFrame = self.currentFrame
            return img
        except:
            return None

    def loadCalibration(self):
        """
        Load csmera distortion calibration from file and applies to the image:
        Look at camera_cal.py final lines to see how that is done
        This is optional, you do not need to implement distortion correction
        """
        pass

    # calculate mask coordinates in pixel frame for the boundary mask
    def calculateBoundaryMask(self):
        if self.aff_flag == 2:
            A = self.aff_matrix
            A_T = np.transpose(A)
            b = self.w_boundary_mask 
            #B = np.array([[b[0][0],b[1][0]],[b[0][1],b[1][1]],[1,1]])
            for i in range (0,len(b)):
                # x = (A^T*A)^(-1)*A^T*b
                # A is self.aff_matrix
                # B is index i of w_boundary_matrix in np.array format
                B = np.array([[b[i][0]],[b[i][1]],[1]])
                p_coords = np.dot(np.dot((np.linalg.inv(np.dot(A_T,A))), A_T), B)
                self.p_boundary_mask[i] = (p_coords[0],p_coords[1]) 

            # find the lower and upper x and y values for cropping the image        
            self.b_lower_xbound = int(min(self.p_boundary_mask[0][0],self.p_boundary_mask[1][0])) 
            self.b_lower_ybound = int(min(self.p_boundary_mask[0][1],self.p_boundary_mask[1][1]))
            self.b_upper_xbound = int(max(self.p_boundary_mask[0][0],self.p_boundary_mask[1][0]))
            self.b_upper_ybound = int(max(self.p_boundary_mask[0][1],self.p_boundary_mask[1][1])) 
            
            #print "Boundary Mask Values"
            #print "Lower X: ",
            #print self.b_lower_xbound,
            #print "\tLower Y: ",
            #print self.b_lower_ybound,
            #print "\tUpper X: ",
            #print self.b_upper_xbound,
            #print "\tUpper Y: ",
            #print self.b_upper_ybound

        pass

    # calculate mask coordinates in pixel frame for the arm mask
    def calculateArmMask(self):
        if self.aff_flag == 2:
            A = self.aff_matrix
            A_T = np.transpose(A)
            b = self.w_arm_mask 
            #B = np.array([[b[0][0],b[1][0]],[b[0][1],b[1][1]],[1,1]])
            for i in range (0,len(b)):
                # x = (A^T*A)^(-1)*A^T*b
                # A is self.aff_matrix
                # B is index i of w_boundary_matrix in np.array format
                B = np.array([[b[i][0]],[b[i][1]],[1]])
                p_coords = np.dot(np.dot((np.linalg.inv(np.dot(A_T,A))), A_T), B)
                self.p_arm_mask[i] = (p_coords[0],p_coords[1])  
            
            # find the lower and upper x and y values for masking the arm
            self.a_lower_xbound = int(min(self.p_arm_mask[0][0],self.p_arm_mask[1][0]))   
            self.a_lower_ybound = int(min(self.p_arm_mask[0][1],self.p_arm_mask[1][1]))
            self.a_upper_xbound = int(max(self.p_arm_mask[0][0],self.p_arm_mask[1][0]))
            self.a_upper_ybound = int(max(self.p_arm_mask[0][1],self.p_arm_mask[1][1])) 

            #print "Arm Mask Values"
            #print "Lower X: ",
            #print self.a_lower_xbound,
            #print "\tLower Y: ",
            #print self.a_lower_ybound,
            #print "\tUpper X: ",
            #print self.a_upper_xbound,
            #print "\tUpper Y: ",
            #print self.a_upper_ybound           

        pass

    # NOT USED, scaling factor is an easy 2
    def calculateFrameScaling(self, height, width):
        if self.aff_flag == 2:
            print "OpenCV Height: ",
            print height,
            print "\tOpenCV Width: ",
            print width
            self.scaling_factor = ((height/self.QT_frame_size[0]),(width/self.QT_frame_size[1]))
            print self.scalng_factor
        pass

    # create a boundary mask to block out pokemon outside of the base
    def generateBoundaryMask(self):
        if self.aff_flag == 2:
            img = np.zeros((960,1280,3), np.uint8)
            cv2.rectangle(img, (self.b_lower_xbound*2,self.b_lower_ybound*2), (self.b_upper_xbound*2,self.b_upper_ybound*2),(255,255,255),-1)
            mask = cv2.inRange(img, (255,255,255), (255,255,255))
            cv2.imwrite("testy.jpg", mask)
            self.boundary_mask = mask
        pass

    def blobDetector(self):
        """
        Implement your color blob detector here.  
        You will need to detect 5 different color blobs
        """

        # don't try to do anything unless the affine transform has been performed
        if self.aff_flag == 2:
                
            # clear lists
            self.location = []
            self.poke_color = []
            self.identity = 0
            self.distance = 10000
            
            # TESTING
            #cv2.namedWindow('TEST WINDOW', cv2.WINDOW_AUTOSIZE)    
            #cv2.waitKey(10)
            
            print "[Msg] Pokemon Color Scheme: 0 = lb, 1 = db, 2 = o, 3 = y, 4 = g"

            # loop over # of pokemon thresholds
            for i in range(0,len(self.lower_thresh)):
                print "[Msg] Acquiring pokemon colour ",
                print i
                # for at most 5 frames
                #for j in range(0,5):
                    
                # Let's try just looking at one frame
                # if the pokemon was acquired in the last frame, exit the loop
                self.captureNextFrame()
                #change current frame of color BAYER_GB2BGR to
                hsv = cv2.cvtColor(self.currentFrame, cv2.COLOR_RGB2HSV)

                # draw filled black rectangle over arm mask region
                cv2.rectangle(hsv, (self.a_lower_xbound*2, self.a_lower_ybound*2), (self.a_upper_xbound*2, self.a_upper_ybound*2), (0,0,0), -1)
    
                hsv = cv2.bitwise_and(hsv, hsv, mask = self.boundary_mask)
                #hsv = cv2.multiply(hsv, self.boundary_mask)

                # -x,y ; x,y ; x,-y;-x,-y
                # crop the hsv image removing that which is outside the board area
                #hsv = hsv[self.b_lower_xbound:self.b_upper_xbound, self.b_lower_ybound:self.b_upper_ybound]
                # TESTING
                #cv2.imshow('TEST WINDOW', hsv)
                
                # apply various filters to binary mask
                mask = cv2.inRange(hsv, self.lower_thresh[i], self.upper_thresh[i])
                open_1 = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.r2_kernel)
                close_1 = cv2.morphologyEx(open_1, cv2.MORPH_CLOSE, self.r3_kernel)
                open_2 = cv2.morphologyEx(close_1, cv2.MORPH_OPEN, self.r4_kernel)
                close_2 = cv2.morphologyEx(open_2, cv2.MORPH_CLOSE, self.r4_kernel)
                #thresh = close_2.copy()
                
                #cpy = close_2.copy()
                _, contours, hierarchy = cv2.findContours(close_2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                # run through each contour detected and assume the contour is 
                # the pokemon if it is of radius 17 > x > 28 -> ammended to 14>x>30
                # WHY ARE THE MOTHERFUCKING COORDINATES A FACTOR OF 2
                for cnt in contours:
                    (x,y),radius = cv2.minEnclosingCircle(cnt)
                    center = (int(x/2),int(y/2))
                    radius = int(radius)
                    #cv2.drawContours(cpy, contours, -1, (255,255,255), 3)
                    #cv2.circle(cpy,center,radius,(255,255,255),2)
                    if radius > 12 and radius < 38:
                        tmp_center = np.dot(self.aff_matrix, np.array([[center[0]],[center[1]],[1]]))
                        center = (int(tmp_center[0]),int(tmp_center[1]))
                        self.location.append(center)
                        self.poke_color.append(i)
                        tmp_distance = ((center[0]**2)+(center[1]**2))**(.5)
                        if tmp_distance < self.distance:
                            self.distance = tmp_distance
                            #print self.distance
                            self.identity = (len(self.location))
                        print "[Msg] Pokemon Found!!!",
                        print "\t Number: ",
                        print len(self.location),   
                        print "\t Colour: ",
                        print self.poke_color[-1]
                        print "\t Distance: ",
                        print tmp_distance
                        print "\t Center: ",
                        print center

        if len(self.location) != 0:
            self.numPokRemain = len(self.location)
            self.nextLocationofPokmon[0] = self.location[(self.identity-1)][0]
            self.nextLocationofPokmon[1] = self.location[(self.identity-1)][1]
            self.whetherFinishedCam = 1
            print "Pokemon to Pursue: ",
            print self.identity,
            print "\tPokemon Colour: ",
            print self.poke_color[self.identity-1],
            print "\tPokemon Location: ",
            print self.location[self.identity-1],
            print "\tPokemon Distance from Center: ",
            print self.distance
        else:
            self.numPokRemain = 0
            self.nextLocationofPokmon = [0,0]
            self.whetherFinishedCam = 1

    def affineTransform(self):
        """
        To compute the affine transform in our simplified space with constant z
        we simplify 
        Ax = b to 
        x = A-1b
        But because A is not necessarily square we use the pseudo-inverse
        x = (A^T*A)^(-1)*A^T*b

        To form our matrices A and B from the mouse_coord and real_coord lists
        for N correspondences we iteratively append to np.arrays based on N
        mouse_click_id's.
        """
        A = np.empty((0,6), np.float32)
        B = np.empty((0,1), np.float32)

        for x in range(0, self.mouse_click_id):
            A_tmp = np.array([[self.mouse_coord[x][0], self.mouse_coord[x][1], 1, 0, 0, 0],
                 [0, 0, 0, self.mouse_coord[x][0], self.mouse_coord[x][1], 1]])
            A = np.vstack((A, A_tmp))

            B_tmp = np.array([[self.real_coord[x][0]], [self.real_coord[x][1]]])
            B = np.concatenate((B, B_tmp), axis=0)

        #print "Matrix A\n"
        #print A

        #print "Matrix B\n"
        #print B
 
        A_T = np.transpose(A)

        C = np.dot(np.dot((np.linalg.inv(np.dot(A_T,A))), A_T), B) 
    
        """
        Here we simply verify that the affine transform was correctly found
        """
        #if np.allclose(np.dot(A,C), B):
        #    print 'Successfully found affine transform!'
        #else: 
        #    print 'Failed to find affine transform!'
        #    return

        C_resize = np.reshape(C, (2, 3))
        C_resize = np.append(C_resize, [[0,0,1]], axis=0)

        #print "Coefficient matrix for the affine transform.\n"
        #print C_resize

        return C_resize
        
        pass

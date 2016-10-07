import cv2
import numpy as np
from PyQt4 import QtGui, QtCore, Qt

class Video():
    def __init__(self,capture):
        self.capture = capture
        self.capture.set(3, 1280)
        self.capture.set(4, 960)
        self.currentFrame=np.array([])
        """
        Statemachine trigger.

        """
        self.numPokRemain  = 0
        self.whetherFinishedCam = False;
        self.nextLocationofPokmon = [0,0];


        """ 
        Affine Calibration Variables 
        Currently only takes three points: center of arm and two adjacent, 
        corners of the base board. Use more for better calibration.
        Note that OpenCV requires float32 arrays
        """
        self.aff_npoints = 6
	# center, pos. y, pos. x., neg. y, neg. x, 15cm 15cm (bot right)
	# labeled on table, 1-6
        self.real_coord = np.float32([[0., 0.], [0.,280.], [280.,0.],[0.,-280.],[-280.,0.],[150.,150.]])
        self.mouse_coord = np.float32([[0.0, 0.0],[0.0, 0.0],[0.0, 0.0],[0.0, 0.0],[0.0, 0.0],[0.0, 0.0]])      
        self.mouse_click_id = 0
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

    def blobDetector(self):
        """
        Implement your color blob detector here.  
        You will need to detect 5 different color blobs
        """
        r2_kernel = np.ones((2,2),np.uint8)
        r3_kernel = np.ones((3,3),np.uint8)
        r4_kernel = np.ones((4,4),np.uint8)

	# lb, db, o, y, g
	lower_thresh = [np.array([88,63,95]),np.array([105,111,43]),np.array([5,157,166]),np.array([17,165,159]),np.array([48,149,100])]
	upper_thresh = [np.array([104,178,209]),np.array([113,181,92]),np.array([7,237,255]),np.array([30,218,255]),np.array([56,174,187])]
	#switch to 1 as each pokemon location is acquired
	acquired = [0,0,0,0,0]
	# note: initialized locations as x,y tuples. MIGHT NEED TO DO LIST OF LISTS
	location = [(0,0),(0,0),(0,0),(0,0),(0,0)]
	
	# loop over pokemon
	for i in range(0,5):
		# while we haven't acquired a pokemons location
		print "Acquiring location ",
		print i+1
		while acquired[i] != 1:
			self.captureNextFrame()
			#change current frame of color BAYER_GB2BGR to
			hsv = cv2.cvtColor(self.currentFrame, cv2.COLOR_RGB2HSV)

			# apply various filters to binary mask
                        mask = cv2.inRange(hsv, lower_thresh[i], upper_thresh[i])
                        open_1 = cv2.morphologyEx(mask, cv2.MORPH_OPEN, r2_kernel)
                        close_1 = cv2.morphologyEx(open_1, cv2.MORPH_CLOSE, r3_kernel)
                        open_2 = cv2.morphologyEx(close_1, cv2.MORPH_OPEN, r4_kernel)
                        close_2 = cv2.morphologyEx(open_2, cv2.MORPH_CLOSE, r4_kernel)
                        thresh = close_2.copy()
                        #cpy = close_2.copy()
                        _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                        # run through each contour detected and assume the contour is 
			# the pokemon if it is of radius 17 > x > 28
                        for cnt in contours:
                                (x,y),radius = cv2.minEnclosingCircle(cnt)
                                center = (int(x),int(y))
                                radius = int(radius)
                                #cv2.drawContours(cpy, contours, -1, (255,255,255), 3)
                                #cv2.circle(cpy,center,radius,(255,255,255),2)
				if radius > 17 and radius < 28:
					location[i] = center
					acquired[i] = 1
					print "Pokemon: ",
					print i,
                                	print "\tRadius: ",
                                	print radius,
                                	print "\tCenter: ",
                                	print center

	print "Coordinates of pokemon in pixel frame"
	print location


    """
    Begin Ted's affine transform
    """
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

        #print "Coefficient matrix for the affine transform.\n"
        #print C_resize

        return C_resize
        
        pass

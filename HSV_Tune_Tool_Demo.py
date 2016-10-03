
"""
Zhentao Xu
HSV Tuning Tool
Function: You can click the region of desired color, and the terminal can show the hsv range.
		   During selection, you can click Button "u" to undo the selection.
"""

import cv2
import numpy as np


ColorRange =[]
ColorRange.append([[17,179,184],[30,219,255]])
ColorRange.append([[4,158,162],[7,243,255]])
ColorRange.append([[47,144,78],[59,182,185]])
ColorRange.append([[88,128,109],[101,204,210]])
ColorRange.append([[104,99,36],[113,177,86]])


currentColor = 0;

def mouse_callback(event, x,y,flags, param):
	global ColorRange
	global currentColor


	


if __name__ == "__main__":


	cap = cv2.VideoCapture(0)
	cap.set(3,800)
	cap.set(4,600)

	lower_blue = np.array(ColorRange[0][0])
	upper_blue = np.array(ColorRange[0][1])

	


	cv2.namedWindow('Window',cv2.WINDOW_AUTOSIZE)
	cv2.setMouseCallback('Window',mouse_callback)

	cv2.namedWindow('MASK INRANGE',cv2.WINDOW_AUTOSIZE)
	cv2.setMouseCallback('MASK INRANGE',mouse_callback)

	while True:
		ret, frame = cap.read()
		if ret:
			"""
			Already got one frame.
			"""
			bgr = cv2.cvtColor(frame, cv2.COLOR_BAYER_GR2BGR)
			rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
			hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

			cv2.imshow('Window', bgr)
			
            # 0x1B = ESC	
			ch = cv2.waitKey(10)
			if ch == 0x1B:
				break
            
            # 0x75 = u
			if ch == 0x75:
				if currentColor <= 3:
					currentColor   = currentColor + 1
				else:
					currentColor = 0
			
			print(currentColor)

			
			lower_blue= np.array(ColorRange[currentColor][0])
			upper_blue= np.array(ColorRange[currentColor][1])
					


			"""
			Do the thresholding.
			"""
			
			
			
			mask = cv2.inRange(hsv, lower_blue, upper_blue)
	    	cv2.imshow('MASK INRANGE',mask)		

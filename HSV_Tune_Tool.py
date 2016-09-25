
"""
Zhentao Xu
HSV Tuning Tool
Function: You can click the region of desired color, and the terminal can show the hsv range.
"""

import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(3,800)
cap.set(4,600)





lower_blue = np.array([180,255,255])
upper_blue = np.array([0,0,0])

hsv_history = []

CurrentIndex = 0;

def push_hsv_value(hsv,x,y):
	h = hsv[y][x][0]
	s = hsv[y][x][1]
	v = hsv[y][x][2]
	hsv_history.append([h,s,v])
	#print(hsv_history)

def recalculate_hsv_lower():
	lower_blue[0] =  min(a for (a,b,c) in hsv_history)
	lower_blue[1] =  min(b for (a,b,c) in hsv_history)
	lower_blue[2] =  min(c for (a,b,c) in hsv_history)
	return lower_blue



def recalculate_hsv_upper():
	upper_blue[0] =  max(a for (a,b,c) in hsv_history)
	upper_blue[1] =  max(b for (a,b,c) in hsv_history)
	upper_blue[2] =  max(c for (a,b,c) in hsv_history)
	return upper_blue

def mouse_callback(event, x,y,flags, param):
	if event == cv2.EVENT_LBUTTONDOWN:
		push_hsv_value(hsv,x,y)
		lower_blue= recalculate_hsv_lower()
		upper_blue= recalculate_hsv_upper()

		print("Click Histary:"),
		print(len(hsv_history)),
		print("Current Range:"),
		print(lower_blue),
		print(upper_blue)




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

		cv2.imshow('Window', frame)	
		ch = cv2.waitKey(10)
		if ch == 0x1B:
			break

		if ch == 0x75:
			if len(hsv_history) > 1:
				del hsv_history[-1]
				lower_blue= recalculate_hsv_lower()
				upper_blue= recalculate_hsv_upper()
				print("Click Histary:"),
				print(len(hsv_history)),
				print("Current Range:"),
				print(lower_blue),
				print(upper_blue)



		"""
		Do the thresholding.
		"""
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, lower_blue, upper_blue)
    	cv2.imshow('MASK INRANGE',mask)
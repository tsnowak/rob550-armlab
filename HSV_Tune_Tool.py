
"""
Zhentao Xu
HSV Tuning Tool
Function: You can click the region of desired color, and the terminal can show the hsv range.
		   During selection, you can click Button "u" to undo the selection.
"""

import cv2
import numpy as np





def push_hsv_value(hsv,x,y):
	h = hsv[y][x][0]
	s = hsv[y][x][1]
	v = hsv[y][x][2]
	hsv_history.append([h,s,v])
	#print(hsv_history)

def recalculate_hsv_lower():
	#lower_blue[0] =  min(a for (a,b,c) in hsv_history)
	#lower_blue[1] =  min(b for (a,b,c) in hsv_history)
	#lower_blue[2] =  min(c for (a,b,c) in hsv_history)
	
	# Light Blue
	lower_blue[0] = 88
	lower_blue[1] = 63
	lower_blue[2] = 95

	# Dark Blue
	#lower_blue[0] = 105 
	#lower_blue[1] = 111 	
	#lower_blue[2] = 43

	# Orange
	#lower_blue[0] = 5
	#lower_blue[1] = 157	
	#lower_blue[2] = 166

	# Yellow
	#lower_blue[0] = 17 
	#lower_blue[1] = 165	
	#lower_blue[2] = 159

	#Green
	#lower_blue[0] = 48
	#lower_blue[1] = 149	
	#lower_blue[2] = 100

	return lower_blue


def recalculate_hsv_upper():
	#upper_blue[0] =  max(a for (a,b,c) in hsv_history)
	#upper_blue[1] =  max(b for (a,b,c) in hsv_history)
	#upper_blue[2] =  max(c for (a,b,c) in hsv_history)
	
	# Light Blue
	upper_blue[0] = 104
	upper_blue[1] = 178
	upper_blue[2] = 209

	# Dark Blue
	#upper_blue[0] = 113
	#upper_blue[1] = 181	
	#upper_blue[2] = 92 

	# Orange
	#upper_blue[0] = 7
	#upper_blue[1] = 237	
	#upper_blue[2] = 255

	# Yellow
	#upper_blue[0] = 30
	#upper_blue[1] = 218	
	#upper_blue[2] = 255

	#Green
	#upper_blue[0] = 56
	#upper_blue[1] = 174	
	#upper_blue[2] = 187

	return upper_blue

def mouse_callback(event, x,y,flags, param):
	if event == cv2.EVENT_LBUTTONDOWN:
		push_hsv_value(hsv,x,y)
		lower_blue= recalculate_hsv_lower()
		upper_blue= recalculate_hsv_upper()

		print("Click History:"),
		print(len(hsv_history)),
		print("Current Range:"),
		print(lower_blue),
		print(upper_blue)



if __name__ == "__main__":


	cap = cv2.VideoCapture(0)
	cap.set(3,1280)
	cap.set(4,960)

    # switch between opening filter output and simple B/W mask output
	filter_switch = False
	# switch between showing minimum closing circle or not
	circle_switch = False
    # convolutional filter kernel 
	r2_kernel = np.ones((2,2),np.uint8)
	r3_kernel = np.ones((3,3),np.uint8)
	r4_kernel = np.ones((4,4),np.uint8)

	lower_blue = np.array([180,255,255])
	upper_blue = np.array([0,0,0])

	hsv_history = []

	CurrentIndex = 0;

    # instantiate windows
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

			cv2.imshow('Window', hsv)
				
			ch = cv2.waitKey(10)
            # 0x1B = ESC
			if ch == 0x1B:
				break
            
            # 0x75 = u
            # undo the last point
			#if ch == 0x75:
			if ch == 1048693:
				if len(hsv_history) > 1:
					del hsv_history[-1]
					lower_blue= recalculate_hsv_lower()
					upper_blue= recalculate_hsv_upper()
					print("Click Histary:"),
					print(len(hsv_history)),
					print("Current Range:"),
					print(lower_blue),
					print(upper_blue)

			# apply various filters to binary mask
			mask = cv2.inRange(hsv, lower_blue, upper_blue)
			open_1 = cv2.morphologyEx(mask, cv2.MORPH_OPEN, r2_kernel)
			close_1 = cv2.morphologyEx(open_1, cv2.MORPH_CLOSE, r3_kernel)
			open_2 = cv2.morphologyEx(close_1, cv2.MORPH_OPEN, r4_kernel)
			close_2 = cv2.morphologyEx(open_2, cv2.MORPH_CLOSE, r4_kernel)
			#closed = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, r4_kernel)
			thresh = close_2.copy()
			cpy = close_2.copy()
			_, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

			#if ch == 1048675:
			for cnt in contours:	
				(x,y),radius = cv2.minEnclosingCircle(cnt)
				center = (int(x),int(y))
				radius = int(radius)
				cv2.drawContours(cpy, contours, -1, (255,255,255), 3)
				cv2.circle(cpy,center,radius,(255,255,255),2)
				print "Radius: ",
				print radius,
				print "\tCenter: ",
				print center
			#circle_switch = True
			#print "Circle Status: ",
			#print circle_switch

			#if circle_switch:
			#else:

			# press f to apply filters
			if ch == 1048678:
				filter_switch = not filter_switch
				print "Filter Status: ",
				print filter_switch

            # show filtered mask
			if filter_switch:
				cv2.imshow('MASK INRANGE',close_2)		
            # show unfiltered mask
			else:	
				cv2.imshow('MASK INRANGE',cpy)		

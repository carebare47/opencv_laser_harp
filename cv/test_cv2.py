#!/usr/bin/python

import cv2
import time
import math
import numpy as np

THRESHOLD = 110
w=640
h=480
oldstate = None



grey2 = np.zeros((h,w,1), np.uint8)

def getpoints(image):
	points = []
	global grey2
        grey = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        thresh = cv2.inRange(grey, THRESHOLD, 255)
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        grey2 = thresh.copy()
        for contour in contours:
		bound_rect = cv2.boundingRect(contour)
	        
		pt = (bound_rect[0] + bound_rect[2]/2, bound_rect[1] + bound_rect[3]/2)
		#cv.Circle(image, pt, 2, cv.CV_RGB(255,0,0), 1)
		points.append(pt)
	return points


def near(p1, p2, d):
	x,y = (p1[0]-p2[0],p1[1]-p2[1])
	dist = math.sqrt(x**2+y**2)
	return dist < d

v_off = -0.1

n_start = 44 + 1 + 6
n_end = 44 + 24 + 6
n_off = 1

offsets = [0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0]
xpos = [0, 0.5, 1, 1.5, 2, 3, 3.5, 4, 4.5, 5, 5.5, 6]

def gxpos(x):
	o = x // 12
	v = x % 12
	return xpos[v] + o * 7

dots = []

xstart = gxpos(n_start + n_off)
xend = gxpos(n_end + n_off)


webcam = cv2.VideoCapture(-1)


#check, frame = webcam.read()
#print(check) #prints true as long as the webcam is running
#print(frame) #prints matrix values of each framecd 

#grey = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
#thresh = cv2.inRange(grey, THRESHOLD, 255)
#contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

#img = cv2.drawContours(thresh, contours, -1, (0,255,0), 3)
#getpoints(frame)
#cv2.imshow("image", img)
#cv2.imshow("Capturing", hsv)
#cv2.waitKey()
#time.sleep(0.1)

def note(n, state=False):
	print "on: ", state, "note: ", n




try:
	for i in range(10):
		check, image = webcam.read()
	refpoints = getpoints(image)
	refpoints.sort(key=lambda x: x[0])
	
	print(len(refpoints), n_end - n_start + 1)
	
	frameno=1
	start=time.time()
	while True:
		check, image = webcam.read()
		#print(frameno, frameno/(time.time()-start))
		frameno += 1
		cpoints = getpoints(image)
		state = [False]*len(refpoints)
		for i, rp in enumerate(refpoints):
			for cp in cpoints:
				if near(rp,cp,4):
					break
			else:
				state[i] = True
		
		if oldstate is not None:
			for i,(j,k) in enumerate(zip(oldstate, state)):
				if j != k:
					note(n_start+len(oldstate)-i, k)
					#if k:
					#	print("PRESSED: %d"%i)
					#else:
					#	print("RELEASED: %d"%i)
		oldstate = state
		
		cv2.imshow("thresholded", grey2)
		if cv2.waitKey(10)&0xfff == 27:
			break

except:
	raise



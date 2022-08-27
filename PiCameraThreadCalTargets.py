# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import threading
from threading import Thread
import time 
import math
import numpy as np


t0 = 0.0 #Used for testing performance
t1 = 0.0 #Used for testing performance

#Holders for target data
#pixels = [(0,0), (0,0), (0,0), (0,0)]
#pixel_data = ""

##Set capture resolution
width, height = (320 , 240)

CalFile = open ('CamExpCal').read().splitlines()
br = int(CalFile[0])
ct = int(CalFile[1])
st = int(CalFile[2])
bg = int(CalFile[3])
rg = int(CalFile[4])
sp = int(CalFile[5])

#This class is used to configure the Pi Camera and then continuously 
#acquire the latest frame from the camera. This prevents
#camera acquisition from blocking the image processing. 
class PiVideoStream:
	def __init__(self, resolution=(320, 240) , framerate = 90):
		# initialize the camera and stream
		self.camera = PiCamera()
		self.camera.resolution = resolution
		self.camera.framerate = framerate
		self.camera.awb_mode = 'off'
		self.camera.iso = 800
		self.camera.exposure_mode = 'off'
		#Set camera config values
		self.camera.brightness = br
		self.camera.contrast = (ct)
		self.camera.saturation = (st)
		self.camera.awb_gains = (bg, rg)
		self.camera.shutter_speed = sp
		self.rawCapture = PiRGBArray(self.camera, size=resolution)
		self.stream = self.camera.capture_continuous(self.rawCapture,
			format="bgr", use_video_port=True)
		# initialize the frame and the variable used to indicate
		# if the thread should be stopped
		self.frame = None
		self.stopped = False
		
	def start(self):
		# start the thread to read frames from the video stream
		Thread(target=self.update, args=()).start()
		return self
		
	def update(self):
		# keep looping infinitely until the thread is stopped
		for f in self.stream:
			# grab the frame from the stream and clear the stream in
			# preparation for the next frame
			self.frame = f.array
			self.rawCapture.truncate(0)
			# if the thread indicator variable is set, stop the thread
			# and resource camera resources
			if self.stopped:
				self.stream.close()
				self.rawCapture.close()
				self.camera.close()
				return
				
	def read(self):
		# return the frame most recently read
		return self.frame
		
	def stop(self):
		# indicate that the thread should be stopped
		self.stopped = True


#Width around selected value to set sliders to when mouse click
CalWidth = 20

#Start the image acquisition thread
cam = PiVideoStream().start()  #Used with Pi Camera
time.sleep(2.0)


##just some global colors for ease of use
blu =(255, 0, 0);
r = (0, 0, 255);
g = (0, 255, 0);
w = (255, 255, 255);


##Windows to display Images
cv2.namedWindow('Camera', cv2.WINDOW_AUTOSIZE )
cv2.namedWindow('HSV', cv2.WINDOW_AUTOSIZE )
cv2.namedWindow('Binary', cv2.WINDOW_AUTOSIZE )


#Action to take on mouse click
def on_mouse(event, x, y, flag, param):
	
	if event == cv2.EVENT_LBUTTONDOWN:
		Mouseframe = cam.read() # Grab and retrieve a frame from the cam
		MousehsvImage = cv2.cvtColor(Mouseframe, cv2.COLOR_BGR2HSV)
		m = MousehsvImage[y, x]
		cv2.setTrackbarPos('UpperHue', 'HSV',int (m[0]+CalWidth))
		cv2.setTrackbarPos('LowerHue', 'HSV',int (m[0]-CalWidth))
		#cv2.setTrackbarPos('UpperSat', 'HSV',int (m[1]+CalWidth))
		cv2.setTrackbarPos('UpperSat', 'HSV',255)
		cv2.setTrackbarPos('LowerSat', 'HSV',int (m[1]-CalWidth))
		#cv2.setTrackbarPos('UpperVal', 'HSV',int (m[2]+CalWidth))
		cv2.setTrackbarPos('UpperVal', 'HSV', 255)
		cv2.setTrackbarPos('LowerVal', 'HSV',int (m[2]-CalWidth))
		del (MousehsvImage)
		del (Mouseframe)
		
		


##Functions to grab new slider values
def changeUpperHueval(x):
    return
def changeUpperSatval(x):
    return
def changeUpperValval(x):
    return
def changeLowerHueval(x):
    return
def changeLowerSatval(x):
    return
def changeLowerValval(x):
    return
def delay(x):
    return
def dilate(x):
    return
def approx(x):
    return
def saturation(x):
    return
def contrast(x):
    return
def erode(x):
    return
def area(x):
    return
def sides(x):
    return
def solidity(x):
    return
 
    
##Create Trackbars used for calibration
cv2.createTrackbar('UpperHue', 'HSV', 0, 180, changeUpperHueval)
cv2.createTrackbar('LowerHue', 'HSV', 0, 255, changeLowerHueval)
cv2.createTrackbar('UpperSat', 'HSV', 0, 255, changeUpperSatval)
cv2.createTrackbar('LowerSat', 'HSV', 0, 255, changeLowerSatval)
cv2.createTrackbar('UpperVal', 'HSV', 0, 255, changeUpperValval)
cv2.createTrackbar('LowerVal', 'HSV', 0, 255, changeLowerValval)
cv2.createTrackbar('Dilate', 'Binary', 0, 20, dilate)
cv2.createTrackbar('Erode', 'Binary', 0, 20, erode)
cv2.createTrackbar('Approx', 'Binary', 0, 100, approx)
cv2.createTrackbar('Sides', 'Binary', 1, 8, sides)
cv2.createTrackbar('Area', 'Binary', 0, 5000, area)
cv2.createTrackbar('Solidity', 'Binary', 0, 100, solidity)


##Pre set Trackbar positions
cv2.setTrackbarPos('UpperHue', 'HSV', 180)
cv2.setTrackbarPos('LowerHue', 'HSV', 0) 
cv2.setTrackbarPos('UpperSat', 'HSV', 255)
cv2.setTrackbarPos('LowerSat', 'HSV', 0)
cv2.setTrackbarPos('UpperVal', 'HSV', 255)
cv2.setTrackbarPos('LowerVal', 'HSV', 0)
cv2.setTrackbarPos('Dilate', 'Binary', 1)
cv2.setTrackbarPos('Erode', 'Binary', 0)
cv2.setTrackbarPos('Approx', 'Binary', 8)
cv2.setTrackbarPos('Sides', 'Binary', 4)
cv2.setTrackbarPos('Area', 'Binary', 50)
cv2.setTrackbarPos('Solidity', 'Binary', 20)


#Image processing loop starts here
while(True):
	t0 = time.perf_counter()
	#Grab latest image from camera access thread
	Raw = cam.read()
	
	#Convert BGR("RGB") image to HSV
	hsvImage = cv2.cvtColor( Raw, cv2.COLOR_BGR2HSV)
	
	#Add mouse click selection
	cv2.setMouseCallback("Camera", on_mouse, 0)
	
	
	##Threshold values from Trackbars
	uh = cv2.getTrackbarPos('UpperHue', 'HSV')
	us = cv2.getTrackbarPos('UpperSat', 'HSV')
	uv = cv2.getTrackbarPos('UpperVal', 'HSV')
	lh = cv2.getTrackbarPos('LowerHue', 'HSV')
	ls = cv2.getTrackbarPos('LowerSat', 'HSV')
	lv = cv2.getTrackbarPos('LowerVal', 'HSV')
	er = cv2.getTrackbarPos('Erode', 'Binary')
	dl = cv2.getTrackbarPos('Dilate', 'Binary')
	ap = cv2.getTrackbarPos('Approx', 'Binary')
	sd = cv2.getTrackbarPos('Sides', 'Binary')
	ar = cv2.getTrackbarPos('Area', 'Binary')
	sl = cv2.getTrackbarPos('Solidity', 'Binary')

	## Threshold HSV Image to find specific color
	binImage = cv2.inRange(hsvImage, (lh, ls, lv), (uh, us, uv))
	
	# Erode image to remove noise
	binImage = cv2.erode(binImage, None, iterations = er)
	#Dilate image to fill in gaps
	binImage = cv2.dilate(binImage, None, iterations = dl) 
	cv2.imshow('Binary', binImage)
	
	
	##Finds contours (like finding edges/sides), 'contours' is what we are after
	contours, hierarchy = cv2.findContours(binImage, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_KCOS)
	
	##arrays to will hold the good/bad polygons
	squares = []
	badPolys = []
	#corners = []
	
	
	## Parse through contours to find targets
	for c in contours:
		if (contours != None) and (len(contours) > 0):
			#print ('Got here')
			cnt_area = cv2.contourArea(c)
			hull = cv2.convexHull(c , 1)
			hull_area = cv2.contourArea(hull)
			p = cv2.approxPolyDP(hull, ap, 1)
			#print (cv2.isContourConvex(p))
			if (cv2.isContourConvex(p) != False) and (len(p) == sd) and (cv2.contourArea(p) >= ar):
				filled = cnt_area/hull_area
				if filled <= sl/100 :  #Can be switched to >= if prefered
				    squares.append(p)
			else:
				badPolys.append(p)
	
	##draws targets in blue
	cv2.polylines(Raw, squares, 1, blu, 2, cv2.LINE_AA)
	
	##BoundingRectangles are just CvRectangles, so they store data as (x, y, width, height)
	##Calculate and draw the center of the rectangle based on the BoundingRect
	for s in squares:        
		br = cv2.boundingRect(s) 
		x = br[0] + (br[2]/2)
		y = br[1] + (br[3]/2)
		cv2.line(Raw, (int(x)-5,int(y)),(int(x)+5,int(y)),w, 1, cv2.LINE_AA)
		cv2.line(Raw, (int(x),int(y)-5),(int(x),int(y)+5),w, 1, cv2.LINE_AA)
		#print (s)
		
		##Store the corners as tuples
		br = (s[0][0][0],s[0][0][1])
		bl = (s[1][0][0],s[1][0][1])
		tl = (s[2][0][0],s[2][0][1])
		tr = (s[3][0][0],s[3][0][1])
		##print (br,bl,tl,tr)
		
		##Draw circles on the corners
		cv2.circle(Raw,tl, 3, r, -1, cv2.LINE_AA)
		cv2.circle(Raw,tr, 3, r, -1, cv2.LINE_AA)
		cv2.circle(Raw,bl, 3, r, -1, cv2.LINE_AA)
		cv2.circle(Raw,br, 3, r, -1, cv2.LINE_AA)
		cv2.putText(Raw,str(x),(int(x)-35,int(y)-10),cv2.FONT_HERSHEY_PLAIN, .8, w) ## Display x coordinate of center of target
		cv2.putText(Raw,str(y),(int(x)+10,int(y)-10),cv2.FONT_HERSHEY_PLAIN, .8, w) ## Display y coordinate of center of target

	
	cv2.imshow('Camera', Raw)
	#cv2.imshow('HSV', hsvImage)
	del (Raw)
	del (hsvImage)
	del (binImage)
	t1 = time.perf_counter()
	print (1/(t1-t0)), 'FPS'
	
	k = cv2.waitKey(1) & 0xFF # wait xx ms for specified key to be pressed
	# if the `q` key was pressed, break from the loop
	if k == ord("q"): 
		break # end the while loop

#Write calibration values to a text file named "Calibration" 
CalFile = open('Calibration', 'w')

CalFile.truncate()#Clear out old calibraton values
CalFile.write(str(uh))	
CalFile.write("\n")
CalFile.write(str(lh))	
CalFile.write("\n")
CalFile.write(str(us))	
CalFile.write("\n")
CalFile.write(str(ls))	
CalFile.write("\n")
CalFile.write(str(uv))	
CalFile.write("\n")
CalFile.write(str(lv))	
CalFile.write("\n")
CalFile.write(str(er))	
CalFile.write("\n")
CalFile.write(str(dl))	
CalFile.write("\n")
CalFile.write(str(ap))	
CalFile.write("\n")
CalFile.write(str(sd))	
CalFile.write("\n")
CalFile.write(str(ar))	
CalFile.write("\n")
CalFile.write(str(sl))	
CalFile.write("\n")

CalFile.close()#Close calibration file


##When everything is finished, release capture and kill windows
cam.stop() #Stop the image acquisition thread
cv2.destroyAllWindows() # Close the display window

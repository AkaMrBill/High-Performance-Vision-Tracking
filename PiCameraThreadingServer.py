# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import threading
from threading import Thread
import time 
import math
from networktables import NetworkTables

#t0 = 0.0 #Used for testing performance
#t1 = 0.0 #Used for testing and flush() trigger
# tLast = 0.0
# FlushRate = 100

#cv2.namedWindow('Camera', cv2.WINDOW_AUTOSIZE )

cond = threading.Condition()
notified = [False]


def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()

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
	def __init__(self, resolution=(320, 240)):
		# initialize the camera and stream
		self.camera = PiCamera()
		self.camera.resolution = resolution
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

#Start Camera Acquisition Thread
#cam = WebcamVideoStream(src=0).start() ##Used for USB camera
cam = PiVideoStream().start()  #Used with Pi Camera

#Begin NetworkTables and wait until connected
NetworkTables.initialize(server='10.TE.AM.2') #Set NT Server IP, Roborio IP
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    print("Waiting")
    if not notified[0]:
        cond.wait()

# Insert your processing code here
print("Connected!")

vs = NetworkTables.getTable('Vision') # Get the Vision NetworkTable

CalFile = open ('Calibration').read().splitlines()
uh = int(CalFile[0])
lh = int(CalFile[1])
us = int(CalFile[2])
ls = int(CalFile[3])
uv = int(CalFile[4])
lv = int(CalFile[5])
er = int(CalFile[6])
dl = int(CalFile[7])
ap = int(CalFile[8])
sd = int(CalFile[9])
ar = int(CalFile[10])
sl = int(CalFile[11])


x = 0 #X center of target
y = 0 #Y center of target
targeting = 0 # Is targeting or not

#Begin Image processing loop
while(True):
	#t0 = time.perf_counter()#Used for testing performance

	Raw = cam.read() ## For USB camera or Pi Camera

	#Convert BGR("RGB") image to HSV
	hsvImage = cv2.cvtColor( Raw, cv2.COLOR_BGR2HSV)
	
	## Threshold HSV Image to find specific color
	binImage = cv2.inRange(hsvImage, (lh, ls, lv), (uh, us, uv))
	
	# Erode image to remove noise
	binImage = cv2.erode(binImage, None, iterations = er)
	#Dilate image to fill in gaps
	binImage = cv2.dilate(binImage, None, iterations = dl) 

	
		##Finds contours (like finding edges/sides), 'contours' is what we are after
	contours, hierarchy = cv2.findContours(binImage, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_KCOS)
	
	##arrays to will hold the good/bad polygons
	squares = []
	badPolys = []
	corners = []
	
	
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
				if filled <= sl/100 : 
				    squares.append(p)
				    targeting = 1
			else:
				badPolys.append(p)
				targeting = 0
				# t1 = time.perf_counter()#Used for testing performance
				# vs.putNumber("ProcessTime", t1-t0)
				# NetworkTables.flush()
				
	    
	##BoundingRectangles are just CvRectangles, so they store data as (x, y, width, height)
	##Calculate the center of the rectangle based on the BoundingRect
	for s in squares:        
		br = cv2.boundingRect(s) 
		x = br[0] + (br[2]/2)
		y = br[1] + (br[3]/2)


	#FPS = (1/(t1-t0))#Used for testing performance
	#cv2.putText(Raw,str(FPS),(25,25),cv2.FONT_HERSHEY_PLAIN, .8, (255,255,255))
	vs.putNumber("Targeting", targeting)
	vs.putNumber("X", x)
	vs.putNumber("Y", y)
	#t1 = time.perf_counter()#Used for testing performance
	#vs.putNumber("ProcessTime", t1-t0)
	NetworkTables.flush()
	
	#To manually trigger flush() at desired rate other than 10ms
	#uncomment below
	# if ((t1-tLast) >= (1/FlushRate)): 
		# NetworkTables.flush()
		# tLast = t1
	
	#cv2.imshow('Camera', Raw)
	#t0 = t1 #Used for testing
	
	#The following "waitKey" block can be deleted if desired
	k = cv2.waitKey(1) # wait xx ms for specified key to be pressed
	if k % 256 == 27: # "27" is the "Esc" key
		break # end the while loop

##When everything is finished, release capture and kill windows
cam.stop()
cv2.destroyAllWindows() # Close the display window

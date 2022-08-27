# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

width = 320
height = 240
framerate = 90

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (width,height)
camera.framerate = (framerate)
camera.awb_mode = 'off'
camera.iso = 800
camera.exposure_mode = 'off'
rawCapture = PiRGBArray(camera, size=(width, height))

#Color to use for text
w = (255, 255, 255);

# allow the camera to warmup
time.sleep(2)
cv2.namedWindow('Camera', cv2.WINDOW_AUTOSIZE )

#Action to take on mouse click
def on_mouse(event, x, y, flag, param):
	
	if event == cv2.EVENT_LBUTTONDOWN:
		Mouseframe = frame.array # Grab and retrieve a frame from the cam
		R = Mouseframe[y,x] #Read pixel value at mouse
		print (str(R[1])) #Print Green value at mouse
		del (Mouseframe)
		
		


##Functions to grab new slider values
def brightness(x):
    return
def contrast(x):
    return
def saturation(x):
    return
def bluegain(x):
    return
def redgain(x):
    return
def shutterspeed(x):
    return
    
        
##Create Trackbars used for calibration
cv2.createTrackbar('Brightness', 'Camera', 0, 100, brightness)
cv2.createTrackbar('Contrast', 'Camera', 0, 200, contrast)
cv2.createTrackbar('Saturation', 'Camera', 0, 200, saturation)
cv2.createTrackbar('Blue Gain', 'Camera', 0, 8, bluegain)
cv2.createTrackbar('Red Gain', 'Camera', 0, 8, redgain)
cv2.createTrackbar('Shutter Speed', 'Camera', 1000, 10000 , shutterspeed)


##Pre set Trackbar positions
cv2.setTrackbarPos('Brightness', 'Camera', 50)
cv2.setTrackbarPos('Contrast', 'Camera', 100)
cv2.setTrackbarPos('Saturation', 'Camera', 100)
cv2.setTrackbarPos('Blue Gain', 'Camera', 1)
cv2.setTrackbarPos('Red Gain', 'Camera', 2)
cv2.setTrackbarPos('Shutter Speed', 'Camera', 1000)

# capture frames from the camera
#This loop grabs images from the camera and displays them
# with the values assigned by the trackbar positions
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	
	##Threshold values from Trackbars
	br = cv2.getTrackbarPos('Brightness', 'Camera')
	ct = cv2.getTrackbarPos('Contrast', 'Camera')
	st = cv2.getTrackbarPos('Saturation', 'Camera')
	bg = cv2.getTrackbarPos('Blue Gain', 'Camera')
	rg = cv2.getTrackbarPos('Red Gain', 'Camera')
	sp = cv2.getTrackbarPos('Shutter Speed', 'Camera')

	#Set camera config values
	camera.brightness = br
	camera.contrast = (ct - 100)
	camera.saturation = (st - 100)
	camera.awb_gains = (bg, rg)
	camera.shutter_speed = sp
	
	#Add mouse click selection
	cv2.setMouseCallback("Camera", on_mouse, 0)
	
	
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array
	# show the frame
	cv2.imshow("Camera", image)
	key = cv2.waitKey(1) & 0xFF
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

#Write Camera Exposure values to a text file named "CamExpCal" 
CalFile = open('CamExpCal', 'w')
CalFile.truncate()#Clear out old calibraton values
CalFile.write(str(br))	
CalFile.write("\n")
CalFile.write(str(ct - 100))	
CalFile.write("\n")
CalFile.write(str(st - 100))	
CalFile.write("\n")
CalFile.write(str(bg))	
CalFile.write("\n")
CalFile.write(str(rg))	
CalFile.write("\n")
CalFile.write(str(sp))

CalFile.close()#Close calibration file	

#Clean up and close windows
cv2.destroyAllWindows() # Close the display window

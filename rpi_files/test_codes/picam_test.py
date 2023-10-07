# import the necessary packages
import picamera
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
stream = camera.capture_continuous(rawCapture,format="bgr", use_video_port=True)

#capture frames from the camera
for frame in stream:
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array
    
    # show the frame
    cv2.imshow("Frame", image)
    key = cv2.waitKey(1) & 0xFF
    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    if cv2.countNonZero(mask) > 0:
        print('yellow detected')
        yellow = 1
#             motor_stop()
    else:
        print('not detected')
        yellow = 0
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
   
    
    # if the `q` key was pressed, break from the loop and close display window
    if key == ord("q"):
        cv2.destroyAllWindows()
        break
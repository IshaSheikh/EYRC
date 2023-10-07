import numpy as np
import cv2 as cv

cap = cv.VideoCapture("D:\\EYRC\\rpicon\\test_vid.mp4")
 
# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error opening video stream or file")
 
# Read until video is completed
while(cap.isOpened()):
  # Capture frame-by-frame
  ret, frame = cap.read()
  if ret == True:
    # ////
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # define range of white color in HSV
    # change it according to your need !
    lower_white = np.array([0,0,0], dtype=np.uint8)
    upper_white = np.array([0,0,255], dtype=np.uint8)

    # Threshold the HSV image to get only white colors
    mask = cv.inRange(hsv, lower_white, upper_white)
    # Bitwise-AND mask and original image
    res = cv.bitwise_and(frame,frame, mask= mask)

    cv.imshow('frame',frame)
    cv.imshow('mask',mask)
    cv.imshow('res',res)
    # ////

    if cv.waitKey(25) & 0xFF == ord('q'):
      cv.destroyAllWindows()
      break


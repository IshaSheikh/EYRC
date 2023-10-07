import cv2
import numpy as np
import time

# Load image, grayscale, Gaussian blur, Otsu's threshold
# cap = cv2.VideoCapture("D:\\EYRC\\rpicon\\copeliasim_road.mp4")
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if(frame is None):
       break
#     src = np.float32([[100,0],[40,frame.shape[1]],[frame.shape[0]-40,0],[frame.shape[0],frame.shape[1]]])
#     dst = np.float32([[0,0],[0,frame.shape[1]],[frame.shape[0],0],[frame.shape[0],frame.shape[1]]])
#     M = cv2.getPerspectiveTransform(src,dst)
#     image_t = cv2.warpPerspective(frame, M, (frame.shape[0], frame.shape[1]))

#     gray = cv2.cvtColor(image_t, cv2.COLOR_BGR2GRAY)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    gray = cv2.GaussianBlur(gray, (3,3), 0)
    _,thresh  = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
    thresh=255-thresh

    # Find contours
    contours,hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cX=0
    cY=0
    area=0
    ind=0
    if (len(contours)==0):
        continue
    cnt = max(contours,key=cv2.contourArea)
    
    cv2.drawContours(frame, [cnt], 0, (0,255,0), 3)
    cv2.drawContours(thresh, contours, -1, (0,255,0), 3)

    if cv2.waitKey(25) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        break

    M = cv2.moments(cnt)
    print(M["m00"])
    if M["m00"]==0:
        continue
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    
    # Draw the contour and center of the shape on the image
    cv2.circle(frame, (cX, cY), 10, (0, 0, 255), -1) 

    cv2.imshow('image.png', frame)
    cv2.imshow('thresh.png', thresh)
    # time.sleep(0.1)
       
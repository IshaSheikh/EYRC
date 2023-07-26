import cv2
import numpy as np
img= cv2.imread('D:\EYRC\PB_Task1_Windows\PB_Task1_Windows\Task1A\public_test_images\maze_0.png')

traffic_signals=[]
col=['A','B','C','D','E','F','G']
row=['1','2','3','4','5','6','7']

for j in range(1,8):
    for i in range(1,8):
        if (img[i*100][j*100]==(0,0,255)).all():
            traffic_signals.append(str(col[j-1])+str(row[i-1]))
print(traffic_signals)
import cv2
import numpy as np
maze_image= cv2.imread('D:\EYRC\PB_Task1_Windows\PB_Task1_Windows\Task1A\public_test_images\maze_0.png')
vertical_roads_under_construction = []
col=['A','B','C','D','E','F','G']
row=['1','2','3','4','5','6','7']
for j in range(1,7):
    for i in range(1,7):
        if (maze_image[i*100+50][j*100]==(255,255,255)).all():
            vertical_roads_under_construction.append(col[j-1]+row[i-1]+'-'+col[j-1]+row[i])
print(vertical_roads_under_construction)

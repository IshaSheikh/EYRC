import cv2
import numpy as np
img= cv2.imread('D:\EYRC\PB_Task1_Windows\PB_Task1_Windows\Task1A\public_test_images\maze_0.png')
green=[0,255,0]
orange=[0,127,255]
skyblue=[255,255,0]
pink=[180,0,255]

colour1=[]
for r in range(1,8):
    for c in range(1,8):
        for  i in range(207,294):
            for j in range(107,194):
                if (img[i][j]!=(255,255,255)).any():
                    if list(img[i][j]) not in colour1:
                        colour1.append(list(img[i][j]))
        color=[]
        for i in colour1:
            if i==green:
                color.append("Green")
            
            if i==orange:
            color.append("Orange")

            if i==skyblue:
                color.append("skyblue")
            
            if i==pink:
            color.append("Pink")
        print(color)
        colour1=[]
        for  i in range(107,194):
            for j in range(107,194):
                if (img[i][j]!=(255,255,255)).any():
                    if list(img[i][j]) not in colour1:
                        colour1.append(list(img[i][j]))
        color=[]
        for i in colour1:
            if i==green:
                color.append("Green")
            
            if i==orange:
            color.append("Orange")

            if i==skyblue:
                color.append("skyblue")
            
            if i==pink:
            color.append("Pink")
        print(color)




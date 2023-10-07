from zmqRemoteApi import RemoteAPIClient
import traceback
import cv2
from pyzbar.pyzbar import decode
import numpy as np

coppelia_client = RemoteAPIClient()
sim = coppelia_client.getObject('sim')

bot = sim.getObject('/bot')
# orange = sim.getObject('/Orange_cone')
# pink = sim.getObject('/Pink_cone')
floor = sim.getObject('/Floor')
# botPos = (sim.getObjectPosition(bot, bot))
# orangePos = [botPos[0]+0.0369, botPos[1]+0.015, botPos[2]+0.045]
# sim.setObjectPosition(orange, bot, orangePos)
# pinkPos = [botPos[0]+0.0369, botPos[1]-0.017, botPos[2]+0.045]
# sim.setObjectPosition(pink, bot, pinkPos)
# sim.setObjectParent(pink, bot, True)
# sim.setObjectParent(orange, bot, True)
# print(botPos)


# result = sim.setObjectInt32Param(bot, 10,65535)
# # sim.setObjectParent(cube, bot, True)

# sensor = sim.getObject('/bot/Vision_sensor')
# result = sim.getVisionSensorImg(sensor)

# buf = np.frombuffer(result[0], np.uint8)

# ht = int(result[1][0])
# wt = int(result[1][0])
# buf.shape = (ht, wt, 3)
# buf = cv2.flip(buf,0)	
# buf = cv2.cvtColor(buf, cv2.COLOR_RGB2BGR)
# output = decode(buf)
# print(output)
# for x in range(len(output)):
#     data = str(output[x].data)[2:-1]
    
##################################################

# print(data)
node = 'F3'

alphabets = ['A','B','C','D','E','F']
numbers = ['1','2','3','4','5','6']
constant = (0.89*2)/5
xval = constant * alphabets.index(node[0])
xcord =  xval - 0.89 
yval = constant * numbers.index(node[1])
ycord = 0.89 - yval 
center = [xcord, ycord]
center.append(0.15588)
print(center)

print(sim.getObjectPosition(bot, floor))
'''
*****************************************************************************************
*
*        		===============================================
*           		Pharma Bot (PB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script is to implement Task 3D of Pharma Bot (PB) Theme (eYRC 2022-23).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ 1079 ]
# Author List:		[ Sujitha A V, Ananth S, Aadithya R, Mohammed Sohail Sheikh ]
# Filename:			socket_server_pt1.py
# Functions:		
# 					[ Comma separated list of functions in this file ]

####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the three available ##
## modules for this task (numpy, opencv)                    ##
##############################################################
import socket
import time
import os, sys
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
import time
import numpy as np
import cv2 as cv2
##############################################################

################# ADD UTILITY FUNCTIONS HERE #################
# initializing the pin numbers where motors are connected
L_PWM_PIN1 = 32
L_PWM_PIN2 = 33
R_PWM_PIN2 = 38
R_PWM_PIN1 = 40
L_encoder = 8
R_encoder = 7

# initialising for camera
cap = cv2.VideoCapture(0)

# callback functions
def l_encoder_callback(channel):
    global left_count
    left_count += 1

def r_encoder_callback(channel):
    global right_count
    right_count += 1

# event detectors for encoders
GPIO.setup(L_encoder, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(R_encoder, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

GPIO.add_event_detect(L_encoder, GPIO.RISING, callback = l_encoder_callback, bouncetime = 50)
GPIO.add_event_detect(R_encoder, GPIO.RISING, callback = r_encoder_callback, bouncetime = 50)

# declare motor pins as output pins
# motors get input from the PWM pins
def motor_pin_setup():
	global L_MOTOR1, L_MOTOR2, R_MOTOR1, R_MOTOR2

	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(R_PWM_PIN1, GPIO.OUT, initial=GPIO.LOW)
	# GPIO.setup(R_PWM_PIN2, GPIO.OUT, initial=GPIO.LOW)
	GPIO.setup(L_PWM_PIN1, GPIO.OUT, initial=GPIO.LOW)
	# GPIO.setup(L_PWM_PIN2, GPIO.OUT, initial=GPIO.LOW)
	GPIO.setup(31, GPIO.OUT, initial=GPIO.HIGH)
	GPIO.setup(37, GPIO.OUT, initial=GPIO.HIGH)

	# setting initial PWM frequency for all 4 pins
	L_MOTOR1 = GPIO.PWM(L_PWM_PIN1, 100) 
	R_MOTOR1 = GPIO.PWM(R_PWM_PIN1, 100)
	# L_MOTOR2 = GPIO.PWM(L_PWM_PIN2, 100)
	# R_MOTOR2 = GPIO.PWM(R_PWM_PIN2, 100) 

	# setting initial speed (duty cycle) for each pin as 0
	L_MOTOR1.start(0)
	R_MOTOR1.start(0)
	# L_MOTOR2.start(0)
	# R_MOTOR2.start(0)

def stop():
    L_MOTOR1.stop()
    R_MOTOR1.stop()
    L_MOTOR2.stop()
    R_MOTOR2.stop()
    
def move_forward():
	check=0
	src = np.float32([[80,0],[20,512],[442,0],[512,512]])
	dst = np.float32([[0,0],[0,frame.shape[1]],[frame.shape[0],0],[frame.shape[0],frame.shape[1]]])
	while True:
		ret, frame = cap.read()
		
		M = cv2.getPerspectiveTransform(src,dst)
		image = cv2.warpPerspective(frame, M, (frame.shape[0], frame.shape[1]))
		centre=(int(image.shape[0]/2),int(image.shape[1]/2))

	# //// checking if yellow is detected and if detected waitng for it to end and stop the motor
		if(yellow(image)):
			check=1
		elif(not yellow(image)):
			if(check):
				time.sleep(0.5)
				stop()
				return
	# ////


	# start feature detection
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		
		# blur = cv2.GaussianBlur(gray, (3,3), 0)
		# circle=cv2.circle(image,centre,240,(0,0,255),3)
		
		ret, thresh1 = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)
		contours,hierarchy = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		# print("\nlength of contour\n"+ str(len(contours)))

		Moments = cv2.moments(contours[3])
		x = int(Moments["m10"] / Moments["m00"])
		# y = int(Moments["m01"] / Moments["m00"])
		# print(Moments)
		# print(x)
		# thresh1=cv2.circle(thresh1,(x,256),5,(0,0,255),-1)

		error= (centre[0]-x)/frame.shape[1]
		error_duty_cycle=error*50
		forward(50+error_duty_cycle,50-error_duty_cycle)

	
def yellow(image):
    flag = 0
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    if cv2.countNonZero(mask) > 0:
        # contains yellow
        flag = 1
    return flag

def run_motor(speed_left,speed_right):
	L_MOTOR1.ChangeDutyCycle(speed_left)
	R_MOTOR1.ChangeDutyCycle(speed_right)
	
# turn left
def turn_left():
    global right_count
    right_count = 0
    run_motor(0, 50)
    while (right_count < 240):
        pass
    stop()
    right_count = 0

# turn right
def turn_right():
    global left_count
    left_count = 0
    run_motor(50, 0)
    while (left_count < 240):
        pass
    stop()
    left_count = 0

		
	

def move(cmd):
    
    if(cmd=="STRAIGHT"):
		# run motor
		move_forward()
		
	elif(cmd=="LEFT"):
		# turn
		turn_left()
	elif(cmd=="RIGHT"):
		# turn
		turn_right
		
	elif(cmd=="WAIT_5"):
		# turn
		time.sleep(5)

	elif(cmd=="PICKUP"):
		send_message_via_socket("pickup location reached")

	elif(cmd=="DELIVERY"):
		# turn off orange led or red
		send_message_via_socket("delivery location reached")
    

##############################################################

def setup_client(host, port):
	client = None

	##################	ADD YOUR CODE HERE	##################
	client = socket.socket()
	client.connect((host, port))

	##########################################################
	return client

def receive_message_via_socket(client):
	"""
	Purpose:
	---
	This function listens for a message from the specified
	socket client and returns the message when received.
	Input Arguments:
	---
	`client` :	[ socket object ]
			socket client object created by setup_client() function
	Returns:
	---
	`message` : [ string ]
			message received through socket communication
	
	Example call:
	---
	message = receive_message_via_socket(connection)
	"""

	message = None

	##################	ADD YOUR CODE HERE	##################
	message = client.recv(1024).decode()

	##########################################################

	return message

def send_message_via_socket(client, message):
	"""
	Purpose:
	---
	This function sends a message over the specified socket client
	Input Arguments:
	---
	`client` :	[ socket object ]
			client socket object created by setup_client() function
	`message` : [ string ]
			message sent through socket communication
	Returns:
	---
	None
	
	Example call:
	---
	send_message_via_socket(connection, message)
	"""

	##################	ADD YOUR CODE HERE	##################
	client.send(message.encode())

	##########################################################
	

def decode(message):
	path=message.split()
	for cmds in path:
		move(cmds)
		send_message_via_socket("Reached")

def led_indicator(message):
	color=message.split()
	if ("Orange" in color[0]) or ("Orange" in color[1]):
		# turn on orange 
		i=0
	if("Pink" in color[0]) or ("Pink" in color[1]):
		# turn on pink
	
	

	
	


	
######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THE MAIN SECTION #########
#########      APART FROM THE REQUIRED AREAS (host, port etc)     #########

if __name__ == "__main__":

	host = "192.168.29.208"
	port = 5050


	## Set up new socket client and connect to a socket server
	try:
		client = setup_client(host, port)

		# print(type(client))

	except socket.error as error:
		print("Error in setting up client")
		print(error)
		sys.exit()

	# set up pin
	motor_pin_setup()

	##send message to the server that the bot is ready to recive commands
	print("\nwaiting for path_string\n")

	#recieve pick up path 

	while(True):
		message = receive_message_via_socket(client)
		if message is not None:
			print("\npath_string for pickup recieved:" + message+"\n")
			break
	decode(message)

	# recieve LED info
	while(True):
		message = receive_message_via_socket(client)
		if message is not None:
			print("\nLED info recieved:" + message+"\n")
			break
	led_indicator(message)

	# recieve first delivery path
	while(True):
		message = receive_message_via_socket(client)
		if message is not None:
			print("\npath_string for pickup recieved:" + message+"\n")
			break
	decode(message)
	# turn of first led

	# recieve the second path
	while(True):
		message = receive_message_via_socket(client)
		if message is not None:
			print("\npath_string for pickup recieved:" + message+"\n")
			break
	decode(message)
	# turn off second led

	# recieve the third path for goint to the end node
	while(True):
		message = receive_message_via_socket(client)
		if message is not None:
			print("\npath_string for end node recieved:" + message+"\n")
			break
	decode(message)
	send_message_via_socket("reached end node")	

		
		
		
		


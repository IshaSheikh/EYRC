import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
import math
import picamera
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import threading
import socket
import time
import os, sys

WHEEL_RADIUS = 0.0325 # wheel radius in meters
WHEEL_SEPARATION = 0.13 # wheel separation in meters
ENCODER_RESOLUTION = 19 # encoder ticks per revolution
L_PWM_PIN1 = 32
L_PWM_PIN2 = 33
R_PWM_PIN2 = 38
R_PWM_PIN1 = 40
LEFT_WHEEL_GPIO = 24
RIGHT_WHEEL_GPIO = 26
done = 0
left_encoder_value = 0
right_encoder_value = 0

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
stream = camera.capture_continuous(rawCapture,format="bgr", use_video_port=True)

def left_encoder_handler(gpio):
    global left_encoder_value, right_encoder_value
    left_encoder_value += 1

# Function to handle the right wheel encoder interrupt
def right_encoder_handler(gpio):
    global right_encoder_value, left_encoder_value
    right_encoder_value -= 1
   
   
GPIO.setup(LEFT_WHEEL_GPIO, GPIO.IN)
GPIO.setup(RIGHT_WHEEL_GPIO, GPIO.IN)
GPIO.add_event_detect(LEFT_WHEEL_GPIO, GPIO.RISING, callback=left_encoder_handler)
GPIO.add_event_detect(RIGHT_WHEEL_GPIO, GPIO.RISING, callback=right_encoder_handler)


def turn_right():
    global right_encoder_value
    # Calculate the distance that each wheel should travel
    circumference = 2 * math.pi * WHEEL_RADIUS
    distance = WHEEL_SEPARATION * (90*math.pi/180)
    wheel_distance = distance
    # Calculate the number of encoder ticks required to travel the distance
    encoder_ticks = wheel_distance / circumference * ENCODER_RESOLUTION
    # Set the initial encoder values
    initial_right_encoder = right_encoder_value
    # Set the wheel speeds
    forward(50, 0)
    encoder_ticks=int(encoder_ticks)
    # Wait until the desired number of encoder ticks have been travelled
    while True:
        if (right_encoder_value - initial_right_encoder <= -encoder_ticks):
            break
    # Stop the wheels
    motor_stop()
    time.sleep(0.5)
  
  
def turn_left():
    global left_encoder_value
    # Calculate the distance that each wheel should travel
    circumference = 2 * math.pi * WHEEL_RADIUS
    distance = WHEEL_SEPARATION * (90*math.pi/180)
    wheel_distance = distance
    # Calculate the number of encoder ticks required to travel the distance
    encoder_ticks = wheel_distance / circumference * ENCODER_RESOLUTION
    # Set the initial encoder values
    initial_left_encoder = left_encoder_value
    # Set the wheel speeds
    forward(0, 50)
    encoder_ticks=int(encoder_ticks)
    # Wait until the desired number of encoder ticks have been travelled
    while True:
        if (left_encoder_value - initial_left_encoder >= encoder_ticks):
            break
    # Stop the wheels
    motor_stop()
    time.sleep(0.5)

def move(cmd):
    if(cmd == "STRAIGHT"):
        motor_run(50,50)
        move_forward()
    elif(cmd == "LEFT"):
        turn_left()
    elif(cmd == "RIGHT"):
        turn_right()
    elif(cmd == "WAIT_5"):
        motor_pause(5)
    elif(cmd == "PICKUP"):
        send_message_via_socket("Pickup location reached")
    elif(cmd == "DELIVERY"):
        # turn off orange led or red
        send_message_via_socket("Delivery location reached")

def decode(message):
	path = message.split()
	for cmds in path:
		move(cmds)
		# send_message_via_socket("Reached")

def send_message_via_socket(message):
    global client
    client.send(message.encode())

def receive_message_via_socket(client):
	message = client.recv(1024).decode()
	return message

def setup_client(host, port):
	client = socket.socket()
	client.connect((host, port))
	return client

def motor_pin_setup():
    global L_MOTOR1, L_MOTOR2, R_MOTOR1, R_MOTOR2
    
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(R_PWM_PIN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(R_PWM_PIN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(L_PWM_PIN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(L_PWM_PIN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(31, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(37, GPIO.OUT, initial=GPIO.HIGH)
#     GPIO.setup(L_ENCODER, GPIO.IN)
#     GPIO.setup(R_ENCODER, GPIO.IN)

    # setting initial PWM frequency for all 4 pins
    L_MOTOR_R = GPIO.PWM(L_PWM_PIN1, 100) 
    R_MOTOR_R = GPIO.PWM(R_PWM_PIN1, 100)
    L_MOTOR_F = GPIO.PWM(L_PWM_PIN2, 100)
    R_MOTOR_F = GPIO.PWM(R_PWM_PIN2, 100) 
    
    # setting initial speed (duty cycle) for each pin as 0
    L_MOTOR1.start(0)
    R_MOTOR1.start(0)
    L_MOTOR2.start(0)
    R_MOTOR2.start(0)

def motor_run(left, right):
    L_MOTOR_F.ChangeDutyCycle(left)
    R_MOTOR_F.ChangeDutyCycle(right)
    
def stop():
    L_MOTOR1.stop()
    R_MOTOR1.stop()
    L_MOTOR2.stop()
    R_MOTOR2.stop()
    GPIO.cleanup()
    

def yellow(frame):
    flag = 0    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    if cv2.countNonZero(mask) > 0:
        # contains yellow
        flag = 1
        return flag
    else:
        return flag

def move_forward():
   
    Kp = 0.5
    Ki = 0.01
    Kd = 0.1
    check = 0
    lastError = 0
    error_l = 0
    error_r = 0
    integral_r = 0
    integral_l = 0
    prev_error_r = 0
    prev_error_l = 0
    current_time = time.time()
    print('in move_forward')
    cap = cv2.VideoCapture(0)
    while True:
        # time.sleep(0.2)
        ret, frame = cap.read()
        if not ret:
            break

        # cv2.imshow("frame",frame)
        val=yellow(frame)
        if val:
            print("yellow")
            # check=1
            
            motor_stop()
            time.sleep(0.5)
            print('time over')
            cap.release()
            return 
        else :
            # if check:
            #     motor_stop()
            #     # stop()
            #     time.sleep(1)
                
            #     check = 0
            #     return
            pass
         
        centre=(int(frame.shape[1]/2),int(frame.shape[0]/2))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _,thresh  = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
        thresh=255-thresh
        # Find contours
        contours,hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if(len(contours)==0):
            continue
        cnt = max(contours,key=cv2.contourArea)
        M = cv2.moments(cnt)
        if M["m00"]==0:
            continue
        cX = int(M["m10"] / M["m00"])
        # cY = int(M["m01"] / M["m00"])

        error_l = (centre[0] - cX)/ frame.shape[1]
        integral_l += error_l * (time.time() - current_time)

        # Calculate the derivative term
        derivative_l = (error_l - prev_error_l) / (time.time() - current_time)

        # Calculate the control signal
        control_signal_l = Kp * error_l + Ki * integral_l + Kd * derivative_l

        min_duty_cycle = 0
        max_duty_cycle = 50
        duty_cycle = min_duty_cycle + (control_signal_l / centre[0]) * (max_duty_cycle - min_duty_cycle)

        # Limit the duty cycle to the minimum and maximum values
        duty_cycle = min(max_duty_cycle, max(min_duty_cycle, duty_cycle))

        if check:
            pass
        else:
            motor_run(50+duty_cycle, 50-duty_cycle)
            # error= (centre[0]-cX)/frame.shape[1]
            # if error>0.05:
            #     forward(70-error*10,70+error*10)
            # elif error<-0.05:
            #     forward(70+error*10,70-error*10)
        prev_error_l = error_l
        # print(error)


def motor_stop():
    L_MOTOR1.ChangeDutyCycle(0)
    R_MOTOR1.ChangeDutyCycle(0)
    L_MOTOR2.ChangeDutyCycle(0)
    R_MOTOR2.ChangeDutyCycle(0)


def motor_pause(secs):
    L_MOTOR1.ChangeDutyCycle(0)
    R_MOTOR1.ChangeDutyCycle(0)
    L_MOTOR2.ChangeDutyCycle(0)
    R_MOTOR2.ChangeDutyCycle(0)
    time.sleep(secs)

def rgb_leds_setup():
    redPin1 = 8
    bluePin1 = 10
    greenPin1 = 12
    redPin2 = 3
    bluePin2 = 5
    greenPin2= 7
    redPin3 = 19
    greenPin3 = 21
    bluePin3 = 23
    
    GPIO.setup(redPin1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(bluePin1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(greenPin1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(redPin2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(bluePin2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(greenPin2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(redPin3, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(bluePin3, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(greenPin3, GPIO.OUT, initial=GPIO.LOW)
	
    global red1, blue1, green1, red2, blue2, green2, red3, blue3, green3
    red1 = GPIO.PWM(redPin1, 50)
    blue1 = GPIO.PWM(bluePin1, 50)
    green1 = GPIO.PWM(greenPin1, 50)
    red2 = GPIO.PWM(redPin2, 50)
    blue2 = GPIO.PWM(bluePin2, 50)
    green2 = GPIO.PWM(greenPin2, 50)
    red3 = GPIO.PWM(redPin3, 50)
    blue3 = GPIO.PWM(bluePin3, 50)
    green3 = GPIO.PWM(greenPin3, 50)

    red1.start(0)
    blue1.start(0)
    green1.start(0)
    red2.start(0)
    blue2.start(0)
    green2.start(0)
    red3.start(0)
    blue3.start(0)
    green3.start(0)
	
def rgb_leds_set_color(color, led_num):
    global red1, blue1, green1, red2, blue2, green2, red3, blue3, green3
    rc = 0
    bc = 0
    gc = 0
    pwm_values = {"Red": (255, 0, 0), "Blue": (0, 0, 255), "Green": (0, 255, 0), "Orange": (255, 35, 0), "Pink": (255, 0, 122), "Sky Blue": (0, 100, 100)}
    rc = list(pwm_values[color])[0]
    gc = list(pwm_values[color])[1]
    bc = list(pwm_values[color])[2]

    if led_num == 1:
        red1.ChangeDutyCycle(rc/255*100)
        green1.ChangeDutyCycle(gc/255*100)
        blue1.ChangeDutyCycle(bc/255*100)
    elif led_num == 2:
        red2.ChangeDutyCycle(rc/255*100)
        green2.ChangeDutyCycle(gc/255*100)
        blue2.ChangeDutyCycle(bc/255*100)
    elif led_num == 3:
        red3.ChangeDutyCycle(rc/255*100)
        green3.ChangeDutyCycle(gc/255*100)
        blue3.ChangeDutyCycle(bc/255*100)



def led_indicator(message):
    colours = message.split(',')
    for x in range(len(colours)):
        if ('Orange' in colours[x]):
            rgb_leds_set_color('Orange', x+1)
        elif ('Red' in colours[x]):
            rgb_leds_set_color('Red', x+1)
        elif ('Green' in colours[x]):
            rgb_leds_set_color('Green', x+1)
        elif ('Sky Blue' in colours[x]):
            rgb_leds_set_color('Sky Blue', x+1)
        elif ('Pink' in colours[x]):
            rgb_leds_set_color('Pink', x+1)
        elif ('Blue' in colours[x]):
    send_message_via_socket('LEDs done')

def led_stop(led_num):
    if led_num == 1:
        red1.stop()
        green1.stop()
        blue1.stop()
    elif led_num == 2:
        red2.stop()
        green2.stop()
        blue2.stop()
    elif led_num == 3:
        red3.stop()
        green3.stop()
        blue3.stop()
    elif led_num==0:
        red1.stop()
        green1.stop()
        blue1.stop()
        red2.stop()
        green2.stop()
        blue2.stop()
        red3.stop()
        green3.stop()
        blue3.stop()
        



def main():
    host = "192.168.141.182"
    port = 5050

    ## Set up new socket client and connect to a socket server
    try:
        global client
        client = setup_client(host, port)
        # print(type(client))
    except socket.error as error:
        print("Error in setting up client")
        print(error)
        sys.exit()

    # set up pin
    motor_pin_setup()
    rgb_leds_setup()

    ##send message to the server that the bot is ready to recive commands
    print("\nWaiting for Path_string\n")

    #recieve pick up path 
    while(True):
        message = receive_message_via_socket(client)
        if message is not None:
            print("\nPath_string for pickup recieved: " + message+ "\n")
            break
    # decode(message)
    send_message_via_socket("Delivery location reached")

    # recieve LED info
    while(True):
        message = receive_message_via_socket(client)
        if message is not None:
            print("\nLED info recieved: " + message+"\n")
            break
    # led_indicator(message)
    # time.sleep(10)
    send_message_via_socket("LED indication done")

    # recieve first delivery path
    while(True):
        message = receive_message_via_socket(client)
        if message is not None:
            print("\nPath_string for pickup recieved: " + message+"\n")
            break
    # decode(message)
    # # turn of first led
    # time.sleep(3)
    # led_stop(1)
    send_message_via_socket("1st Delivery location reached")

  

    # recieve the second path
    while(True):
        message = receive_message_via_socket(client)
        if message is not None:
            print("\nPath_string for pickup recieved: " + message+"\n")
            break
    
    # decode(message)
    # turn off second led
    # time.sleep(3)
    # led_stop(2)
    send_message_via_socket("2nd Delivery location reached")

   
    # recieve the third path for goint to the end node
    while(True):
        print('trying to receive 3rd')
        message = receive_message_via_socket(client)
        if message is not None:
            print("\nPath_string for end node recieved: " + message+"\n")
            break
    send_message_via_socket("3rd Delivery location reached")
    # decode(message)

    send_message_via_socket("Reached end node")	
    stop()
    GPIO.setmode(GPIO.BOARD)
    GPIO.remove_event_detect(24)
    GPIO.remove_event_detect(26)
    GPIO.cleanup()

    
try:
    main()
    led_stop(0)
    rgb_leds_setup()
    GPIO.cleanup()
except Exception as e:
    print('Exception ', e)
    GPIO.cleanup()
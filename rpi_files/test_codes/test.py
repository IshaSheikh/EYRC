import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
import time
import math

import cv2
import numpy as np

WHEEL_RADIUS = 0.0325 # wheel radius in meters
WHEEL_SEPARATION = 0.13 # wheel separation in meters
ENCODER_RESOLUTION = 19 # encoder ticks per revolution
LEFT_WHEEL_GPIO = 24 # GPIO pin for the left wheel encoder
RIGHT_WHEEL_GPIO = 26 # GPIO pin for the right wheel encoder

L_PWM_PIN1 = 32
L_PWM_PIN2 = 33
R_PWM_PIN2 = 38
R_PWM_PIN1 = 40
L_encoder = 24
R_encoder = 26
done = 0
# Variables to keep track of the encoder values
left_encoder_value = 0
right_encoder_value = 0

# cap = cv2.VideoCapture(0)
# camera = PiCamera()
# camera.resolution = (640, 480)
# camera.framerate = 32
# rawCapture = PiRGBArray(camera, size=(640, 480))
# stream = camera.capture_continuous(rawCapture,format="bgr", use_video_port=True)

def left_encoder_handler(gpio):
    global left_encoder_value, right_encoder_value
    left_encoder_value += 1

# Function to handle the right wheel encoder interrupt
def turn_rev():
    left_MOTOR_Forward.ChangeDutyCycle(left)
    right_MOTOR_Forward.ChangeDutyCycle(right)
    


def right_encoder_handler(gpio):
    global right_encoder_value, left_encoder_value
    right_encoder_value -= 1
   
   
GPIO.setup(LEFT_WHEEL_GPIO, GPIO.IN)
GPIO.setup(RIGHT_WHEEL_GPIO, GPIO.IN)
GPIO.add_event_detect(LEFT_WHEEL_GPIO, GPIO.RISING, callback=left_encoder_handler)
GPIO.add_event_detect(RIGHT_WHEEL_GPIO, GPIO.RISING, callback=right_encoder_handler)

def turn_left():
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
    encoder_ticks=int(encoder_ticks)+3
    # Wait until the desired number of encoder ticks have been travelled
    while True:
        if (right_encoder_value - initial_right_encoder <= -encoder_ticks):
            break
    # Stop the wheels
    motor_stop()
    time.sleep(0.5)


def turn_right():
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
    return
  

def motor_pin_setup():
    global left_MOTOR_Forward, left_MOTOR_Rev, right_MOTOR_Rev, right_MOTOR_Forward
    
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(R_PWM_PIN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(R_PWM_PIN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(L_PWM_PIN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(L_PWM_PIN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(31, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(37, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(L_encoder, GPIO.IN)
    GPIO.setup(R_encoder, GPIO.IN)
    
    # setting initial PWM frequency for all 4 pins
    left_MOTOR_Rev = GPIO.PWM(L_PWM_PIN1, 150) 
    right_MOTOR_Rev = GPIO.PWM(R_PWM_PIN1, 150)
    left_MOTOR_Forward = GPIO.PWM(L_PWM_PIN2, 150)
    right_MOTOR_Forward = GPIO.PWM(R_PWM_PIN2, 150) 
    
    # setting initial speed (duty cycle) for each pin as 0
    left_MOTOR_Rev.start(0)
    right_MOTOR_Rev.start(0)
    left_MOTOR_Forward.start(0)
    right_MOTOR_Forward.start(0)

# function for moving forward
def forward(left, right):
    left_MOTOR_Forward.ChangeDutyCycle(left)
    right_MOTOR_Forward.ChangeDutyCycle(right)
    
def move(cmd):
    if(cmd == "STRAIGHT"):
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

def move_forward():
   
    Kp =100
    Ki = 0
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
        
        if yellow(frame):
            print("yellow")
            check=1
            
        else :
            if check:
                time.sleep(1)
                motor_stop()
                stop()
                check = 0
                break
            
         
        centre=(int(frame.shape[1]/2),int(frame.shape[0]/2))
        print(centre)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _,thresh  = cv2.threshold(gray, 125, 255, cv2.THRESH_BINARY)
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
        print(error_l)
        integral_l += error_l * (time.time() - current_time)

        # Calculate the derivative term
        derivative_l = (error_l - prev_error_l) / (time.time() - current_time)

        # Calculate the control signal
        control_signal_l = Kp * error_l + Ki * integral_l + Kd * derivative_l

        min_duty_cycle = 0
        max_duty_cycle = 50
        # duty_cycle = min_duty_cycle + (control_signal_l / centre[0]) * (max_duty_cycle - min_duty_cycle)
        duty_cycle = min_duty_cycle +control_signal_l
        print("duty cycle:",duty_cycle)

        # Limit the duty cycle to the minimum and maximum values
        duty_cycle = min(max_duty_cycle, max(min_duty_cycle, duty_cycle))
        print("duty cycle   :",duty_cycle)

        if check:
            pass
        else:
           
            forward(50+duty_cycle, 50-duty_cycle)
            # error= (centre[0]-cX)/frame.shape[1]
            # if error>0.05:
            #     forward(70-error*10,70+error*10)
            # elif error<-0.05:
            #     forward(70+error*10,70-error*10)
        prev_error_l = error_l
        # print(error)

    cap.release()

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

def stop():
    left_MOTOR_Forward.stop()
    right_MOTOR_Forward.stop()
    left_MOTOR_Rev.stop()
    right_MOTOR_Rev.stop()
    GPIO.cleanup()
    
def main():
    
    motor_pin_setup()
    move_forward()
    # turn_left()
    # time.sleep(0.3)
    # turn_right()
    # move_forward()


    return 0

def motor_stop():
    left_MOTOR_Forward.ChangeDutyCycle(0)
    right_MOTOR_Forward.ChangeDutyCycle(0)
    right_MOTOR_Rev.ChangeDutyCycle(0)
    left_MOTOR_Rev.ChangeDutyCycle(0)
   

# main()
try:
    main()
except Exception as e:
    print('Error is: ', e)
    GPIO.cleanup()
    print('Error is: ', e)
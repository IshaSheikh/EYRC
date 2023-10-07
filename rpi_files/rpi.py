import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
import time
import picamera
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np

# initializing the pin numbers where motors are connected
L_PWM_PIN1 = 32
L_PWM_PIN2 = 33
R_PWM_PIN2 = 38
R_PWM_PIN1 = 40
L_encoder = 8
R_encoder = 7

# initialize the camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
# stream = camera.capture_continuous(rawCapture,format="bgr", use_video_port=True)
# camera.capture(rawCapture, format="bgr")
# image = rawCapture.array

def motor_pin_setup():
    global L_MOTOR1, L_MOTOR2, R_MOTOR1, R_MOTOR2
    
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(R_PWM_PIN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(R_PWM_PIN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(L_PWM_PIN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(L_PWM_PIN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(31, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(37, GPIO.OUT, initial=GPIO.HIGH)
    # GPIO.setup(L_encoder, GPIO.IN)
    # GPIO.setup(R_encoder, GPIO.IN)

    # setting initial PWM frequency for all 4 pins
    L_MOTOR1 = GPIO.PWM(L_PWM_PIN1, 100) 
    R_MOTOR1 = GPIO.PWM(R_PWM_PIN1, 100)
    L_MOTOR2 = GPIO.PWM(L_PWM_PIN2, 100)
    R_MOTOR2 = GPIO.PWM(R_PWM_PIN2, 100) 
    
    # setting initial speed (duty cycle) for each pin as 0
    L_MOTOR1.start(0)
    R_MOTOR1.start(0)
    L_MOTOR2.start(0)
    R_MOTOR2.start(0)

# function for moving forward
def forward(left, right):
    L_MOTOR1.ChangeDutyCycle(left)
    R_MOTOR1.ChangeDutyCycle(right)
    
# function for moving backward
def backward(left, right):
    R_MOTOR2.ChangeDutyCycle(left)
    L_MOTOR2.ChangeDutyCycle(right)

# function for pausing the motors
def stop_wait(secs):
    L_MOTOR1.ChangeDutyCycle(0)
    R_MOTOR1.ChangeDutyCycle(0)
    L_MOTOR2.ChangeDutyCycle(0)
    R_MOTOR2.ChangeDutyCycle(0)
    time.sleep(secs)

# function for stopping both motors
def stop():
    L_MOTOR1.stop()
    R_MOTOR1.stop()
    L_MOTOR2.stop()
    R_MOTOR2.stop()

# stop
def motor_stop():
    L_MOTOR1.ChangeDutyCycle(0)
    R_MOTOR1.ChangeDutyCycle(0)
    L_MOTOR2.ChangeDutyCycle(0)
    R_MOTOR2.ChangeDutyCycle(0)

# turn left
def turn_left():
    global right_count
    right_count = 0
    forward(0, 50)
    while (right_count < 240):
        pass
    motor_stop()
    right_count = 0

# turn right
def turn_right():
    global left_count
    left_count = 0
    forward(50, 0)
    while (left_count < 240):
        pass
    motor_stop()
    left_count = 0

# detect yellow
def yellow():
    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array
    flag = 0
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    if cv2.countNonZero(mask) > 0:
        # contains yellow
        flag = 1
    rawCapture.truncate(0)
    return flag

# straight line pid
def st_line():
    global kp
    kp = 0.1
    while not yellow():
        left_speed = speed_left()
        right_speed = speed_right()
        error = left_speed - right_speed
        dc_error = (kp*error/left_speed)*100
        forward(50 + dc_error, 50 - dc_error)


# find speed
def speed_left():
    c1 = left_count
    time.sleep(0.1)
    c2 = left_count
    speed = (c2 - c1)/ 0.1
    return speed

right_count = 0
# find speed
def speed_right():
    global right_count
    c1 = right_count
    time.sleep(0.1)
    c2 = right_count
    speed = (c2 - c1)/ 0.1
    return speed

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

# main function
def main():
    motor_pin_setup()
    forward(50,50)
    time.sleep(1)
    print(speed_right())
    stop()

try:
    main()
except KeyboardInterrupt:
    GPIO.cleanup()
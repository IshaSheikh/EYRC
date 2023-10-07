import RPi.GPIO as GPIO
import time
import math
import threading
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
import picamera
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np

# Constants
WHEEL_RADIUS = 0.0325 # wheel radius in meters
WHEEL_SEPARATION = 0.13 # wheel separation in meters
ENCODER_RESOLUTION = 19 # encoder ticks per revolution
LEFT_WHEEL_GPIO = 24 # GPIO pin for the left wheel encoder
RIGHT_WHEEL_GPIO = 26 # GPIO pin for the right wheel encoder
L_PWM_PIN1 = 32
L_PWM_PIN2 = 33
R_PWM_PIN2 = 38
R_PWM_PIN1 = 40

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
stream = camera.capture_continuous(rawCapture,format="bgr", use_video_port=True)

# Variables to keep track of the encoder values
left_encoder_value = 0
right_encoder_value = 0
done = 0
yellow = 0
target_speed = 30 # revs/sec
# Set the PID parameters
Kp = 0.5
Ki = 0.01
Kd = 0.1
# Initialize the error and integral_l_l terms
error_l = 0
error_r = 0
integral_l = 0
integral_r = 0
# Create variables to store the previous error and the current time
prev_error_l = 0
prev_error_r = 0
current_time = time.time()
left_speed = 0 
right_speed = 0

def get_speeds():
    global left_speed, right_speed, done, left_encoder_value, right_encoder_value
    while not done:
        initial_right_encoder_value = right_encoder_value
        initial_left_encoder_value = left_encoder_value
        start_time = time.time()
        time.sleep(0.5)
        elapsed_time = time.time() - start_time
        left_speed = (left_encoder_value - initial_left_encoder_value) / elapsed_time
        right_speed = -((right_encoder_value - initial_right_encoder_value) / elapsed_time)


def st_line():
    global done, target_speed, yellow
    
    # Set the minimum and maximum duty cycle values
    min_duty_cycle = 0
    max_duty_cycle = 50
    global right_encoder_value, left_encoder_value, left_speed, right_speed
    error_l = 0
    error_r = 0
    integral_r = 0
    integral_l = 0
    # Create variables to store the previous error and the current time
    prev_error_r = 0
    prev_error_l = 0
    current_time = time.time()
    forward(50,50)
    while not yellow:
        print('in st_line')
        # Read the current speed of the motor
        # thread will update the global variables
        global left_speed, right_speed        
        # Calculate the error
        error_l = target_speed - left_speed
        error_r = target_speed - right_speed

        # Update the integral_l term
        integral_l += error_l * (time.time() - current_time)
        integral_r += error_r * (time.time() - current_time)

        # Calculate the derivative term
        derivative_r = (error_r - prev_error_r) / (time.time() - current_time)
        derivative_l = (error_l - prev_error_l) / (time.time() - current_time)

        # Calculate the control signal
        control_signal_l = Kp * error_l + Ki * integral_l + Kd * derivative_l
        control_signal_r = Kp * error_r + Ki * integral_r + Kd * derivative_r

        # Update the previous error and the current time
        prev_error_r = error_r
        prev_error_l = error_l
        current_time = time.time()

        # Convert the control signal into a PWM duty cycle
        # Map the control signal to the duty cycle
        duty_cycle_l = min_duty_cycle + (control_signal_l / target_speed) * (max_duty_cycle - min_duty_cycle)
        duty_cycle_r = min_duty_cycle + (control_signal_r / target_speed) * (max_duty_cycle - min_duty_cycle)

        # Limit the duty cycle to the minimum and maximum values
        duty_cycle_l = min(max_duty_cycle, max(min_duty_cycle, duty_cycle_l))
        duty_cycle_r = min(max_duty_cycle, max(min_duty_cycle, duty_cycle_r))

        # Apply the PWM signal to control the motor
        forward(50+duty_cycle_l, 50+duty_cycle_r)
        ...
        # Wait for a short period of time before the next iteration
        time.sleep(0.05)
    motor_stop()

def main():
    # motor_pin_setup()
    global right_encoder_value, left_encoder_value
    global yellow
    # stop()
    motor_pin_setup()
    st_line()
    time.sleep(0.5)
    turn_right()
    time.sleep(0.5)
    st_line()
    time.sleep(0.5)
    turn_left()

    time.sleep(2)
    global done
    done = 1

def yellow_detect():
    print('yellow running')
    global yellow
    global done
    for frame in stream:
        image = frame.array
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        if cv2.countNonZero(mask) > 0:
            yellow = 1
        else:
            yellow = 0
        rawCapture.truncate(0)        
        if (done):
            stop()
            return 0

# Function to handle the left wheel encoder interrupt
def left_encoder_handler(gpio):
    global left_encoder_value, right_encoder_value
    left_encoder_value += 1

# Function to handle the right wheel encoder interrupt
def right_encoder_handler(gpio):
    global right_encoder_value, left_encoder_value
    right_encoder_value -= 1

def motor_pin_setup():
    global L_MOTOR1, L_MOTOR2, R_MOTOR1, R_MOTOR2
    GPIO.setup(R_PWM_PIN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(R_PWM_PIN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(L_PWM_PIN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(L_PWM_PIN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(31, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(37, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(LEFT_WHEEL_GPIO, GPIO.IN)
    GPIO.setup(RIGHT_WHEEL_GPIO, GPIO.IN)
    GPIO.add_event_detect(LEFT_WHEEL_GPIO, GPIO.RISING, callback=left_encoder_handler)
    GPIO.add_event_detect(RIGHT_WHEEL_GPIO, GPIO.RISING, callback=right_encoder_handler)

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
    
def stop():
    L_MOTOR1.stop()
    R_MOTOR1.stop()
    L_MOTOR2.stop()
    R_MOTOR2.stop()
    GPIO.cleanup()

# Function to set wheel speeds
def forward(left_speed, right_speed):
    L_MOTOR1.ChangeDutyCycle(left_speed)
    R_MOTOR1.ChangeDutyCycle(right_speed)

# Function to turn by a specified angle
def turn_right():
    motor_stop()
    print('inright')
    global right_encoder_value, left_encoder_value
    # Calculate the distance that each wheel should travel
    circumference = 2 * math.pi * WHEEL_RADIUS
    distance = WHEEL_SEPARATION * (90*math.pi/180)
    wheel_distance = distance
    # Calculate the number of encoder ticks required to travel the distance
    encoder_ticks = wheel_distance / circumference * ENCODER_RESOLUTION
    # Set the initial encoder values
    initial_right_encoder = right_encoder_value
    # Set the wheel speeds
    forward(40, 0)
    encoder_ticks=int(encoder_ticks)
    # Wait until the desired number of encoder ticks have been travelled
    while True:
        if (right_encoder_value - initial_right_encoder <= -(encoder_ticks)):
            break
    print('turn done')
    # Stop the wheels
    forward(0, 0)
  
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
    forward(0, 40)
    encoder_ticks=int(encoder_ticks)
    # Wait until the desired number of encoder ticks have been travelled
    while True:
        if (left_encoder_value - initial_left_encoder >= encoder_ticks):
            break
    # Stop the wheels
    forward(0, 0)
  
def motor_stop():
    L_MOTOR1.ChangeDutyCycle(0)
    R_MOTOR1.ChangeDutyCycle(0)
    L_MOTOR2.ChangeDutyCycle(0)
    R_MOTOR2.ChangeDutyCycle(0)  


thread1 = threading.Thread(target = main)
thread2 = threading.Thread(target = yellow_detect)
thread3 = threading.Thread(target = get_speeds)

thread1.start()
thread2.start()
thread3.start()

thread1.join()
thread2.join()
thread3.join()
# 
# try:
#     main()
# except KeyboardInterrupt:
#     GPIO.cleanup()

# try:
#     start_threads()
# except Exception as e:
#     print("An error has occurred:", e)


import RPi.GPIO as GPIO
import time
import math
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

'''
IN1	- L_PWM_PIN1
IN2	- L_PWM_PIN2
IN3	- R_PWM_PIN2
IN4	- R_PWM_PIN1

Control theory：

IN1	IN2	IN3	IN4	Description
1	0	0	1	Motors co-roatating and Alphabot move forward
0	1	1	0	Motors reversing and Alphabot move backward.
0	0	0	1	Left motor stops, and the right motor co-rotate. Alphabot turn left.
1	0	0	0	Rightmotor stops, and the left motor co-rotate. Alphabot turn Right.
0	0	0	0	Motors stop and Alphabot stop.

'''

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

# Variables to keep track of the encoder values
left_encoder_value = 0
right_encoder_value = 0

def motor_pin_setup():
    global L_MOTOR1, L_MOTOR2, R_MOTOR1, R_MOTOR2
    GPIO.setup(R_PWM_PIN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(R_PWM_PIN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(L_PWM_PIN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(L_PWM_PIN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(31, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(37, GPIO.OUT, initial=GPIO.HIGH)

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

# Function to handle the left wheel encoder interrupt
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
# 
# GPIO.setup(ENCODER_L_GPIO, GPIO.IN)
# GPIO.setup(ENCODER_R_GPIO, GPIO.IN)
# # GPIO.add_event_detect(ENCODER_L_GPIO, GPIO.RISING, callback=count_rising_edges)
# GPIO.add_event_detect(ENCODER_R_GPIO, GPIO.RISING, callback=count_rising_edges)


# Function to set wheel speeds
def forward(left_speed, right_speed):
    L_MOTOR1.ChangeDutyCycle(left_speed)
    R_MOTOR1.ChangeDutyCycle(right_speed)

# Initialize GPIO


# Function to turn by a specified angle
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
    forward(0, 50)
    encoder_ticks=int(encoder_ticks)
    # Wait until the desired number of encoder ticks have been travelled
    while True:
        if (left_encoder_value - initial_left_encoder >= encoder_ticks):
            break
    # Stop the wheels
    forward(0, 0)
  
  

def main():
    motor_pin_setup()
#     L_MOTOR1.stop()
#     R_MOTOR1.stop()
#     L_MOTOR2.stop()
#     R_MOTOR2.stop()
    turn_right()
    time.sleep(2)
#     turn_left()
# 


try:
    main()
except KeyboardInterrupt:
    GPIO.cleanup()
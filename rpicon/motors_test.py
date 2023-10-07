# This code can be used for testing the motors
# attached to the Alpha Bot kit you have received.

# When the code is run in the terminal of Raspberry Pi
# the robot should
# i) go in forward direction at full speed for 2 seconds
# ii) pause for 2 seconds
# iii) go in backward direction at full speed for 2 seconds and stop

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
import time

# initializing the pin numbers where motors are connected
L_PWM_PIN1 = 32
L_PWM_PIN2 = 33
R_PWM_PIN2 = 38
R_PWM_PIN1 = 40

# declare motor pins as output pins
# motors get input from the PWM pins
def motor_pin_setup():
    global L_MOTOR1, L_MOTOR2, R_MOTOR1, R_MOTOR2
    
    GPIO.setmode(GPIO.BOARD)
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
    
# function for moving forward
def forward(left, right):
    L_MOTOR1.ChangeDutyCycle(left)
    R_MOTOR1.ChangeDutyCycle(right)
    
# function for moving backward
def backward(left, right):
    R_MOTOR2.ChangeDutyCycle(left)
    L_MOTOR2.ChangeDutyCycle(right)

# function for pausing the motors
def motor_pause(secs):
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
    GPIO.cleanup()

# main function
if __name__ == "__main__":
    motor_pin_setup()
    forward(100,100)
    time.sleep(2)
    motor_pause(2)
    backward(100,100)
    time.sleep(2)
    # left
    forward(0,100)
    time.sleep(2)
    # rght
    forward(100,0)
    time.sleep(2)
    stop()
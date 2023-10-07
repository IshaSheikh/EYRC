import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
import time

# initializing the pin numbers where motors are connected
L_PWM_PIN1 = 32
L_PWM_PIN2 = 33
R_PWM_PIN2 = 38
R_PWM_PIN1 = 40
#CE0
L_encoder = 24
#CE1
R_encoder = 26

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
    #GPIO.setup(8, GPIO.IN)
    #GPIO.setup(7, GPIO.IN)
    GPIO.setup(L_encoder, GPIO.IN)
    GPIO.setup(R_encoder, GPIO.IN)
    

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
    
def forward(left, right):
    L_MOTOR1.ChangeDutyCycle(left)
    R_MOTOR1.ChangeDutyCycle(right)

def turn_right():
    counter = 0
    speed = 250
    forward(50,0)
    flag = 0

    while(not flag):
        print(GPIO.input(L_encoder), GPIO.input(R_encoder))
        if (GPIO.input(L_encoder)):
            counter = counter + 1
        if (counter > speed):
            flag = 1
    stop()



if __name__ == "__main__":
    motor_pin_setup()
    
  
    turn_right()
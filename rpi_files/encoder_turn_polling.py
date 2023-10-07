import time
import RPi.GPIO as GPIO
import subprocess

# Constants
LEFT_WHEEL_GPIO = 24
RIGHT_WHEEL_GPIO = 26
ENCODER_RESOLUTION = 100
L_PWM_PIN1 = 32
L_PWM_PIN2 = 33
R_PWM_PIN2 = 38
R_PWM_PIN1 = 40

# Setup GPIO
GPIO.setmode(GPIO.BOARD)

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

# Function to set wheel speeds
def forward(left_speed, right_speed):
    L_MOTOR1.ChangeDutyCycle(left_speed)
    R_MOTOR1.ChangeDutyCycle(right_speed)

def stop(secs):
    L_MOTOR1.ChangeDutyCycle(0)
    R_MOTOR1.ChangeDutyCycle(0)
    L_MOTOR2.ChangeDutyCycle(0)
    R_MOTOR2.ChangeDutyCycle(0)

def main():
    # get_encoder_values()
    motor_pin_setup()
    input_voltage = get_input_voltage()
    print("Input voltage: {:.2f} V".format(input_voltage))
    forward(50,50)
    # check_for_turn(ENCODER_RESOLUTION)
    time.sleep(60)
    # check_for_turn(ENCODER_RESOLUTION)
    input_voltage = get_input_voltage()
    print("Input voltage: {:.2f} V".format(input_voltage))
        


def get_input_voltage():
    result = subprocess.run(['vcgencmd', 'measure_volts'], stdout=subprocess.PIPE)
    voltage_str = result.stdout.decode().strip()
    voltage = float(voltage_str.split('=')[1][:-2])
    return voltage




if __name__ == '__main__':
    main()

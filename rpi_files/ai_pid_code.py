import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
import time

import cv2
import numpy as np
import threading

L_PWM_PIN1 = 32
L_PWM_PIN2 = 33
R_PWM_PIN2 = 38
R_PWM_PIN1 = 40

# Initialize the camera
camera = cv2.VideoCapture(0)

def motor_pin_setup():
    global L_MOTOR1, L_MOTOR2, R_MOTOR1, R_MOTOR2
    
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(R_PWM_PIN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(R_PWM_PIN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(L_PWM_PIN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(L_PWM_PIN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(31, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(37, GPIO.OUT, initial=GPIO.HIGH)
#     GPIO.setup(L_encoder, GPIO.IN)
#     GPIO.setup(R_encoder, GPIO.IN)
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

def motor_stop():
    L_MOTOR1.ChangeDutyCycle(0)
    R_MOTOR1.ChangeDutyCycle(0)
    L_MOTOR2.ChangeDutyCycle(0)
    R_MOTOR2.ChangeDutyCycle(0)

def forward(left, right):
    L_MOTOR1.ChangeDutyCycle(left)
    R_MOTOR1.ChangeDutyCycle(right)
# Set the camera resolution
# camera.set(3, 256)
# camera.set(4, 256)

# Initialize the PID coefficients
kp = 0.4
ki = 0
kd = 0

# Initialize the integral and derivative errors
integral_error = 0
derivative_error = 0
previous_error = 0

i=0

motor_pin_setup()


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

def main():
    kp = 1
    ki = 0
    kd = 0

    # Initialize the integral and derivative errors
    integral_error = 0
    derivative_error = 0
    previous_error = 0

    
    while True:
    # Get the image from the camera
        _, image = camera.read()
        # image=image[0:image.shape[0],130:image.shape[1]]
        if image is None:
            continue
        print(image.shape)
        
        if yellow(image):
            motor_stop()
            return 0

        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Threshold the image to detect the white dashed line
        _, threshold = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)
        threshold=255-threshold

        # Find the contours of the thresholded image
        contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # Find the contour with the largest area
            contour = max(contours, key=cv2.contourArea)
            cv2.drawContours(image, [contour], 0, (0,255,0), 3)

            # Find the moments of the contour
            moments = cv2.moments(contour)

            if moments["m00"] != 0:
                # Calculate the center of the contour
                center_x = int(moments["m10"] / moments["m00"])
        #             center_y = int(moments["m01"] / moments["m00"])

                # Draw a circle at the center of the contour
                cv2.circle(image, (center_x, int(image.shape[0]/2)), 6, (0, 0, 255), -1)

                # Calculate the error between the center of the line and the center of the image
                error = image.shape[1] - center_x

                # Calculate the integral error
                integral_error += error

                # Calculate the derivative error
                derivative_error = error - previous_error

                # Calculate the control variable
                control = kp * error + ki * integral_error + kd * derivative_error

                # Send the control variable to the robot's drive system
                # robot.drive(control)

                # Update the previous error
                previous_error = error
                
                control=control-350
                forward(50-(control/image.shape[1]*50),50+(control/image.shape[1]*50))
                
                print(control)
        else:
            # If no contours are found, set the error to zero
            error = 0
            integral_error = 0
            derivative_error = 0
            previous_error = 0
            # and stop the robot
            # robot.drive(0)

        # Show the image
        cv2.imshow("Line Following", image)
        # cv2.imshow("Line threshold", threshold)

        # Exit the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Release the camera
        camera.release()

        # Close all windows
        cv2.destroyAllWindows()


try:
    main()
except KeyboardInterrupt:
    GPIO.cleanup()
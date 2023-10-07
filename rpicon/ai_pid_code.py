import cv2
import numpy as np
import time

# Initialize the camera
camera = cv2.VideoCapture("D:\\EYRC\\rpicon\\test_vid.mp4")

# Set the camera resolution
camera.set(3, 256)
camera.set(4, 256)

# Initialize the PID coefficients
kp = 1
ki = 0
kd = 0

# Initialize the integral and derivative errors
integral_error = 0
derivative_error = 0
previous_error = 0

i=0

while True:
    # Get the image from the camera
    _, image = camera.read()
    # image=image[0:image.shape[0],130:image.shape[1]]
    print(image.shape)

    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Threshold the image to detect the white dashed line
    _, threshold = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)

    # Find the contours of the thresholded image
    contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # Find the contour with the largest area
        contour = max(contours, key=cv2.contourArea)

        # Find the moments of the contour
        moments = cv2.moments(contour)

        if moments["m00"] != 0:
            # Calculate the center of the contour
            center_x = int(moments["m10"] / moments["m00"])
            center_y = int(moments["m01"] / moments["m00"])

            # Draw a circle at the center of the contour
            cv2.circle(image, (center_x, int(image.shape[0]/2)), 6, (0, 0, 255), -1)

            # Calculate the error between the center of the line and the center of the image
            error = 128 - center_x

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
    cv2.imshow("Line threshold", threshold)
    
    print(i)
    i+=1
    # Exit the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera
camera.release()

# Close all windows
cv2.destroyAllWindows()

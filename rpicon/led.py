import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
redPin = 24
gndPin = 23
greenPin = 5
bluePin = 18
#color = input('Enter color: ')
GPIO.setup(gndPin, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(redPin, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(bluePin, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(greenPin, GPIO.OUT, initial=GPIO.LOW)
red = GPIO.PWM(redPin, 50)
blue = GPIO.PWM(bluePin, 50)
green = GPIO.PWM(greenPin, 50)
red.start(0)
blue.start(0)
green.start(0)
rc = 0
bc = 0
gc = 0

color = input('Enter color: ')
pwm_values = {"Red": (255, 0, 0), "Blue": (0, 0, 255), "Green": (0, 255, 0), "Orange": (255, 35, 0), "Pink": (255, 0, 122), "Sky Blue": (0, 100, 100)}
rc = list(pwm_values[color])[0]
gc = list(pwm_values[color])[1]
bc = list(pwm_values[color])[2]
red.ChangeDutyCycle(rc/255*100)
green.ChangeDutyCycle((gc/255*100))
blue.ChangeDutyCycle(bc/255*100)
print(rc, gc, bc)
# red.stop()
# green.stop()
# blue.stop()
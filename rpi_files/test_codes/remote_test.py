import socket
import time
import os, sys
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.cleanup()

def rgb_led_setup():
	redPin = 24
	gndPin = 23
	greenPin = 5
	bluePin = 18

	GPIO.setup(gndPin, GPIO.OUT, initial=GPIO.LOW)
	GPIO.setup(redPin, GPIO.OUT, initial=GPIO.LOW)
	GPIO.setup(bluePin, GPIO.OUT, initial=GPIO.LOW)
	GPIO.setup(greenPin, GPIO.OUT, initial=GPIO.LOW)
	
	global red, blue, green
	red = GPIO.PWM(redPin, 50)
	blue = GPIO.PWM(bluePin, 50)
	green = GPIO.PWM(greenPin, 50)
	red.start(0)
	blue.start(0)
	green.start(0)
	##########################################################

def rgb_led_set_color(color):
	global red, blue, green
	red.stop()
	blue.stop()
	green.stop()
	rc = 0
	bc = 0
	gc = 0
	pwm_values = {"Red": (255, 0, 0), "Blue": (0, 0, 255), "Green": (0, 255, 0), "Orange": (255, 35, 0), "Pink": (255, 0, 122), "Sky Blue": (0, 100, 100)}
	rc = list(pwm_values[color])[0]
	gc = list(pwm_values[color])[1]
	bc = list(pwm_values[color])[2]
	red.start(rc/255*100)
	green.start(gc/255*100)
	blue.start(bc/255*100)
	time.sleep(20)

rgb_led_setup()
rgb_led_set_color('Sky Blue')
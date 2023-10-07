import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
# from gpiozero import RGBLED
import time

    
def main():
    # led = RGBLED(red=14, green=18, blue=15)
    # (255, 35, 0) (255, 0, 122)
    # led.color = (0.45, 0, 0.5)
    # time.sleep(10)
    # led.off()
    rgb_leds_setup()
    led_indicator('Pink_cone, Orange_cylinder')
    time.sleep(3)
    led_stop(1)
    time.sleep(3)
    led_stop(2)



def rgb_leds_setup():
    redPin1 = 8
    bluePin1 = 10
    greenPin1 = 12
    redPin2 = 3
    bluePin2 = 5
    greenPin2= 7
    redPin3 = 19
    greenPin3 = 21
    bluePin3 = 23
    
    GPIO.setup(redPin1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(bluePin1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(greenPin1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(redPin2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(bluePin2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(greenPin2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(redPin3, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(bluePin3, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(greenPin3, GPIO.OUT, initial=GPIO.LOW)
	
    global red1, blue1, green1, red2, blue2, green2, red3, blue3, green3
    red1 = GPIO.PWM(redPin1, 50)
    blue1 = GPIO.PWM(bluePin1, 50)
    green1 = GPIO.PWM(greenPin1, 50)
    red2 = GPIO.PWM(redPin2, 50)
    blue2 = GPIO.PWM(bluePin2, 50)
    green2 = GPIO.PWM(greenPin2, 50)
    red3 = GPIO.PWM(redPin3, 50)
    blue3 = GPIO.PWM(bluePin3, 50)
    green3 = GPIO.PWM(greenPin3, 50)

    red1.start(0)
    blue1.start(0)
    green1.start(0)
    red2.start(0)
    blue2.start(0)
    green2.start(0)
    red3.start(0)
    blue3.start(0)
    green3.start(0)

def rgb_leds_set_color(color, led_num):
    global red1, blue1, green1, red2, blue2, green2, red3, blue3, green3
    rc = 0
    bc = 0
    gc = 0
    pwm_values = {"Red": (255, 0, 0), "Blue": (0, 0, 255), "Green": (0, 255, 0), "Orange": (255, 35, 0), "Pink": (255, 0, 122), "Sky Blue": (0, 100, 100)}
    rc = list(pwm_values[color])[0]
    gc = list(pwm_values[color])[1]
    bc = list(pwm_values[color])[2]

    if led_num == 1:
        red1.ChangeDutyCycle(rc/255*100)
        green1.ChangeDutyCycle(gc/255*100)
        blue1.ChangeDutyCycle(bc/255*100)
    elif led_num == 2:
        red2.ChangeDutyCycle(rc/255*100)
        green2.ChangeDutyCycle(gc/255*100)
        blue2.ChangeDutyCycle(bc/255*100)
    elif led_num == 3:
        red3.ChangeDutyCycle(rc/255*100)
        green3.ChangeDutyCycle(gc/255*100)
        blue3.ChangeDutyCycle(bc/255*100)

def led_indicator(message):
    colours = message.split(',')
    for x in range(len(colours)):
        if ('Orange' in colours[x]):
            rgb_leds_set_color('Orange', x+1)
        elif ('Red' in colours[x]):
            rgb_leds_set_color('Red', x+1)
        elif ('Green' in colours[x]):
            rgb_leds_set_color('Green', x+1)
        elif ('Sky Blue' in colours[x]):
            rgb_leds_set_color('Sky Blue', x+1)
        elif ('Pink' in colours[x]):
            rgb_leds_set_color('Pink', x+1)
        elif ('Blue' in colours[x]):
            rgb_leds_set_color('Blue', x+1)

def led_stop(led_num):
    if led_num == 1:
        red1.stop()
        green1.stop()
        blue1.stop()
    elif led_num == 2:
        red2.stop()
        green2.stop()
        blue2.stop()
    elif led_num == 3:
        red3.stop()
        green3.stop()
        blue3.stop()

        

main()
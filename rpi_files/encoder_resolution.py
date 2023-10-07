import time
import RPi.GPIO as GPIO

# Constants
ENCODER_L_GPIO = 24
ENCODER_R_GPIO = 26

# Function to count rising edges on the encoder A signal
# def count_rising_edges(gpio, level, tick):
def count_rising_edges(gpio):
    global count_rising_edges_counter
    count_rising_edges_counter += 1
    print(count_rising_edges_counter)

# Initialize the counter
count_rising_edges_counter = 0

# Initialize GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(ENCODER_L_GPIO, GPIO.IN)
GPIO.setup(ENCODER_R_GPIO, GPIO.IN)
# GPIO.add_event_detect(ENCODER_L_GPIO, GPIO.RISING, callback=count_rising_edges)
GPIO.add_event_detect(ENCODER_R_GPIO, GPIO.RISING, callback=count_rising_edges)

# Rotate the wheel one complete revolution
time.sleep(5)

# Measure the number of rising edges
encoder_ticks = count_rising_edges_counter

# Calculate the encoder resolution
encoder_resolution = encoder_ticks

# Clean up GPIO
GPIO.remove_event_detect(ENCODER_L_GPIO)
GPIO.cleanup()

print("Encoder resolution:", encoder_resolution)

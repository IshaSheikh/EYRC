import csv

# create a csv file
with open("example.csv", "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Column 1", "Column 2"])
    writer.writerow(["Value 1", "Value 2"])


# appending to a csv file
with open("example.csv", "a", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Value 3", "Value 4"])


'''
Inputs and Outputs to a differential bot:

Inputs - Duty cycle applied to each wheel, power
Outputs - Location of the center of the road contour in every frame

'''

# To find the battery voltage level
import smbus

# Get I2C bus
bus = smbus.SMBus(1)

# Battery voltage register address
address = 0x75

# Read the voltage from the register
voltage = bus.read_word_data(address, 0)

# Convert the voltage to a floating-point value
voltage = (voltage & 0xff) * 256 + (voltage >> 8)
voltage = voltage / 1000.0

# Print the battery voltage
print("Battery voltage: ", voltage, "V")

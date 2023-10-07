import subprocess

def get_input_voltage():
    result = subprocess.run(['vcgencmd', 'measure_volts'], stdout=subprocess.PIPE)
    voltage_str = result.stdout.decode().strip()
    voltage = float(voltage_str.split('=')[1][:-2])
    return voltage

input_voltage = get_input_voltage()
print("Input voltage: {:.2f} V".format(input_voltage))

import minimalmodbus
import time
import serial

instrument = minimalmodbus.Instrument('/dev/ttyUSB0', 1)

# instrument.serial.baudrate = 9600

# Check the current settings:
print(instrument)

# print the current reading every one second
while True:
    # Register number, number of decimals, function code
    temperature = instrument.read_register(1, 2, 4)
    print temperature
    time.sleep(1)

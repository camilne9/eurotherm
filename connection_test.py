import minimalmodbus
import time
import serial

# The first argument is the port and the second argument is the address.
instrument = minimalmodbus.Instrument('/dev/ttyUSB0', 1)

# These settings should be default but I set them explicitly to be safe.
instrument.serial.baudrate = 19200
instrument.serial.parity   = serial.PARITY_NONE

# Check the current settings:
print(instrument)

# print the current reading every one second
while True:
    # Register number, number of decimals, function code
    # Function code 4 means that it is a "holding register" which means
    # values can be read or written... I am not positive that our register is
    # a holding register but it is the most common.
    temperature = instrument.read_register(1, 2, 4)
    print temperature
    time.sleep(1)

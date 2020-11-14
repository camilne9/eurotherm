import minimalmodbus

instrument = minimalmodbus.Instrument("/dev/ttyUSB1", 1)

def convert_to_hex_register(register):
    return int(str((2*register+8000)),16)

register = 3
decimals = 1
read_register_output = instrument.read_register(register, decimals)
print(read_register_output)
read_float_output = instrument.read_float(convert_to_hex_register(register))
print(read_float_output)

writing_value = 15
write_register_output = instrument.write_register(register, writing_value, decimals)
print(write_register_output)
write_float_output = instrument.write_float(convert_to_hex_register(register), writing_value)
print(write_float_output)

read_register_output = instrument.read_register(register, decimals)
print(read_register_output)
read_float_output = instrument.read_float(convert_to_hex_register(register))
print(read_float_output)

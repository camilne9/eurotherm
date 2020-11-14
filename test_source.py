import minimalmodbus

instrument = minimalmodbus.Instrument("/dev/ttyUSB1", 1)

def convert_to_hex_register(register):
    return int(str((2*register+8000)),16)

register = 2
decimals = 0
instrument.read_register(register, decimals)
instrument.read_float(convert_to_hex_register(register))

writing_value = 20
instrument.write_register(register, writing_value, decimals)
instrument.write_float(convert_to_hex_register(register), writing_value)

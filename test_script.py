import time

measured_temperatures = []
setpoint_temperatures = []

print('\n Temperature: \n')
print(myEuro.temperature)

print('\n Working Setpoint: \n')
print(myEuro.workingSetpoint)

# print('\n Ramp Rate: \n')
# print(myEuro.rampRate)

print('\n Target Setpoint: \n')
print(myEuro.setpoint)

measured_temperatures.append(myEuro.temperature)
setpoint_temperatures.append(myEuro.setpoint)

# I set the temperature goal, wait for it to reach it
#for temp in [25, 40, 25]:
#myEuro.setPoint(25)

print(myEuro.workingSetpoint)
print(myEuro.setpoint)

for i in range(30):
    print('\n Working Setpoint: \n')
    print(myEuro.workingSetpoint)
    print('\n Target Setpoint: \n')
    print(myEuro.setpoint)
    time.sleep(10)
    measured_temperatures.append(myEuro.temperature)
    print(measured_temperatures[-1])
    setpoint_temperatures.append(myEuro.setpoint)
    print(setpoint_temperatures[-1])
    i += 1

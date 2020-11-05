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
setpoint_temperatures.append(myEuro.setPoint)

# I set the temperature goal, wait for it to reach it
for temp in [25, 40, 25]:
    myEuro.setPoint(temp)

    print(myEuro.workingSetpoint)

    for i in range(10):
        print('\n Working Setpoint: \n')
        print(myEuro.workingSetpoint)
        print('\n Target Setpoint: \n')
        print(myEuro.setPoint)
        time.sleep(30)
        measured_temperatures.append(myEuro.temperature)
        print(measured_temperatures[-1])
        setpoint_temperatures.append(myEuro.setPoint)
        i += 1



import machine
import time
import MPU6050


led = machine.Pin(25,machine.Pin.OUT)

# CT = time.time()
# 
# led = machine.Pin(25,machine.Pin.OUT)
# 
# # Convert the time to a tuple representing local time
# local_time = time.localtime(CT)
# 
# # Print the local time tuple
# print("Local Time Tuple:", local_time)
# 
# # Extract individual components of the local time tuple
# year, month, day, hour, minute, second, _, _ = local_time
# 
# # Print the date and time components
# 
# theDate = "{:04d}-{:02d}-{:02d}".format(year, month, day)
# theTime = "{:02d}:{:02d}:{:02d}".format(hour, minute, second)
# theDateTime = theDate + theTime


#set Up solenoid PINs
relay1 = machine.Pin(10, machine.Pin.OUT)
relay2 = machine.Pin(11, machine.Pin.OUT)
relay3 = machine.Pin(12, machine.Pin.OUT)
relay4 = machine.Pin(9, machine.Pin.OUT)

#close all solenoids
def closeAllSolenoids():
    print("Closing All Valves")
    relay1.value(0) # Off = 0 ; On = 1
    relay2.value(0) # Off = 0 ; On = 1
    relay3.value(0)
    relay4.value(0)
    time.sleep(1)


closeAllSolenoids()
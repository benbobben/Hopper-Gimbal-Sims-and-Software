import machine
import time
import MPU6050

#set Up solenoid PINs
relay1 = machine.Pin(10, machine.Pin.OUT)
relay2 = machine.Pin(11, machine.Pin.OUT)
relay3 = machine.Pin(12, machine.Pin.OUT)
relay4 = machine.Pin(9, machine.Pin.OUT)
led    = machine.Pin(25,machine.Pin.OUT)
#close all solenoids

relay1.value(1) # Off = 0 ; On = 1
print("1")
time.sleep(3)
relay1.value(0) # Off = 0 ; On = 1
time.sleep(3)

relay2.value(1) # Off = 0 ; On = 1
print("2")
time.sleep(3)
relay2.value(0) # Off = 0 ; On = 1

time.sleep(3)
print("3")
relay3.value(1)
time.sleep(3)
relay3.value(0)

time.sleep(3)
print("4")
relay4.value(1)
time.sleep(3)
relay4.value(0)
time.sleep(3)

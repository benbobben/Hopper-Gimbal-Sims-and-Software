# Gimbal Hysterisis Controller 3/15


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
relay4 = machine.Pin(10, machine.Pin.OUT)
relay2 = machine.Pin(11, machine.Pin.OUT)
relay1 = machine.Pin(12, machine.Pin.OUT)
relay3 = machine.Pin(9, machine.Pin.OUT)

#close all solenoids
def closeAllSolenoids():
    print("Closing All Valves")
    relay1.value(0) # Off = 0 ; On = 1
    relay2.value(0) # Off = 0 ; On = 1
    relay3.value(0)
    relay4.value(0)
    time.sleep(1)


closeAllSolenoids()

# Set up the I2C interface
i2c = machine.I2C(1, sda=machine.Pin(14), scl=machine.Pin(15))

# Set up the MPU6050 class 
mpu = MPU6050.MPU6050(i2c)

# wake up the MPU6050 from sleep
mpu.wake() 

#Initial angles deg
roll  = 0
pitch = 0
yaw   = 0 
dt    = 0.001

gyro_initial = mpu.read_gyro_data()

file_Name = "StepTest_1sFire_9OFF.csv"


# Sleep Set Up time
led.value(1)
time.sleep(1)
led.value(0)
time.sleep(.1)
led.value(1)
time.sleep(.1)
led.value(0)
time.sleep(.1)

# Turn On one Solenoid
relay1.value(1)
u = 1

startTime = time.ticks_ms()
lastTime = 0.0

print("StartTime: " + str(startTime))
print(startTime)
try:
    # continuously print the data
    while True:
    
        gyro = mpu.read_gyro_data()
        currentTime = (time.ticks_ms()-startTime)/1000
        timeStep    = currentTime -lastTime
#         print("Gyro: " + str(gyro)) #degrees per second = body rate = w = p q r; gyro[1] = first body rate
        
        #Get orientation of IMU
#         pitch += (gyro[0]- gyro_initial[0]) *dt
#         roll  += (gyro[1]- gyro_initial[1]) *dt
#         yaw   += (gyro[2]- gyro_initial[2]) *dt
        
        
        pitch += (gyro[0]) *timeStep
        roll  += (gyro[1]) *timeStep
        yaw   += (gyro[2]) *timeStep
        
        print((currentTime))
        print("ROll: " + str(roll))
        print("TimeStep: " + str(timeStep))
        
        if currentTime > 4.0:
            print(currentTime)
            relay1.value(0)
            u = 0
        if currentTime > 10:
            break
        
        with open(file_Name, "a") as f:
            f.write("{0}, {1}, {2} \n".format((str(currentTime)),(str(roll)),(str(u))))
        time.sleep(dt)
        lastTime = currentTime
    
    
except:
    closeAllSolenoids()
    print("STOPPING")
    while True:
        led.value(1)
        time.sleep(2)
        led.value(0)
        time.sleep(2)
    
    
    




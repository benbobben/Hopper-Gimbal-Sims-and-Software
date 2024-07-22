# Gimbal Hysterisis Controller


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
roll = 0
pitch = 0
yaw = 0 
dt = .01

#Control Parameters
desiredRoll = 45#Deg
err = 2

gyro_initial = mpu.read_gyro_data()

file_Name = "HYS_50PSI_0to180Deg_Err10Deg.csv"

file = open(file_Name,"a")


startTime = time.ticks_ms()
lastTime = 0.0
#try:
    # continuously print the data
while True:
    gyro        = mpu.read_gyro_data()
    currentTime = (time.ticks_ms()-startTime)/1000
    timeStep    = currentTime - lastTime
#    print("Gyro: " + str(gyro)) #degrees per second = body rate = w = p q r; gyro[1] = first body rate
    
    #Get orientation of IMU
    pitch += (gyro[0] - gyro_initial[0]) * timeStep
    roll  += (gyro[1] - gyro_initial[1]) * timeStep
    yaw   += (gyro[2] - gyro_initial[2]) * timeStep
#        time.sleep(.1)
    
#    print("Roll: " + str(roll) + "Pitch: " + str(pitch) + "Yaw: " + str(yaw))
    
    if roll < desiredRoll - err :
        print("Positive Fire" + str(roll))
        relay1.value(0) # Off = 0 ; On = 1=
        time.sleep(.01)
        relay3.value(1)
        u = 1
    elif roll > (desiredRoll + err):
        relay3.value(0)
        time.sleep(.01)
        print("Negative Fire" + str(roll))
        relay1.value(1) # Off = 0 ; On = 1
        u = -1
    else:
        relay3.value(0) # Off = 0 ; On = 1
        time.sleep(.01)
        relay1.value(0)
        u = 0
        print("off")
        
        file.write("{0}, {1}, {2}\n".format((str(currentTime)),(str(roll)),(str(u))))
    time.sleep(dt)
    lastTime = currentTime
    
#     
# except:
#    closeAllSolenoids()
#    while True:
#        led.value(1)
#        time.sleep(2)
#        led.value(0)
#        time.sleep(2)
#    print("\nCtrl+C")
#     
    
    




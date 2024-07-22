# Gimbal Hysterisis Controller


import machine
import time
import MPU6050
import random
import math

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
# desiredRoll = 45#Deg
desiredRoll = 0.7854#Rad
err = 0

gyro_initial = mpu.read_gyro_data()

FireDirection = 1
# Generate a random number between 0 and 100


random_number = random.randint(0, 100)

file_Name = "MPH_Controller_Gimbal.csv" + str(random_number) +  ".txt"



#Initialize Tau T
previous_Error = desiredRoll

a = desiredRoll;
X2_T =  0;
c =  0;
b =  0;


alpha   =  28;
beta    =  14.2;


Tau = -(alpha*b + b*beta - alpha*((beta + alpha)*(- 2*beta*c + b**2 + 2*a*beta)/alpha)**(1/2))/(beta**2 + alpha*beta)
T   = b/alpha - ((beta + alpha)*(alpha*b + b*beta - alpha*((beta + alpha)*(- 2*beta*c + b**2 + 2*a*beta)/alpha)**(1/2)))/(alpha*(beta**2 + alpha*beta))


print(Tau)
print(T)

startTime = time.ticks_ms()
lastTime = 0.0

counter = 0

# file = open(file_Name,"a")

while True:
    gyro        = mpu.read_gyro_data()
    currentTime = (time.ticks_ms()-startTime)/1000
    timeStep    = currentTime - lastTime
    print("Gyro: " + str(gyro)) #degrees per second = body rate = w = p q r; gyro[1] = first body rate
    
    #Get orientation of IMU
    pitch += (gyro[0] - gyro_initial[0]) * timeStep
    roll  += (gyro[1] - gyro_initial[1]) * timeStep
    yaw   += (gyro[2] - gyro_initial[2]) * timeStep

    t_k = currentTime
        
    
    
    if FireDirection == 1:
        
        if   t_k <  Tau:
            relay1.value(0) # Off = 0 ; On = 1=
            time.sleep(.01)
            relay3.value(1)
            u = 1
            print("Left")
            
        elif t_k < T: # Turn off Thruster and let friction level out
            relay3.value(0) # Off = 0 ; On = 1
            time.sleep(.01)
            relay1.value(0)
            u = -1
        
        print("Off")
        
        
    elif FireDirection == -1:
        
        if   t_k <  Tau:
            relay3.value(0) # Off = 0 ; On = 1=
            time.sleep(.01)
            relay1.value(1)
            u = -1
            print("Left")
            
        elif t_k < T: # Turn off Thruster and let friction level out
            relay1.value(0) # Off = 0 ; On = 1
            time.sleep(.01)
            relay3.value(0)
            u = 1
        print("Off")
        
    if t_k > T:
        #If outside the current period Recalculate Tau T
        relay3.value(0) # Off = 0 ; On = 1
        time.sleep(.01)
        relay1.value(0)
        u = 0
        
       
        c =  desiredRoll-.01
        
        time.sleep(3)
        
        if counter == 1:
            desiredRoll = 0#Rad
            FireDirection = -1
            
        elif counter == 2:
            desiredRoll = 2*0.7854#Rad
            FireDirection = 1
            
        elif counter == 3:
            desiredRoll = 2 * 0.7854#Rad
            
        a = desiredRoll
        c = abs(c - desiredRoll)
        
        Tau = -(alpha*b + b*beta - alpha*((beta + alpha)*(- 2*beta*c + b**2 + 2*a*beta)/alpha)**(1/2))/(beta**2 + alpha*beta)
        T   = b/alpha - ((beta + alpha)*(alpha*b + b*beta - alpha*((beta + alpha)*(- 2*beta*c + b**2 + 2*a*beta)/alpha)**(1/2)))/(alpha*(beta**2 + alpha*beta))
        
        counter += 1
        print(desiredRoll)
        t_k = 0
        startTime = time.ticks_ms()
#         va_0 =  0;
#         xa_0 =  0;
#         tau = t_k + -(alpha*b + b*beta - alpha*((beta + alpha)*(- 2*beta*c + b**2 + 2*a*beta)/alpha)**(1/2))/(beta**2 + alpha*beta)
#         T   = t_k + b/alpha - ((beta + alpha)*(alpha*b + b*beta - alpha*((beta + alpha)*(- 2*beta*c + b**2 + 2*a*beta)/alpha)**(1/2)))/(alpha*(beta**2 + alpha*beta))
        
        
    else:
        print("ERROR")
#     file.write("{0}, {1}, {2}\n".format((str(currentTime)),(str(roll)),(str(u))))

    time.sleep(dt)
    lastTime = currentTime
    
     







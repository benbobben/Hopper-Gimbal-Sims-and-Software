# Gimbal Hysterisis Controller


import machine
import time
import MPU6050
import random
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


#Control Parameters
desiredRoll = -45 #Deg
err = 0

gyro_initial = mpu.read_gyro_data()
current_time_seconds = time.time()

# Convert the time to a readable string format
current_time_struct = time.localtime()



# Generate a random number between 0 and 100
random_number = random.randint(0, 100)

file_Name = "PIDPWM_50PSI_45Deg" + str(random_number) +  ".txt"


startTime = time.ticks_ms()
lastTime = 0.0
dt       = .01


max_Err = .5
#Initialize PID
previous_Error = desiredRoll
int_Err = 0
Ts      = 0

kp = 32
ki = 20
kd = 13

duty_Cycle = 1
period    = .1
u_ON      = 14.1
param = 1

# with open(file_Name, "a") as f:

# file = open(file_Name,"a")

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
    
    roll_Err  = desiredRoll - roll
    der_Err = (-previous_Error + roll_Err)/timeStep
    int_Err  = int_Err + roll_Err
    
    # PID Controller
    u_Cont    =  roll_Err*kp + kd*der_Err +ki*int_Err*dt
    
#         print(currentTime)
    
    t_k = currentTime
    
    if abs(roll_Err) < max_Err:
        relay3.value(0) # Off = 0 ; On = 1
        time.sleep(.01)
        relay1.value(0)
        time.sleep(1)
        desiredRoll += 45
#         print("Within Errr")
#         break
    else:
        if   t_k <  duty_Cycle * period + Ts:
            #If in the duty cycle, fire thruster
            
            u_Disc = 1
            
            if param > 0:
                relay1.value(0) # Off = 0 ; On = 1=
                time.sleep(.01)
                relay3.value(1)
            else:
                relay3.value(0) # Off = 0 ; On = 1=
                time.sleep(.01)
                relay1.value(1)
            
        elif t_k < period + Ts:
            #If still within period and outside duty cycle turn off thruster
            u_Disc = 0
            relay3.value(0) # Off = 0 ; On = 1
            time.sleep(.01)
            relay1.value(0)
            
        else:
            #If outside the current period Recalculate duty cycle
            param = (u_Last + u_Cont)/(u_ON) * 1
    #         print("recalculate")
            if param >= 1:
                duty_Cycle  =  1
            elif param <= 1:
                duty_Cycle  =  1
            else:
                duty_Cycle  =  abs(param)
            
            u_Disc = param/abs(param)
        
    
#         print("param: " + str(param))
#         print("u_Cont: " + str(u_Cont))
        
            Ts     = t_k
    
        u_Last = u_Cont
    
        print("currentTime: " + str(currentTime))
        print("Roll: " + str(roll))
    
#         file.write("{0}, {1}, {2}, {3}\n".format((str(currentTime)),(str(roll)),(str(u_Disc)),(str(param)) ))
    
    time.sleep(dt)
    
    previous_Error = roll_Err
    lastTime       = currentTime
    
     






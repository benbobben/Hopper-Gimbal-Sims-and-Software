This folder contains programs for a Raspberry Pi Pico. They are used to control the gimbal system developed in the Air Lab. 

The components controlled include multiple relays to control a solenoid solenoid. The states of the system are informed by an IMU Sensor.

Each file contians a different controller using a state machine controller. 

HysController.py - Simple hysterisis controller using two direction of rotation solenoids
MPH_Controller_Gimbal.py - using only positive rotation solenoid
PIDPWM_1DOF_Gimbal.py - using only positive rotation solenoid
PIDPWM_Controller_2_solenoids - Using both positive and negative rotation solenoids
#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase
from time import sleep

# #Variable Assignments

pm = Motor(Port.A) #Pickup Motor
lm = Motor(Port.B) #Left Motor
rm = Motor(Port.C) #Right Motor

ts = TouchSensor(Port.S1)
#   gs = GyroSensor(Port.S2)
cs = ColorSensor(Port.S3) #Measures light intensity
us = UltrasonicSensor(Port.S4) #Measures distance

# PID tuning
Kp = 1  # proportional gain
Ki = 0  # integral gain
Kd = 0  # derivative gain

integral = 0
previous_error = 0

sp = 100
dt = 500       # milliseconds

done = False
collected = False

target_value = cs.reflection()

robot = DriveBase(lm, rm, 56, 114)

#Play a sound.
brick.sound.beep()

#Function Assignments
def slam(): #What you would expect.
    pm.run_time(-400, 1000)

def forward(sp,t):
    robot.drive(sp, t)

#Start Main Loop

while not done:
    # Calculate steering using PID algorithm
    error = target_value - cs.reflection()
    integral += (error * dt)
    derivative = (error - previous_error) / dt

    # u zero:     on target,  drive forward
    # u positive: too bright, turn right
    # u negative: too dark,   turn left

    u = (Kp * error) + (Ki * integral) + (Kd * derivative)

    #limit u to safe values (between -1000 and 1000)
    if sp + abs(u) > (sp * 11):
        if u >= 0:
            u = (sp * 11) - sp
        else:
            u = sp - (sp * 11)

    print("Light level:", cs.reflection())
    print("Distance:", us.distance())
#    print("Color:", cs.color())
#    print("Ambient light:", cs.ambient())

    if u >= 0: #Go right if too bright.
        if not collected:
            if us.distance() < 40:
                slam()
                collected = True
        lm.run_time(sp + u, dt)
        rm.run_time(sp - u, dt)
        wait(5)

    else: #Go left if too dark.
        if not collected:
            if us.distance() < 40:
                slam()
                collected = True
        lm.run_time(sp - u, dt) 
        rm.run_time(sp + u, dt)
        wait(5)
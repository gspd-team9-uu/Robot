#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase
from time import sleep
from machine import Timer

pm = Motor(Port.A) #Pickup Motor
lm = Motor(Port.B) #Left Motor
rm = Motor(Port.C) #Right Motor

#Play a sound.
brick.sound.beep()
done = False


#Function Assignments
def slam(): #What you would expect.
    pm.run_time(-400, 1000)

def slamresverse():
    pm.run_time(400, 1000)

def forward(sp,t):
    robot.drive(sp, t)

#Truen right and left
def turnleft():
    lm.run_time(-300,1450)
    wait(1000)

def turnright():
    lm.run_time(300,1450)
    wait(1000)
    

def waitingOrder():
    command =1
    if command == 1:
        wait(1000)
        turnright()
        return
        

#Start Main Loop
def main():
    # #Variable Assignments
    lm = Motor(Port.B) #Left Motor
    rm = Motor(Port.C) #Right Motor


    #pm = Motor(Port.A) #Pickup Motor
    #lm = Motor(Port.B) #Left Motor
    #rm = Motor(Port.C) #Right Motor

    robot = DriveBase(lm, rm, 56, 114)

    ts = TouchSensor(Port.S1)
    #   gs = GyroSensor(Port.S2)
    cs = ColorSensor(Port.S3) #Measures light intensity
    us = UltrasonicSensor(Port.S4) #Measures distance

    # PID tuning
    Kp = 10  # proportional gain
    Ki = 0.1  # integral gain
    Kd = 0.5 # derivative gain
    
    integral = 0
    previous_error = 0
    colorcomand = 0
    counterTime = 0  

    Tp = 200     #Target Power
    offset = cs.reflection() 
    
    collected = False
    

    while not done:
        # Calculate steering using PID algorithm
        error = offset - cs.reflection()
        integral = (integral + error)
        derivative = (error - previous_error)
        Turn = (Kp*error) + (Ki*integral) + (Kd*derivative)

        powerA = Tp + Turn 
        powerB = Tp - Turn

        rm.run(powerA)
        lm.run(powerB)
        #colorcomand = cs.color() 


        #limit u to safe values (between -1000 and 1000)
        #print("Light level:", cs.reflection())
        #print("Distance:", us.distance())
        #print("color", cs.color())
        #print("Color:", cs.color())
        

    
        if not collected:
            if us.distance() < 40:
                slam()
                collected = True
       # if collected:
           # counterTime = counterTime + 1
            #print("Timer:", counterTime)
           # if counterTime == 50:
            #    turnright()
            #    slamresverse()
           #     turnleft()
            #    counterTime = 0
                #reversing = True
                #while reversing:
                   # rm.run(-powerB)
                   # lm.run(-powerA)
                    #counterTime = counterTime + 1
                    #while 300 < counterTime < 330:
                        #reversing = False
                        #counterTime = counterTime + 1
                       # rm.run(70)
                        #lm.run(70)
                       # turnright()
                       # slamresverse()
                       # rm.run(-120)
                       # lm.run(-120)
                       # turnleft()
                       # counterTime = 0
                       # collected = False
                        

        
        previous_error = error

if __name__ == '__main__':
    main()
    
    

    
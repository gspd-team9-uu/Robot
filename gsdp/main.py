#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase
from time import sleep

pm = Motor(Port.A) #Pickup Motor
lm = Motor(Port.B) #Left Motor
rm = Motor(Port.C) #Right Motor

#Play a sound.
brick.sound.beep()
done = False


#Function Assignments
def slam(): #What you would expect.
    pm.run_time(-400, 1000)

def forward(sp,t):
    robot.drive(sp, t)

#Truen right and left
#def turnleft():
 #   rm.run_time(300,1450)
  #  wait(1000)

#def turnright():
 #   lm.run_time(300,1450)
  #  wait(1000)
    

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
    Kp = 27  # proportional gain
    Ki = 1.5  # integral gain
    Kd = 1.7   # derivative gain
    
    integral = 0
    previous_error = 0
    colorcomand = 0 #checking color 

    Tp = 150     #Target Power
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
        #colorcomand = cs.color() 


        #limit u to safe values (between -1000 and 1000)
        print("Light level:", cs.reflection())
        print("Distance:", us.distance())
        #print("color", cs.color())
    #    print("Color:", cs.color())
    #    print("Ambient light:", cs.ambient())

    
        if not collected:
            if us.distance() < 40:
                slam()
                collected = True
        
        rm.run(powerA)
        lm.run(powerB)
        
        previous_error = error

if __name__ == '__main__':
    main()
    
    

    
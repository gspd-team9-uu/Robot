# Global Software Product Development

# !/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import TouchSensor, GyroSensor
from ev3dev2.led import Leds
from time import sleep

# Variable Assignments

ts = TouchSensor(INPUT_1)
leds = Leds()
lift = LargeMotor(OUTPUT_A)  # Forklift
mL = LargeMotor(OUTPUT_B)  # Left Motor
mR = LargeMotor(OUTPUT_C)  # Right Motor

print('Hello, my name is EV3!')
Sound.speak('Hello, my name is EV3!').wait()
mL.run_to_rel_pos(position_sp=840, speed_sp=250)
mR.run_to_rel_pos(position_sp=-840, speed_sp=250)
mL.wait_while('running')
mR.wait_while('running')

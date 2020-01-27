#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase
import random


#Initialize the Ultrasonic sesnor. It is used to detect obstacles as the robot drives around.
obstacle_sensor = UltrasonicSensor (Port.S1)


# Initialize the Color Sensor. It is used to detect the color of the objects.
color_sensor = ColorSensor(Port.S4)

# Initialize the two motors
left_motor = Motor (Port.A)
right_motor = Motor (Port.B)

#the wheel diameter is 56 millimeters
wheel_diameter = 56

#the axle track is the distance between the centers of each of the wheels, for this it's 114 millimeters
axle_track = 114

#the drivebase is composed of two motors, with a wheel on each motor.
#the wheel_diameter and axle_track values are used to make the motors move at the correct speed when you give a motor command
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)


#the following loop makes the robot drive foreward until it detects an obstacle. Then it backs up and turns around. It keeps on doing this until you stop the program.
while True: 
    #don't move
    robot.stop()
    while color_sensor.color() == Color.BLACK: 
        #begin driving foreward at 200 millimeters per second
        robot.drive(200, 0)
        #wait until an obstacle is detected. This is done by repeatedly doing nothing (waiting for 10 milliseconds) while the measured distance is still greater than 20 mm
        while obstacle_sensor.distance() < 20:
            wait (10)
            robot.drive_time(0, 90, 200)
    while color_sensor.color() == Color.YELLOW:
        rint = random.randint(1, 3)
        if (rint == 1):
            robot.drive_time(0, 90, 200)
        else:
            robot.drive (200, 0)#drive backward at 100 millimeters per second. Keep going for 2 seconds.
robot.drive_time(-100, 0, 200)
#Turn around at 60 degrees per second, around the midpoint between the wheels. Keep gong for 2 seconds.
robot.drive_time(0, 60, 200)

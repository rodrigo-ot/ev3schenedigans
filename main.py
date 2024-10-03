#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# Create your objects here.
ev3 = EV3Brick()
sensor = ColorSensor(Port.S4)
sensor_toque = TouchSensor(Port.S3)

motor_esq = Motor(Port.A)
motor_dir = Motor(Port.D)
motor_garra = Motor(Port.C)

robot = DriveBase(motor_esq, motor_dir, wheel_diameter = 43.2, axle_track = 115)

DRIVE_SPEED = 100
VELOCIDADE_GARRA = 600
LIMITE_GARRA = 40

def voltarInicio():
    robot.turn(180)
    robot.straight(robot.distance())


def usarGarra(velocidade):
    if velocidade >= 0:
        while not sensor_toque.pressed():
            motor_garra.run(VELOCIDADE_GARRA)
        ev3.speaker.beep(300,200)
        motor_garra.stop()
    else:
        motor_garra.run_until_stalled(velocidade, then=Stop.BRAKE, duty_limit = LIMITE_GARRA)

def procuraVerde():
    while True:
        if sensor.color() != Color.GREEN:
            robot.drive(DRIVE_SPEED, 0)
        else:
            robot.stop()
            ev3.speaker.beep(250, 200)
            break
    wait(500)


# #ajusta a pos inicial da garra
usarGarra(VELOCIDADE_GARRA)

# # #anda reto até achar a cor verde
procuraVerde()
# #ajusta distancia até o obstaculo
# robot.straight(50)
# #agarra obstaculo
usarGarra(-VELOCIDADE_GARRA)


#360 perfeito
voltarInicio()
usarGarra(VELOCIDADE_GARRA)
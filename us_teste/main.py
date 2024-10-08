#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


ev3 = EV3Brick()
sensor = UltrasonicSensor(Port.S4)

PASSO = 10
MOVIMENTOS = {[0,1], [1,0],[0,-1],[-1,0]}

pos_atual = [0,0]
lado_atual = 0 # x%4
ultimos_passos = {}

def moverFrente(passo)
    # move passo para frente
    # append passo em ultimos_passos
def moverRe(passo)
    # move passo para frente
    #remove ultimo passo em ultimos_passos

def temCaminho()
    # retorna true para caminho livre e false para caminho obstruido



while True:
    print((sensor.distance())/10)

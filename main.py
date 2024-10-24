#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


ev3 = EV3Brick()
ultra_sensor = UltrasonicSensor(Port.S4)
color_sensor = ColorSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S2)

motor_esq = Motor(Port.C)
motor_dir = Motor(Port.D)
motor_garra = Motor(Port.B)

robot = DriveBase(motor_esq, motor_dir, wheel_diameter = 43.2, axle_track = 122.1231)

DRIVE_SPEED = 100
VELOCIDADE_GARRA = 600
LIMITE_GARRA = 40

PASSO = 10
#esquerda, cima, direita, baixo
DIRECOES = [0,1,2,3]
PASSOS = [[-1,0],[0,1],[1,0],[0,-1]]


pos_atual = [0,0]
dir_atual = 1
#0 norte, 1 leste, 2 sul, 3 oeste

#[[x,y]],
ultimas_coord = []

#[[x,y],]
coords_disponiveis = []

#[[[x1,y1],[x2,y2]], [[x1,y1],[x2,y2]], ...]
caminhos_possiveis = []


#não vou me lembrar como isso funciona, mas segue a lógica de caminhos de um grafo
#guardando nos caminhos_possiveis as coordenadas do ponto atual para o ponto da frente
def temCaminho():
    coord_frente = [x1 + x2 for x1, x2 in zip(pos_atual, PASSOS[DIRECOES[dir_atual%4]])]
    if ultra_sensor.distance()/10 > PASSO:
        if not passo_frente in ultimas_coord or passo_frente in coords_bloqueadas:
            coords_disponiveis.append(coord_frente)
            caminhos_possiveis.append([pos_atual, coord_frente])
        return True
    coords_bloqueadas.append([x1 + x2 for x1, x2 in zip(pos_atual, PASSOS[dir_atual%4])])
    return False


def voltarPasso():
    while(dir_atual%4 != (ultimas_coord[-1][1])%4):
        viraEsquerda()
    robot.straight(ultimas_coord[-1][0])
    ultimas_coord.pop(-1)

def voltarInicio():
    ev3.speaker.say(str(len(ultimas_coord)))
    while(len(ultimas_coord) >= 1):
        voltarPasso()

def usarGarra(velocidade):
    if velocidade >= 0:
        motor_garra.run_until_stalled(velocidade, then=Stop.BRAKE, duty_limit = LIMITE_GARRA)
        ev3.speaker.beep(300,200)
        motor_garra.stop()
    else:
        motor_garra.run_until_stalled(velocidade, then=Stop.BRAKE, duty_limit = LIMITE_GARRA)

def procuraVerde():
    if color_sensor.color() == Color.GREEN:
        ev3.speaker.beep(250, 200)
        return True
    return False

# metodo pra calcular o angulo da curva
def calculaTurn(dir_desejada):
    global dir_atual

def viraEsquerda():
    global dir_atual
    gyro_sensor.reset_angle(0)
    robot.turn(-90)
    if gyro_sensor.angle() > -90:
        robot.turn(-90+gyro_sensor.angle())
    dir_atual -= 1

def viraDireita():
    global dir_atual
    gyro_sensor.reset_angle(0)
    robot.turn(90)
    if gyro_sensor.angle() > 90*1.05:
        robot.turn(-1*(gyro_sensor.angle() - 90))
    elif gyro_sensor.angle < 90*0.95:
        robot.turn(90 - gyro_sensor.angle())
    dir_atual += 1    

def moverFrente():
    robot.reset()
    robot.drive(DRIVE_SPEED, 0)
    # ultimas_coord.append([x1 + x2 for x1, x2 in zip(pos_atual, PASSOS[DIRECOES[dir_atual%4]])])
    # move passo para frente

def procuraCaminho():
    while(True):
        viraDireita
        



def salva():
    usarGarra(-VELOCIDADE_GARRA)
    voltarInicio()
    usarGarra(VELOCIDADE_GARRA)

def checarArredores():
    robot.turn(120)

def navegar():
    procuraCaminho()
    while(True):
        if procuraVerde():
            ultimas_coord.append([robot.distance(), dir_atual+2])
            robot.stop()
            salva()
            break
        if not temCaminho():
            robot.stop()
            ultimas_coord.append([robot.distance(), dir_atual+2])
            procuraCaminho()

def main():
    usarGarra(VELOCIDADE_GARRA)
    navegar()

def teste():
    navegar()



if __name__ == "__main__":
    # main()
    espiral()
    # teste()
    # viraEsquerda()
    # viraDireita()

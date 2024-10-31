#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from Caminho import Caminho
from Coordenada import Coordenada
# import Direcao

ev3 = EV3Brick()
ultra_sensor = UltrasonicSensor(Port.S4)
color_sensor = ColorSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S2)
motor_esq = Motor(Port.C)
motor_dir = Motor(Port.D)
motor_garra = Motor(Port.B)
robot = DriveBase(motor_esq, motor_dir, wheel_diameter = 43.2, axle_track = 122.1231)
robot.settings()
DRIVE_SPEED = 100
VELOCIDADE_GARRA = 600
LIMITE_GARRA = 40
PASSO = 300 #mm

PASSO_LESTE = Coordenada(-1,0)
PASSO_NORTE = Coordenada(0,1)
PASSO_OESTE = Coordenada(1,0)
PASSO_SUL = Coordenada(0,-1)
PASSOS = [PASSO_LESTE, PASSO_NORTE, PASSO_OESTE,PASSO_SUL] #array auxiliar i%4

pos_atual = Coordenada(0,0)
dir_atual = 1

ultimas_coord       = []
coords_disponiveis  = []
caminhos_possiveis  = [] 

def salvaCaminho(caminho):
    caminhos_possiveis.append(caminho)

def caminhoRedundante(caminho):
    for i in caminhos_possiveis:
        if i == caminho:
            return True
    return False

def coordenadaExplorada(coordenada):
    for i in ultimas_coord:
        if coordenada == i:
            return True
        return False

def temCaminho():
    coord_frente = pos_atual + PASSOS[dir_atual%4]
    caminho_frente = Caminho(pos_atual,coord_frente)
    if ultra_sensor.distance()*10 > PASSO:
        #se o caminho nao foi guardado, guarde
        if not caminhoRedundante(caminho_frente):
            salvaCaminho(caminho_frente)
        #se a coordenada nao foi explorada salve em coords disponiveis
        if not coordenadaExplorada(coord_frente):
            coords_disponiveis.append(coord_frente)

        return True
    return False

def turnCorrecao(angulo):
    gyro_sensor.reset_angle(0)  # Reseta o ângulo do giroscópio para 0
    robot.turn(angulo)  # Executa a rotação desejada

    # Lê o ângulo após a rotação
    angulo_atual = gyro_sensor.angle()

    # Corrige se o ângulo atual for diferente do desejado
    if angulo_atual > angulo*1.05:
        # Se girou mais do que o necessário, ajusta para a esquerda
        ajuste = angulo_atual - angulo
        robot.turn(-ajuste)  # Ajuste no sentido inverso
    elif angulo_atual < angulo*0.95:
        # Se girou menos do que o necessário, ajusta para a direita
        ajuste = angulo - angulo_atual
        robot.turn(ajuste)  # Ajuste no sentido normal


def viraParaCoord(coord):
    global dir_atual

    # calcula a diferença de coordenadas entre coord e pos_atual
    diferenca = coord - pos_atual

    # determina a direção alvo com base na diferença
    #melhorar isso com um iterador e os arrays de passos predefinidos
    if diferenca == PASSO_LESTE:
        direcao_alvo = 0
    elif diferenca == PASSO_NORTE:
        direcao_alvo = 1
    elif diferenca == PASSO_OESTE:
        direcao_alvo = 2
    elif diferenca == PASSO_SUL:
        direcao_alvo = 3
    else:
        return False
    # Calcula o ângulo necessário para alinhar a direção
    diferenca_direcao = (direcao_alvo - dir_atual%4)
    angulo = diferenca_direcao * 90  # Cada passo é uma rotação de 90 graus
    # Atualiza a direção atual e vira o robô
    dir_atual = direcao_alvo
    turnCorrecao(angulo)
    return True

def viraParaDir(direcao):
    global dir_atual
    diferenca_dir = direcao - dir_atual%4
    angulo = diferenca_dir * 90
    turnCorrecao(angulo)
    dir_atual = direcao

def moverParaCoord(coord):
    if viraParaCoord(coord):
        robot.straight(PASSO)
        pos_atual = coord
        ultimas_coord.append(coord)

def voltarPasso():
    if ultimas_coord:
        ultima_coord = ultimas_coord[-1]
        moverParaCoord(ultima_coord)
        ultimas_coord.pop(-1)

def checarArredores():
    global pos_atual
    global dir_atual
    dir_original = dir_atual
    #olha para os quatro lados e checa se temCaminho()
    for i in range(len(PASSOS)):
        viraParaDir(i)
        temCaminho()
    viraParaDir(dir_original)

#está errado
def ehAdjascente(coord):
    diferenca = coord - pos_atual
    if diferenca.x <= 1 and diferenca.y <= 1:
        return True
    return False

def navegar():
    ultimas_coord.append(Coordenada(0,0))
    #utiliza todas as funções de movimento para explorar o ambiente em busca do verde
    while(True):
        checarArredores()
        if coords_disponiveis:
            for i in coords_disponiveis:
                if ehAdjascente(i):
                    if not i in ultimas_coord:
                        coord_selecionada = i
                        coords_disponiveis.remove(i)  # Seleciona a primeira coordenada disponível
                        moverParaCoord(coord_selecionada)  # Mover para a nova coordenada
        else: 
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

def voltarInicio():
    #
    pass


def salva():
    usarGarra(-VELOCIDADE_GARRA)
    voltarInicio()
    usarGarra(VELOCIDADE_GARRA)

def main():
    usarGarra(VELOCIDADE_GARRA)
    navegar()




if __name__ == "__main__":
    main()
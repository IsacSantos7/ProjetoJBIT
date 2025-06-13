#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# Inicialização dos componentes
ev3 = EV3Brick()
motor_esquerdo = Motor(Port.D)
motor_direito = Motor(Port.A)
sensor_esq = ColorSensor(Port.S1)
sensor_dir = ColorSensor(Port.S2)

robo = DriveBase(motor_esquerdo, motor_direito)

# Parâmetros do PID
Kp = 10
Ki = 0.0
Kd = 0.5
erro_anterior = 0
integral = 0

while True:
    # Leitura das componentes RGB dos sensores
    rgb_esq = sensor_esq.rgb()
    rgb_dir = sensor_dir.rgb()

    # Calcular a média das componentes RGB para cada sensor
    media_esq = sum(rgb_esq) / 3  # Média de R, G e B no sensor esquerdo
    media_dir = sum(rgb_dir) / 3  # Média de R, G e B no sensor direito

    # Calcula o erro: diferença entre as médias de RGB dos dois sensores
    erro = media_dir - media_esq

    # Controle PID
    integral += erro
    integral = max(min(integral, 100), -100)  # anti-windup
    derivada = erro - erro_anterior
    correcao = Kp * erro + Ki * integral + Kd * derivada
    erro_anterior = erro

    # Velocidade constante e correção baseada no erro
    velocidade = 60
    robo.drive(velocidade, -correcao)

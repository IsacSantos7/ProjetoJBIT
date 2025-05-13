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

robo = DriveBase(motor_esquerdo, motor_direito, wheel_diameter=56, axle_track=114)

# Parâmetros do PID
Kp = 2.0
Ki = 0.0
Kd = 0.4
erro_anterior = 0
integral = 0

# Loop principal
while True:
    # Usar apenas o canal azul (B) dos dois sensores
    _, _, azul_esq = sensor_esq.rgb()
    _, _, azul_dir = sensor_dir.rgb()

    # Cálculo do erro: diferença entre os canais B
    erro = azul_esq - azul_dir

    # PID
    integral += erro
    derivada = erro - erro_anterior
    correcao = Kp * erro + Ki * integral + Kd * derivada
    erro_anterior = erro

    # Movimento
    velocidade = 100
    robo.drive(velocidade, -correcao)

    wait(10)


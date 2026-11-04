import math
from controller import Robot

RAIO_RODA = 0.015   
DIST_RODAS = 0.12   
RAIO_DOHYO = 0.385  
MARGEM_BORDA = 0.20         
VEL = 6.0             
VEL_GIRO = 4.0        
TOLERANCIA_ERRO_ANGULO = 0.08
POSICAO_INICIAL = (0, 0.1, -math.pi/2)


def atualizar_odometria(x, y, theta, ds_esq, ds_dir):
    ds = (ds_dir + ds_esq) / 2.0
    dtheta = (ds_dir - ds_esq) / DIST_RODAS

    x += ds * math.cos(theta + dtheta / 2.0)
    y += ds * math.sin(theta + dtheta / 2.0)
    theta += dtheta

    #normalizar entre pi e -pi
    theta = math.atan2(math.sin(theta), math.cos(theta))

    return x, y, theta


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep()) #passo de controle do simulador(frequência que o controlador troca dados)

    motor_esq = robot.getDevice("motor esq")
    motor_dir = robot.getDevice("motor dir")
    for m in (motor_esq, motor_dir):
        m.setPosition(float("inf"))   #modo velocidade do motor no webots
        m.setVelocity(0.0)

    enc_esq = robot.getDevice("encoder esq")
    enc_dir = robot.getDevice("encoder dir")
    enc_esq.enable(timestep)
    enc_dir.enable(timestep)

    gps = robot.getDevice("gps")    
    gps.enable(timestep)

    robot.step(timestep)            

    x, y, theta = POSICAO_INICIAL
    enc_esq_ant = enc_esq.getValue()
    enc_dir_ant = enc_dir.getValue()
    estado = "AVANCAR"
    ultimo_log = 0.0
    caiu = False

    while robot.step(timestep) != -1:
        enc_esq_atual = enc_esq.getValue()
        enc_dir_atual = enc_dir.getValue()
        ds_esq = (enc_esq_atual - enc_esq_ant) * RAIO_RODA
        ds_dir = (enc_dir_atual - enc_dir_ant) * RAIO_RODA
        enc_esq_ant = enc_esq_atual   
        enc_dir_ant = enc_dir_atual

        x, y, theta = atualizar_odometria(x, y, theta, ds_esq, ds_dir)

        dist_centro = math.hypot(x, y)

        if estado == "AVANCAR" and dist_centro > RAIO_DOHYO - MARGEM_BORDA:
            estado = "VOLTAR"        #perto da borda

        if estado == "VOLTAR":
            alvo = math.atan2(-y, -x)   #direcao do centro
            erro = math.atan2(math.sin(alvo - theta), math.cos(alvo - theta))
            if abs(erro) > TOLERANCIA_ERRO_ANGULO:  #gira no lugar ate apontar pro centro
                motor_esq.setVelocity(-VEL_GIRO)
                motor_dir.setVelocity(VEL_GIRO)
            else:                       #anda reto ate chegar perto do centro
                motor_esq.setVelocity(VEL)
                motor_dir.setVelocity(VEL)
                if dist_centro < 0.5 * RAIO_DOHYO:
                    estado = "AVANCAR"
        else:
            motor_esq.setVelocity(VEL)
            motor_dir.setVelocity(VEL)

        #comparação com gps
        g = gps.getValues()
        x_real, y_real = g[0], g[1]
        dist_real = math.hypot(x_real, y_real)

        if not caiu and dist_real > RAIO_DOHYO:
            caiu = True
            print(f"*** O ROBO CAIU DO DOHYO em t={robot.getTime():.1f}s ***")

        if robot.getTime() - ultimo_log >= 0.5:
            ultimo_log = robot.getTime()
            erro_pos = math.hypot(x - x_real, y - y_real)
            print(f"t={ultimo_log:5.1f}s  odometria=({x:+.3f},{y:+.3f})  |  real=({x_real:+.3f},{y_real:+.3f})  erro={erro_pos*100:.1f} cm")

main()

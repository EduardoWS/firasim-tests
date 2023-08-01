from argparse import ArgumentParser
from typing import Dict, Tuple
import math
import numpy as np
import time
import torch
from communication import FiraSimReader, FiraSimSender

import gym
import robosim
from parameters import *

from utils import TeamColor, TeamSide
from stable_baselines3 import TD3, DDPG

# Supported Models
MODELS ={
    'ddpg': DDPG,
    'td3': TD3,
}

# Contas que determinam o envelope
# tirados do robosim -> get_field_params()
# vss_gym.py do rSoccer

def action_to_vwheels(actions: Tuple[float, float], wheel_radius: float):

    # A saída do agente corresponde a floats entre -1 e 1, que descrevem o quanto (porcentagem)
    # da vel máxima de roda usar na movimentação. Para transformar em velocidades de roda para
    # enviar para o simulador, é necessário fazer continhas de dinâmica de robo diferencial:
    #   - transformar velocidades lineares de roda em velocidades angulares de roda
    #   - o FIRASim espera velocidades angulares como comandos
    
    v_wheel_deadzone = 0.05
    left_wheel_speed = actions[0] * max_v
    right_wheel_speed = actions[1] * max_v

    left_wheel_speed, right_wheel_speed = np.clip(
        (left_wheel_speed, right_wheel_speed), -max_v, max_v
    )

    # Deadzone
    if -v_wheel_deadzone < left_wheel_speed < v_wheel_deadzone:
        left_wheel_speed = 0

    if -v_wheel_deadzone < right_wheel_speed < v_wheel_deadzone:
        right_wheel_speed = 0

    # # Convert to rad/s
    left_wheel_speed /= wheel_radius
    right_wheel_speed /= wheel_radius

    return left_wheel_speed , right_wheel_speed

def mirror_vheels(left_wheel_speed: float , right_wheel_speed: float, wheel_radius: float) -> Tuple[float, float]:

    # Obtendo velocidades de corpo rigido para espelhar. Baseado no código do WRSimActuator para VSSS
    # vy: velocidade linear no eixo y
    # vw: velocidade angular
    vy = (wheel_radius/2) * (left_wheel_speed + right_wheel_speed)
    vw = (-wheel_radius/2) * (left_wheel_speed - right_wheel_speed)
    # Invertendo a vw para espelhar
    vw *= -1
    # Obtendo velocidades de roda novamente
    left_wheel_speed = (vy - vw) / wheel_radius
    right_wheel_speed = (vy + vw) / wheel_radius

    return left_wheel_speed, right_wheel_speed

if __name__ == "__main__":

    parser = ArgumentParser()
    parser.add_argument("--side", type=str, default='right', choices=['left', 'right'], help='Attack field side.')
    parser.add_argument("--color", type=str, default='blue', choices=['blue', 'yellow'], help='Team color.')
    parser.add_argument("--id", type=int, default=0, choices=[0, 1, 2], help='ID of the robot to control.')
    parser.add_argument('--model_path', type=str, required=True, help='Path to the trained policy (pytorch model).')
    parser.add_argument('--model_type', type=str, default='td3',  choices=['ddpg', 'td3'], help='Agent model type.')
    parser.add_argument('--skip', type=int, default=4, help='Frames to skip before every action.')


    args = parser.parse_args()
    # Ajustando enums
    attack_side = TeamSide.RIGHT if args.side == 'right' else TeamSide.LEFT
    team_color = TeamColor.BLUE if args.color == 'blue' else TeamColor.YELLOW



    # Comunicação com FiraSim
    sender = FiraSimSender()
    reader = FiraSimReader()

    print(f'- Loading model {args.model_type} from {args.model_path}')
    print(f'- Actions every {args.skip} frames.')

    model_framework = MODELS[args.model_type]

    full_model = model_framework.load(args.model_path, device='cpu')
    # full_model = DDPG.load(args.model_path)
    actor = full_model.policy.actor
    actor.eval() #modo em que o modelo não aprende
    print(actor)

    iterations_per_second = 60
    interval = 1.0 / iterations_per_second
    i = 1
    
    
    
    # Laço de execução
    while True:

        # Obter o tempo de início da iteração
        start_time = time.time()

        # Actions every skip steps
        if i % args.skip != 0:
            i += 1
            continue

        if i == 1000: 
            i = 1

        

        state = reader.get_observation(attack_side, team_color, args.id)
        next_state_tensor = torch.from_numpy(state).float().unsqueeze(dim=0)

        # Código para serializar modelo em formato compativel com cpp
        # Use torch.jit.trace to generate a torch.jit.ScriptModule via tracing.
        # traced_script_module = torch.jit.trace(modelo, next_state_tensor)
        # traced_script_module.save('data/ddpg_robocin.pt')
        # break

        action = actor(next_state_tensor).detach().numpy()[0] # obter previsão do modelo

        left_speed, right_speed = action_to_vwheels(action, rbt_wheel_radius)
        if attack_side == TeamSide.LEFT:
            left_speed, right_speed = mirror_vheels(left_speed, right_speed, rbt_wheel_radius)
        # print(left_speed, right_speed)



        # Registre o horário do envio dos comandos
        send_time = time.time()

        sender.send_packet(
            team=team_color, id=args.id,
            left_speed=left_speed, 
            right_speed=right_speed
        )

        # Obter o tempo de término da iteração
        end_time = time.time()

        # Calcular o tempo total da iteração
        elapsed_time = end_time - start_time
        
        

        
        

        # Imprimir informações de debug
        #print(f"Iteration: {i}, Elapsed Time: {elapsed_time:.6f} s, Send Time: {send_time:.6f} s")

        i += 1

        # Pausar para manter a taxa de iteração desejada
        time.sleep(max(0, interval - elapsed_time))



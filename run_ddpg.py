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

from agent.parameters_ddpg import *
from agent.agentDDPG import create_actor_model
from utils import TeamColor, TeamSide
from stable_baselines3 import TD3

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
    # parser.add_argument('--model_path', type=str, required=True, help='Path to the trained policy (pytorch model).')


    args = parser.parse_args()
    # Ajustando enums
    attack_side = TeamSide.RIGHT if args.side == 'right' else TeamSide.LEFT
    team_color = TeamColor.BLUE if args.color == 'blue' else TeamColor.YELLOW

    # Comunicação com FiraSim
    sender = FiraSimSender()
    reader = FiraSimReader()

    # Requisitos para carregar o modelo treinado 
    model_params = MODEL_HYPERPARAMS
    action_space = gym.spaces.Box(low=-1, high=1, shape=(2, ), dtype=np.float32)
    observation_space = gym.spaces.Box(low=-NORM_BOUNDS,
                                       high=NORM_BOUNDS,
                                       shape=(40, ), dtype=np.float32)
    
    model_params['state_shape'] = observation_space
    model_params['action_shape'] = action_space
    device = torch.device("gpu")
    modelo = create_actor_model(model_params, observation_space,
                                    action_space, device) # instancia o modelo

    modelo.load_state_dict(torch.load('data/modelo100k.pth', map_location=torch.device('gpu')))
    # modelo.load_state_dict(torch.load('data/modelo.pth', map_location=torch.device('cpu')))
    modelo.eval() # defina o modelo para o modo de avaliação (não treinamento)s
    # print(modelo)
    print(modelo)
    # Laço de execução
    while True:

        state = reader.get_observation(attack_side, team_color, args.id)
        next_state_tensor = torch.from_numpy(state).float()

        # Código para serializar modelo em formato compativel com cpp
        # Use torch.jit.trace to generate a torch.jit.ScriptModule via tracing.
        # traced_script_module = torch.jit.trace(modelo, next_state_tensor)
        # traced_script_module.save('data/ddpg_robocin.pt')
        # break

        action = modelo(next_state_tensor).detach().numpy() # obter previsão do modelo

        left_speed, right_speed = action_to_vwheels(action, rbt_wheel_radius)
        if attack_side == TeamSide.LEFT:
            left_speed, right_speed = mirror_vheels(left_speed, right_speed, rbt_wheel_radius)
        # print(left_speed, right_speed)
        sender.send_packet(
            team=team_color, id=args.id,
            left_speed=left_speed, 
            right_speed=right_speed
        )


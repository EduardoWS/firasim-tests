from typing import Dict, Tuple
import proto.packet_pb2 as packet_pb2
import socket
import math
from utils import *

class SocketInterface:

    def __init__(self):

        self.socket = socket.socket(socket.AF_INET,     # Internet
                                    socket.SOCK_DGRAM)  # UDP


class FiraSimReader(SocketInterface):

    UDP_IP = "224.0.0.1"
    UDP_PORT = 10002

    def __init__(self) -> None:

        super().__init__()
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((FiraSimReader.UDP_IP, FiraSimReader.UDP_PORT))

        self._packet_interface = packet_pb2.Environment()
    
    


    def get_infos_from_id(self, id: int, team: int) -> dict:

        data, _ = self.socket.recvfrom(1024)
        self._packet_interface.ParseFromString(data)

        team_robots = self._packet_interface.frame.robots_yellow if team == 1 \
            else self._packet_interface.frame.robots_blue

        robot = team_robots[id]

        # Podemos usar dataclasses aqui para deixar mais prático

        return {
            "orientation": robot.orientation,
            "x": robot.x,
            "y": robot.y,

            "vorientation": robot.vorientation,
            "vx": robot.vx,
            "vy": robot.vy
        }
    
    def get_observation(self, side: TeamSide, home_team: TeamColor, control_robot_id: int) -> np.array:

        observation = []
        data, _ = self.socket.recvfrom(1024)
        self._packet_interface.ParseFromString(data)

        #print("Data received:", data)

        yellow_robots = self._packet_interface.frame.robots_yellow 
        blue_robots = self._packet_interface.frame.robots_blue
        ball_info = self._packet_interface.frame.ball
        
        # Espelhar campo caso se deseje atacar para o lado esquerdo
        x_correction = 1 if side == TeamSide.RIGHT else -1

        # Preparando array com infos (https://github.com/robocin/rSoccer/blob/main/rsoccer_gym/vss/README.md#vss-v0)
        # Ball
        observation.append(norm_pos(ball_info.x * x_correction))
        observation.append(norm_pos(ball_info.y))
        observation.append(norm_v(ball_info.vx * x_correction))
        observation.append(norm_v(ball_info.vy))

        # Preparando times
        home_team_robots = blue_robots if home_team == TeamColor.BLUE else yellow_robots
        opposing_team_robots = yellow_robots if home_team == TeamColor.BLUE else blue_robots

        # Home Team
        # O robô a ser controlado precisa estar na primeira posição da lista de observações!
        robot_to_control = home_team_robots[control_robot_id]
        home_team_robots.insert(0, robot_to_control)
        del home_team_robots[control_robot_id]

        for robot in home_team_robots:
            theta = robot.orientation
            theta = theta if side == TeamSide.RIGHT else math.pi - theta 
            observation.append(norm_pos(robot.x * x_correction))
            observation.append(norm_pos(robot.y))
            observation.append(math.sin(theta))
            observation.append(math.cos(theta))
            observation.append(norm_v(robot.vx * x_correction))
            observation.append(norm_v(robot.vy))           
            observation.append(norm_w(robot.vorientation * x_correction))
 
        # Opposing Team
        for robot in opposing_team_robots:
            observation.append(norm_pos(robot.x * x_correction))
            observation.append(norm_pos(robot.y))
            observation.append(norm_v(robot.vx * x_correction))
            observation.append(norm_v(robot.vy)) 
            observation.append(norm_w(robot.vorientation * x_correction))
        


        return np.array(observation)
    

class FiraSimSender(SocketInterface):

    UDP_IP = ""
    UDP_PORT = 20011

    def __init__(self) -> None:

        super().__init__()

        try:
            self.socket.connect((FiraSimSender.UDP_IP, FiraSimSender.UDP_PORT))
        except Exception as e:
            print(f"Unable to connect socket. {e}")

        self._packet_interface = packet_pb2.Packet()
    
    
    
    def _create_command(self, team: TeamSide, id: int, left_speed: float, right_speed: float) -> str:
        self._packet_interface = packet_pb2.Packet()
        cmd = self._packet_interface.cmd.robot_commands.add()

        cmd.yellowteam = team.value
        cmd.id = id
        cmd.wheel_left = left_speed
        cmd.wheel_right = right_speed

        return self._packet_interface.SerializeToString()


    def send_packet(self, team: int,
                          id: int, 
                          left_speed: float,
                          right_speed: float) -> None:

        cmd_str = self._create_command(team, id, left_speed, right_speed)

        self.socket.sendall(cmd_str)
    
    def send_ball(self, x: float, y: float, vx: float, vy: float) -> None:
        # Criar um novo pacote para enviar informações da bola
        packet = packet_pb2.Environment()
        ball = packet.frame.ball
        ball.x = x
        ball.y = y
        ball.vx = vx
        ball.vy = vy

        # Serializar o pacote
        cmd_str = packet.SerializeToString()

        # Enviar o pacote
        self.socket.sendto(cmd_str, (FiraSimSender.UDP_IP, FiraSimSender.UDP_PORT))


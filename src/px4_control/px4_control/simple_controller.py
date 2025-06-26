import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import time
import sys
import termios
import fcntl
import os
import math

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configuração do terminal para leitura não bloqueante
        self.fd = sys.stdin.fileno()
        self.oldterm = termios.tcgetattr(self.fd)
        newattr = termios.tcgetattr(self.fd)
        newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
        termios.tcsetattr(self.fd, termios.TCSANOW, newattr)
        self.oldflags = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.oldflags | os.O_NONBLOCK)

        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Variáveis de estado
        self.armed = False
        self.offboard_mode = False
        self.takeoff_active = False
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.yaw_setpoint = 1.57079  # 90 degrees in radians

        # Controle de tempo para pouso
        self.reached_takeoff_height = False
        self.takeoff_reached_time = None

        # Timer para controle
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        # Constantes
        self.MIN_TAKEOFF_HEIGHT = -1.5  # Altura mínima para considerar que o drone decolou
        self.TAKEOFF_HEIGHT = -5.0      # Altura de decolagem em metros

    def vehicle_local_position_callback(self, msg) -> None:
        """
        Callback que atualiza a posição local do veículo.

        Args:
            msg (VehicleLocalPosition): Mensagem recebida com os dados de posição local.
        """
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg) -> None:
        """
        Callback que atualiza o status do veículo.

        Args:
            msg (VehicleStatus): Mensagem recebida com o status atual do drone.
        """
        self.vehicle_status = msg
    
    def publish_offboard_control_heartbeat_signal(self) -> None:
        """
        Publica o sinal de heartbeat necessário para manter o modo offboard ativo.
        Esse sinal deve ser enviado periodicamente.
        """
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float) -> None:
        """
        Publica um TrajectorySetpoint com uma posição desejada e yaw atual.

        Args:
            x (float): Coordenada X desejada (norte).
            y (float): Coordenada Y desejada (leste).
            z (float): Coordenada Z desejada (altura negativa em NED).
        """
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = self.yaw_setpoint
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        # self.get_logger().info(f"Publishing position setpoints: {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """
        Publica um comando de controle geral para o drone.

        Args:
            command (int): Código do comando (ex: armar, mudar modo, pousar, etc).
            **params: Parâmetros adicionais (param1 a param7) definidos pela especificação MAVLink.
        """
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
        
    def engage_offboard_mode(self) -> None:
        """
        Ativa o modo offboard, necessário para controlar o drone via setpoints.
        """
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def arm(self) -> None:
        """
        Envia comando para armar o veículo (ativar os motores).
        """
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')
    
    def disarm(self) -> None:
        """
        Envia comando para desarmar o veículo (desligar os motores).
        """
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')
        
    def move(self, direction: str, distance: float = 1.0) -> None:
        """
        Move o drone em uma direção relativa ao seu corpo.

        Args:
            direction (str): Direção do movimento. Pode ser:
                'f' (frente), 'b' (trás), 'l' (esquerda), 'r' (direita), 
                'u' (cima), 'd' (baixo).
            distance (float): Distância a ser percorrida (em metros). Padrão: 1.0.
        """
        delta_x_body = 0.0
        delta_y_body = 0.0
        delta_z_body = 0.0

        direction = direction.lower()
        if direction == 'f':
            delta_x_body = distance
            self.get_logger().info("Moving forward")
        elif direction == 'b':
            delta_x_body = -distance
            self.get_logger().info("Moving backward")
        elif direction == 'l':
            delta_y_body = -distance
            self.get_logger().info("Moving left")
        elif direction == 'r':
            delta_y_body = distance
            self.get_logger().info("Moving right")
        elif direction == 'u':
            delta_z_body = -distance 
            self.get_logger().info("Moving up")
        elif direction == 'd':
            delta_z_body = distance
            self.get_logger().info("Moving down")
        else:
            self.get_logger().warn(f"Invalid move direction: {direction}")
            return

        if direction in ['f', 'b', 'l', 'r']:
            delta_x_world = delta_x_body * math.cos(self.yaw_setpoint) - delta_y_body * math.sin(self.yaw_setpoint)
            delta_y_world = delta_x_body * math.sin(self.yaw_setpoint) + delta_y_body * math.cos(self.yaw_setpoint)
        else:
            delta_x_world = 0.0
            delta_y_world = 0.0

        target_x = self.vehicle_local_position.x + delta_x_world
        target_y = self.vehicle_local_position.y + delta_y_world
        target_z = self.vehicle_local_position.z + delta_z_body

        # Publicar o novo setpoint
        self.publish_position_setpoint(target_x, target_y, target_z)

    def rotate(self, direction: str) -> None:
        """
        Rotaciona o drone no eixo Z (yaw).

        Args:
            direction (str): Direção da rotação:
                'cc' → sentido anti-horário,
                'c' → sentido horário.
        """
        direction = direction.lower()
        if direction == 'cc':
            # Rotate counter-clockwise
            self.yaw_setpoint -= 0.174533  # 10 graus
            self.get_logger().info("Rotating counter-clockwise")
        elif direction == 'c':
            # Rotate clockwise
            self.yaw_setpoint += 0.174533  # 10 graus
            self.get_logger().info("Rotating clockwise")
        
        self.publish_position_setpoint(self.vehicle_local_position.x, self.vehicle_local_position.y, 
                                           self.vehicle_local_position.z)
    
    def return_to_home(self) -> None:
        """
        Envia comando para retornar à posição inicial de decolagem (RTL).
        Também reseta o yaw para 90 graus.
        """
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.yaw_setpoint = 1.57079
        self.get_logger().info("Return to home command sent")
        
    def takeoff(self) -> None:
        """Inicia o processo de decolagem do drone no modo offboard."""
        if self.offboard_mode and self.takeoff_active:
            self.publish_position_setpoint(0.0, 0.0, self.TAKEOFF_HEIGHT)
            current_altitude = self.vehicle_local_position.z
            if abs(current_altitude - self.TAKEOFF_HEIGHT) < 0.5:
                self.get_logger().info("Altura de decolagem atingida")
                self.takeoff_active = False
        
    def land(self) -> None:
        """
        Envia comando para iniciar o pouso automático do drone.
        """
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Land command sent")

    def timer_callback(self):
        """Função chamada periodicamente pelo timer para processar entradas do usuário e controlar o drone."""
        try:
            key = sys.stdin.read(1)
        except IOError:
            key = None

        # Alterna entre armar/desarmar
        if key == ' ':
            if self.armed:
                self.get_logger().info("Desarmando o veículo")
                self.disarm()
                self.armed = False
                self.offboard_mode = False
                self.takeoff_active = False
            else:
                self.get_logger().info("Armando o veículo")
                self.arm()
                self.engage_offboard_mode()
                self.armed = True
                self.offboard_mode = True

        # Ações com o drone já armado e em modo offboard
        if self.armed and self.offboard_mode:

            # Iniciar decolagem
            if key == 't':
                self.takeoff_active = True
                self.get_logger().info("Iniciando decolagem")
            if self.takeoff_active:
                self.takeoff()

            # Após atingir altitude mínima, habilitar comandos de movimento
            can_move = self.vehicle_local_position.z <= self.MIN_TAKEOFF_HEIGHT and not self.takeoff_active

            if can_move and key:
                key_action_map = {
                    'l': ("Iniciando pouso", self.land),
                    'w': ("Movendo para frente", lambda: self.move('f')),
                    's': ("Movendo para trás", lambda: self.move('b')),
                    'a': ("Movendo para a esquerda", lambda: self.move('l')),
                    'd': ("Movendo para a direita", lambda: self.move('r')),
                    'c': ("Movendo para cima", lambda: self.move('u')),
                    'v': ("Movendo para baixo", lambda: self.move('d')),
                    'z': ("Girando anti-horário", lambda: self.rotate('cc')),
                    'x': ("Girando horário", lambda: self.rotate('c')),
                    'h': ("Return to home", self.return_to_home),
                }
                
                # Mapeia a tecla pressionada para a ação correspondente
                action = key_action_map.get(key)
                if action:
                    msg, func = action
                    self.get_logger().info(msg)
                    func()

        # Publicar heartbeat e setpoints
        self.publish_offboard_control_heartbeat_signal()

def main(args=None) -> None:
    print('Starting offboard control node...')
    time.sleep(13)
 
    print("' ' para armar/desarmar")
    print("'t' para decolar")
    print("'l' para pousar")
    print("'w' para frente")
    print("'s' para trás")
    print("'a' para esquerda")
    print("'d' para direita")
    print("'c' para cima")
    print("'v' para baixo")
    print("'z' para girar anti-horário")
    print("'x' para girar horário")
    print("'h' para retornar para casa")
    
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
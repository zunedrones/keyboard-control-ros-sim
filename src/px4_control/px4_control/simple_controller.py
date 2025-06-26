import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import time
import sys
import termios
import fcntl
import os

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
        self.takeoff_height = -5.0
        self.yaw_setpoint = 1.57079  # 90 degrees in radians

        # Controle de tempo para pouso
        self.reached_takeoff_height = False
        self.takeoff_reached_time = None

        # Timer para controle
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')
        
    def move(self, direction: str):
        if direction.lower() == 'f':
            self.publish_position_setpoint(self.vehicle_local_position.x, self.vehicle_local_position.y + 1, 
                                           self.vehicle_local_position.z)
            self.get_logger().info("Moving forward")
        elif direction.lower() == 'b':
            self.publish_position_setpoint(self.vehicle_local_position.x, self.vehicle_local_position.y - 1, 
                                           self.vehicle_local_position.z)
            self.get_logger().info("Moving backward")
        elif direction.lower() == 'l':
            self.publish_position_setpoint(self.vehicle_local_position.x + 1, self.vehicle_local_position.y, 
                                           self.vehicle_local_position.z)
            self.get_logger().info("Moving left")
        elif direction.lower() == 'r':
            self.publish_position_setpoint(self.vehicle_local_position.x - 1, self.vehicle_local_position.y, 
                                           self.vehicle_local_position.z)
            self.get_logger().info("Moving right")
        if direction.lower() == 'u':
            self.publish_position_setpoint(self.vehicle_local_position.x, self.vehicle_local_position.y, 
                                           self.vehicle_local_position.z - 1.0)
            self.get_logger().info("Moving up")
        elif direction.lower() == 'd':
            self.publish_position_setpoint(self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z + 1.0)
            self.get_logger().info("Moving down")
    
    def rotate(self, direction: str):
        if direction.lower() == 'z':
            # Rotate counter-clockwise
            self.yaw_setpoint -= 0.174533  # 10 graus
            self.publish_position_setpoint(self.vehicle_local_position.x, self.vehicle_local_position.y, 
                                           self.vehicle_local_position.z)
            self.get_logger().info("Rotating counter-clockwise")
        elif direction.lower() == 'x':
            # Rotate clockwise
            self.yaw_setpoint += 0.174533  # 10 graus
            self.publish_position_setpoint(self.vehicle_local_position.x, self.vehicle_local_position.y, 
                                           self.vehicle_local_position.z)
            self.get_logger().info("Rotating clockwise")
    
    def return_to_home(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.get_logger().info("Return to home command sent")

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Land command sent")

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = self.yaw_setpoint
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        # self.get_logger().info(f"Publishing position setpoints: {[x, y, z]}")

    def publish_vehicle_command(self, command, **params):
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

    def timer_callback(self):
        try:
            key = sys.stdin.read(1)
            if key == ' ':
                # Arma ou desarma
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
            elif key == 't' and self.armed and self.offboard_mode:
                # Takeoff
                self.takeoff_active = True
                self.get_logger().info("Iniciando decolagem")
            elif key == 'l' and self.armed and self.offboard_mode:
                # Land
                self.land()
                self.get_logger().info("Iniciando pouso")
                self.takeoff_active = False
            elif key == 'w' and self.takeoff_active and self.offboard_mode:
                # Move front
                self.get_logger().info("Movendo para frente")
                self.move('f')
            elif key == 's' and self.takeoff_active and self.offboard_mode:
                # Move back
                self.get_logger().info("Movendo para trás")
                self.move('b')
            elif key == 'a' and self.takeoff_active and self.offboard_mode:
                # Move left
                self.get_logger().info("Movendo para a esquerda")
                self.move('l')
            elif key == 'd' and self.takeoff_active and self.offboard_mode:
                # Move right
                self.get_logger().info("Movendo para a direita")
                self.move('r')
            elif key == 'c' and self.takeoff_active and self.offboard_mode:
                # Move up
                self.get_logger().info("Movendo para cima")
                self.move('u')
            elif key == 'v' and self.takeoff_active and self.offboard_mode:
                # Move down
                self.get_logger().info("Movendo para baixo")
                self.move('d')
            elif key == 'z' and self.takeoff_active and self.offboard_mode:
                # Rotate counter-clockwise
                self.get_logger().info("Girando anti-horário")
                self.rotate('z')
            elif key == 'x' and self.takeoff_active and self.offboard_mode:
                # Rotate clockwise
                self.get_logger().info("Girando horário")
                self.rotate('x')    
            elif key == 'h' and self.takeoff_active and self.offboard_mode:
                # Return to home
                self.get_logger().info("Retornando para casa")
                self.return_to_home()
        except IOError:
            pass

        # Publica heartbeat constantemente
        self.publish_offboard_control_heartbeat_signal()

        # Publica setpoints de posição se estiver em modo offboard
        if self.offboard_mode and self.takeoff_active:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            current_altitude = self.vehicle_local_position.z
            if abs(current_altitude - self.takeoff_height) < 0.5:
                self.get_logger().info("Altura de decolagem atingida")
                self.takeoff_active = False

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
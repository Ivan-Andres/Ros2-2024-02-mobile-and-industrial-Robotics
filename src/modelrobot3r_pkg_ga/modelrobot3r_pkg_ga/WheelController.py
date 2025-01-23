import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math
import numpy as np
import time

class WheelController(Node):
    def __init__(self):
        super().__init__('wheel_controller')
        
        # Suscriptor para recibir comandos de /debug_pub
        self.subscription = self.create_subscription(
            Twist,
            '/debug_pub',
            self.listener_callback,
            10
        )

        # Suscriptor para recibir nueva posición deseada en /target_position
        self.target_subscription = self.create_subscription(
            Twist,
            '/target_position',
            self.target_position_callback,
            10
        )

        # Publicador para el estado de las juntas
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        # Publicador para publicar en /cmd
        self.cmd_publisher = self.create_publisher(Twist, '/cmd', 10)
        
        # Temporizador para publicar el estado de las juntas y enviar comandos
        self.timer = self.create_timer(0.01, self.publish_joint_states)  # 10 Hz
        
        self.joint_state = JointState()
        self.joint_state.name = [
            'chasis_lwheel_joint', 'chasis_rwheel_joint', 
            'base_chasis_joint_x', 'base_chasis_joint_y', 'base_chasis_joint_yaw'
        ]
        self.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.start_time = time.time()
        
        # Inicializar la posición de las ruedas
        self.lwheel_position = 0.0
        self.rwheel_position = 0.0

        self.eta_dot_d = np.array([[0],
                                   [0],
                                   [0]])  # velocidad deseada
        self.pos_d = np.array([[0],
                               [0],
                               [0]])  # posición deseada

        self.gains = np.array([[1, 0, 0],
                               [0, 1, 0],
                               [0, 0, 1]])

        # Matrices Q y W
        self.Q = np.array([[47.619, -3.8333],
                           [47.619, 3.8333]])
        
        self.W = np.array([[0.0105, 0.0105],
                           [0, 0],
                           [-0.1304, 0.1304]])

        # Inicialización de zeta, eta_dot, y eta
        self.zeta = np.zeros((3, 1))
        self.eta_dot = np.zeros((3, 1))
        self.eta = np.zeros((3, 1))
        self.eta_dot_cmd = np.zeros((3, 1))
        self.zeta_cmd = np.zeros((3, 1))
        self.omega_cmd = np.zeros((2, 1))
        self.phi_d = 0 
        self.eta_error = np.zeros((3, 1))
        self.rho = 0
        self.rho_tol = 0.01
        self.psi = 0

        self.dt = 0.01  # Intervalo de tiempo
        self.last_pos_d = self.pos_d.copy()  # Para verificar cambios en pos_d

    def listener_callback(self, msg):
        # Obtener valores de revoluciones y RPM de las ruedas
        rw_revolutions = msg.linear.x
        rw_rpm = msg.linear.y
        lw_revolutions = -msg.angular.x
        lw_rpm = -msg.angular.y

        # Actualizar la posición de la rueda izquierda
        self.lwheel_position = lw_revolutions * (2 * math.pi)
        self.lwheel_velocity = (lw_rpm / 60) * (2 * math.pi)

        # Actualizar la posición de la rueda derecha
        self.rwheel_position = rw_revolutions * (2 * math.pi)
        self.rwheel_velocity = (rw_rpm / 60) * (2 * math.pi)

        # Actualizar posiciones en el estado de la junta
        self.joint_state.position[0] = self.lwheel_position
        self.joint_state.position[1] = self.rwheel_position

        # Calcular la velocidad del chasis
        omega = np.array([[self.rwheel_velocity], [self.lwheel_velocity]])
        self.zeta = self.W @ omega

        # Matriz de rotación y modelo cinemático
        psi = self.eta[2, 0]

        J_psi = np.array([
            [math.cos(psi), -math.sin(psi), 0],
            [math.sin(psi), math.cos(psi), 0],
            [0, 0, 1]
        ])
        eta_dot = J_psi @ self.zeta

        # Actualizar el estado del chasis
        self.eta = self.eta + self.dt * eta_dot
        self.joint_state.position[2] = self.eta[0, 0]
        self.joint_state.position[3] = self.eta[1, 0]
        self.joint_state.position[4] = self.eta[2, 0]

        # Calcular el error y los comandos de movimiento
        self.calculate_commands()

    def target_position_callback(self, msg):
        # Actualizar la posición deseada solo si es diferente
        new_pos_d = np.array([[msg.linear.x],
                              [msg.linear.y],
                              [msg.angular.z]])  # Suponiendo que la z es 0

        if not np.array_equal(new_pos_d, self.last_pos_d):
            self.pos_d = new_pos_d
            self.last_pos_d = new_pos_d.copy()  # Guardar el nuevo pos_d
            self.get_logger().info(f'Nueva posición deseada recibida: {self.pos_d}')
            self.calculate_commands()  # Ejecutar cálculos si hay un nuevo punto

    def calculate_commands(self):
        self.phi_d = math.atan2(self.pos_d[1, 0] - self.eta[1, 0], self.pos_d[0, 0] - self.eta[0, 0])
        self.eta_error[0, 0] = self.pos_d[0, 0] - self.eta[0, 0]
        self.eta_error[1, 0] = self.pos_d[1, 0] - self.eta[1, 0]
        self.eta_error[2, 0] = self.phi_d - self.eta[2, 0]

        self.rho = math.sqrt((self.eta_error[0, 0])**2 + (self.eta_error[1, 0])**2)

        if self.rho < self.rho_tol:
            self.eta_error = self.pos_d - self.eta

            # Condición de parada
            angle_tolerance = 0.05  # Tolerancia para el ángulo en radianes
            if abs(self.eta_error[2, 0]) < angle_tolerance:
                # Detener el robot
                cmd_msg = Twist()
                cmd_msg.linear.z = 0.0
                cmd_msg.angular.z = 0.0
                self.cmd_publisher.publish(cmd_msg)
                self.get_logger().info("Ángulo alcanzado, deteniendo cálculos y robot.")
                return  # Salir de la función para detener cálculos

        self.psi = self.eta[2, 0]
        J_psi_inv = np.array([
            [math.cos(self.psi), math.sin(self.psi), 0],
            [0, 0, 1],
        ])

        self.eta_dot_cmd = self.eta_dot_d + self.gains @ (self.eta_error)
        self.zeta_cmd = J_psi_inv @ self.eta_dot_cmd
        self.omega_cmd = self.Q @ self.zeta_cmd 

        # Publicar el mensaje de Twist en /cmd
        cmd_msg = Twist()
        cmd_msg.linear.z = (self.omega_cmd[0, 0] * 60) / (2 * math.pi)
        cmd_msg.angular.z = -(self.omega_cmd[1, 0] * 60) / (2 * math.pi)
        self.cmd_publisher.publish(cmd_msg)

    def publish_joint_states(self):
        # Actualizar el timestamp y publicar el estado de la junta
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state)
        self.get_logger().info(f'Publishing Joint States: {self.joint_state.position}')

def main(args=None):
    rclpy.init(args=args)
    node = WheelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

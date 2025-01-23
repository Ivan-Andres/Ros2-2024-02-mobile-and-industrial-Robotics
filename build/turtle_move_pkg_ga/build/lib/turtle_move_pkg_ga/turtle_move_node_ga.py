#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import SetBool
import math

class TurtleMoveNodeGA(Node):
    def __init__(self):
        super().__init__("turtle_move_node_gA")
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subs = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 1)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0
        self.pos_theta = 0.0
        self.diagonal = 0.0
        self.velocity = 1.0  # Velocidad lineal fija (m/s)
        self.angular_velocity = 1.0  # Velocidad angular fija (rad/s)

        # Estados de movimiento
        self.state = 0

        # Crear el servicio para actualizar las coordenadas
        self.update_service = self.create_service(SetBool, 'update_target', self.update_target_callback)

        # Temporizador para controlar la secuencia de movimiento
        self.timer = self.create_timer(0.01, self.timer_callback)


    def update_target_callback(self, request, response):
        if request.data:
            try:
                self.target_x = float(input("Ingrese la nueva coordenada X: "))
                self.target_y = float(input("Ingrese la nueva coordenada Y: "))
                self.target_theta = float(input("Ingrese la nueva orientación Theta: "))
                self.pos_theta =  math.atan2((self.target_y - self.current_pose.y)/(self.target_x - self.current_pose.x))
                self.diagonal = math.sqrt((self.target_x - self.current_pose.x)**2.0+(self.target_y - self.current_pose.y)**2.0)
                self.state = 1  # Iniciar la secuencia de movimientos
                response.success = True
                response.message = "Iniciando movimiento hacia las nuevas coordenadas."
            except ValueError:
                response.success = False
                response.message = "Error al convertir los valores ingresados. Asegúrese de que sean números."
                self.get_logger().error(response.message)
        else:
            response.success = False
            response.message = "Actualización de coordenadas cancelada."
            self.get_logger().info("Actualización de coordenadas cancelada por el usuario.")

        return response

    def timer_callback(self):
        if self.state == 0:
            return

        elif self.state == 1:  # Rotar para alinearse con el eje Y
            if self.pos_theta!= self.current_pose.theta:  # Solo rotar si es necesario
                if self.rotate_turtle(self.pos_theta):
                    self.state = 2
            else:
                self.state = 2 # Saltar la rotación si ya estamos en la posición Y deseada

        elif self.state == 2:  # Mover en X
            if self.move_turtle_x(self.diagonal):
                self.state = 3

        elif self.state == 3:  # Rotar a theta final
            if self.rotate_turtle(self.target_theta):
                self.state = 0  # Movimiento completado
                self.get_logger().info("Secuencia de movimiento completada.")
        

    def move_turtle_x(self, target_x):
        distance = math.sqrt((self.target_x - self.current_pose.x)**2.0+(self.target_y - self.current_pose.y)**2.0)

        if (self.target_x - self.current_pose.x) < 0:
            if (self.target_y - self.current_pose.y) < 0:
                # Movimiento completado
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.0
                self.cmd_pub.publish(cmd_vel)
                return True

        cmd_vel = Twist()
        cmd_vel.linear.x = self.velocity if distance > 0 else -self.velocity
        self.cmd_pub.publish(cmd_vel)
        return False

    def rotate_turtle(self, target_theta):
        angular_diff = target_theta - self.current_pose.theta

        if angular_diff <  0.01:
            # Rotación completada
            cmd_vel = Twist()
            cmd_vel.angular.z = 0.0
            self.cmd_pub.publish(cmd_vel)
            return True

        cmd_vel = Twist()
        cmd_vel.angular.z = self.angular_velocity if angular_diff > 0 else -self.angular_velocity
        self.cmd_pub.publish(cmd_vel)
        return False

    def pose_callback(self, data):
        self.current_pose = data
        msg = 'Posición actual - X: {:.3f}, Y: {:.3f}, Theta: {:.3f}'.format(data.x, data.y, data.theta)
        self.get_logger().info(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleMoveNodeGA()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
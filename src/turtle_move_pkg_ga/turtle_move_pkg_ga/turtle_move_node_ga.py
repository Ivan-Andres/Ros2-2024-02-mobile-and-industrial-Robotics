#!/usr/bin/env python3

 #definicion de librerias de creacion del nodo
import rclpy
from rclpy.node import Node
 #libreria para que la tortuga reciba mensajes tipo twist, estandar de ros2
from geometry_msgs.msg import Twist
 #libreria de msg de turtlesim, que envia un mensaje tipo pose, creada por turtle sim
from turtlesim.msg import Pose
from std_srvs.srv import SetBool
#libreria para realizar los calculos matematicos
import math

#clase nodo
class TurtleMoveNodeGA(Node):
     #define el constructor de la clase, y le asigna un nombre
    def __init__(self):
        super().__init__("turtle_move_node_gA")
        # objeto publicador de mensajes
        # nombre y variables de la clase, metodo con(tipo de mensaje, topic(variable), cuantos mensajes se conservan)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # create a topic subscriber
        # obj (msg_type,topic_name, callback_handler, buffer)
        # nombre del objeto susciber = crea una suscripcion, donde se le da el tipo de variable (pose, conexion al topic que se va a leer, lee la variable por la funcion callback,buffer)
        self.pose_subs = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 1)
        # creamos las variables donde se almacenaran las nuevas coordenadas ingresadas en consola, y se inicializan en 0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0
        self.pos_theta = 0.0
        # creamos e inicializamos la variable que calcula la distancia al punto
        self.diagonal = 0.0
        # creamos dos variables para deinir las velocidades
        self.velocity = 1.0  # Velocidad lineal fija (m/s)
        self.angular_velocity = 1.0  # Velocidad angular fija (rad/s)

        # inicializamos los Estados de movimiento, 0 no se mueve (no se mueve mientras no hallan nuevas coordenadas, y continua si no ha alcanzado el objetivo)
        self.state = 0

        # Crear el servicio para actualizar las coordenadas (se debe de llamar con el comando en la nota al fina en una nueva terminal)
        self.update_service = self.create_service(SetBool, 'update_target', self.update_target_callback)

        # Temporizador para controlar la secuencia de movimiento, llama al metodo timer_callback
        self.timer = self.create_timer(0.00001, self.timer_callback)

    # metodo que llama al servicio desde la termina;
    def update_target_callback(self, request, response):
        # si se requiere el servicio solicitara en la terminal del nodo la informacion requerida
        if request.data:
            try:
                # solicitamos las coordenadas y la orientacion objetivo
                self.target_x = float(input("Ingrese la nueva coordenada X: "))
                self.target_y = float(input("Ingrese la nueva coordenada Y: "))
                # la orientacion se requiere en grados y se convierte automaticamente a rad
                self.target_theta = float(input("Ingrese la nueva orientación Theta en grados entre -180 a 180: "))*(math.pi/180)
                # calculamos la distancia y el anglo al que iniciara a girar.
                self.pos_theta =  math.atan2((self.target_y - self.current_pose.y),(self.target_x - self.current_pose.x))
                self.diagonal = math.sqrt((self.target_x - self.current_pose.x)**2.0+(self.target_y - self.current_pose.y)**2.0)
                self.state = 1  # Iniciar la secuencia de movimientos
                # imprimimos el mensaje indicando que incia la secuencia de movimiento
                response.success = True
                response.message = "Iniciando movimiento hacia las nuevas coordenadas."
            except ValueError:
                # si no los valores no son numericos, indicamos un mensaje de error en el terminal
                response.success = False
                response.message = "Error al convertir los valores ingresados. Asegúrese de que sean números."
                self.get_logger().error(response.message)
        else:
            # si se cancela el requerimiento de los datos imprimimos un mensaje en pantalla
            response.success = False
            response.message = "Actualización de coordenadas cancelada."
            self.get_logger().info("Actualización de coordenadas cancelada por el usuario.")

        return response

    # definicion del metodo que ejecuta los movimientos
    def timer_callback(self):
        # si es un 0, es pq no hay una nueva posicion objetivo, saca el metodo automaticamente y no se mueve la tortuga
        if self.state == 0:
            return

        # si hay un nuevo target iniciamos el movimiento
        elif self.state == 1:  # Mover y rotar al mismo tiempo hacia el objetivo
            # Calcular la distancia al objetivo
            distance = math.sqrt((self.target_x - self.current_pose.x) ** 2.0 + (self.target_y - self.current_pose.y) ** 2.0)

            # Calcular el ángulo objetivo
            target_theta = math.atan2(self.target_y - self.current_pose.y, self.target_x - self.current_pose.x)
            # calculamos el error del angulo actual al angulo objetivo
            angular_diff = target_theta - self.current_pose.theta

            # Ajustar la velocidad angular y lineal
            cmd_vel = Twist()

            # Si la distancia es menor que la tolerancia, ajustar las velocidades a 0, y posicionar la tortuga en el angulo objetivo
            if abs(self.target_x - self.current_pose.x) < 0.01:
                if abs(self.target_y - self.current_pose.y) < 0.01:
                    # Movimiento completado
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    # seteamos las velocidades a 0, para que la tortuga pare
                    self.cmd_pub.publish(cmd_vel)
                    # llamamos al metodo rotar tortuga, y le damos el angulo objetivo
                    if self.rotate_turtle(self.target_theta):
                        self.state = 0  # Movimiento completado
                        self.get_logger().info("Secuencia de movimiento completada.")
                    # sacamos el metodo, para que no vaya a mover la tortuga
                    return True
                    
            # si no se ha alcanzado el objetivo, le indicamos que mueva la tortuga
            cmd_vel = Twist()
            # definimos si la tortuga se debe de mover adelante o atras, segun la distancia
            cmd_vel.linear.x = self.velocity if distance > 0 else -self.velocity
            # definimos si debe de rotar a izquierda o derecha, segun la diferencia angular
            cmd_vel.angular.z = self.angular_velocity if angular_diff > 0 else -self.angular_velocity
            # publicamos los nuevos parametros
            self.cmd_pub.publish(cmd_vel)
            # no retornamos el metodo, para que asi se quede en un bucle hasta lograr el objetivo
        return False

    # metodo de rotar tortuga, recibe el angulo objetivo
    def rotate_turtle(self, target_theta):
        # definimos el error del angulo objetivo
        angular_diff = target_theta - self.current_pose.theta

        #verificamos el cumplimiento del angulo objetivo
        if abs(angular_diff) <  0.0001:
            # Rotación completada
            cmd_vel = Twist()
            cmd_vel.angular.z = 0.0
            self.cmd_pub.publish(cmd_vel)
            # finalizamos la rotacion, y sacamos el metodo, para que pare de girar
            return True

        # de lo contrario no se halla logrado, giramos la tortuga
        cmd_vel = Twist()
        # giramos a izquierda o derecha segun cual sea mas corta al angulo objetivo
        cmd_vel.angular.z = self.angular_velocity if angular_diff > 0 else -self.angular_velocity
        self.cmd_pub.publish(cmd_vel)
        # no retornamos el metodo, para que quede en bucle hasta que se logre el angulo objetivo
        return False
    
    # metodo pose, imprime en pantalla la posicion actual, y actualiza los valores en pantalla de la posicion actual
    def pose_callback(self, data):
        self.current_pose = data
        msg = 'Posición actual - X: {:.6f}, Y: {:.6f}, Theta: {:.6f}'.format(data.x, data.y, data.theta*(180/math.pi))
        self.get_logger().info(msg)

# main que ejecuta el programa
def main(args=None):
    #inicializador del nodo
    rclpy.init(args=args)
    # llama el objeto nodo
    node = TurtleMoveNodeGA()
    # clase que ejecuta un while de control, repite en bucle le ecritura y lectura en la tortuga
    rclpy.spin(node)
    # funcion para parar el programa
    rclpy.shutdown()

if __name__ == "__main__":
    main() #codigo que se ejecuta como una programa main, no manua
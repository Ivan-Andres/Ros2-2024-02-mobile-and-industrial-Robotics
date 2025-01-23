# libreria que toma las direcciones de las carpetas
import os

from ament_index_python.packages import get_package_share_path

#libreria del modulo launch importar los nodos
from launch import LaunchDescription
from launch_ros.actions import Node

# liberia cargar otros nodos
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition

# libreria que lee el archivo sacro
import xacro

# creamos 2 direcciones con el nombre del paquete y del archivo sacro, para cargarlos
pkg_folder = 'modelrobot_pkg_ga'
robot_file = 'robot_car_ga.urdf.xacro'

#funcion del archivo launch
def generate_launch_description():
    # identifica la direccion del paquete en el compilador, y la direccion de el archivo
    pkg_path = os.path.join(get_package_share_path(pkg_folder))
    default_model_path = os.path.join(pkg_path + '/model/' + robot_file)

    # habilita o no la interfaz
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'], description='Flag to enable joint_state_publisher_gui')

    # Process the URDF file, convierte el archivo xacro a archivo urdf
    robot_description_config = xacro.process_file(default_model_path)

    # variable intermedia, que almacena la informacion en un parametro
    params = {'robot_description': robot_description_config.toxml()}

    #nodo que publica el estado del robot
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        #parametros del nodo, el archivo urdf 
        parameters=[params]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    # nodo que publica el estado de los joint o articulaciones
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    # nodo que permite manipular el estado de las articulaciones con el eslide
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # launch y su orden de ejecucion
    return LaunchDescription([
        gui_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
    ])
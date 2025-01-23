#librerias python para la ejecución de multiples nodos

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # definition of the node 
    # adding a namespace to avoid confusion
    # primer objeto que recibe la info del nodo turtlesim
    turtlesim_node_1 = Node(
        package='turtlesim',
        # crea el parametro, para no sobre poner los topics
        namespace='turtlesim1',
        executable='turtlesim_node',
        name='sim'
    )

    # changing the namespace it is possible to start both
    # segundo objeto que recibe la info del nodo turtlesim
    turtlesim_node_2 = Node(
        package='turtlesim',
        namespace='turtlesim2',
        executable='turtlesim_node',
        name='sim'
    )

    # nodo de mimic, que toma la posicion actual de la tortuga 1 y la convierte en velocidad de la tortuga2
    mimic_node = Node(
        package='turtlesim',
        executable='mimic',
        name='mimic',
        remappings=[
            ('/input/pose', '/turtlesim1/turtle1/pose'),
            ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        ]
    )


    #retorno "generate_launch_description", separar por comas, el objeto de cada nodo
    return LaunchDescription([
        turtlesim_node_1,
        turtlesim_node_2,
        mimic_node
    ])